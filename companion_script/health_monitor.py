#!/usr/bin/env python3
"""
Companion Computer Health Monitor for ArduPilot

Sends COMPANION_HEALTH MAVLink messages to the flight controller at a
configurable rate. Supports both serial and UDP connections.

Usage:
    # SITL (UDP):
    python health_monitor.py --device udpout:127.0.0.1:14550

    # Serial (USB):
    python health_monitor.py --device /dev/ttyACM0 --baud 115200

    # UART:
    python health_monitor.py --device /dev/ttyS0 --baud 921600
"""

import argparse
import logging
import os
import signal
import sys
import time

# Must set MAVLINK20 before importing mavutil to use MAVLink 2.0 dialect
os.environ['MAVLINK20'] = '1'

import psutil
from pymavlink import mavutil

# Status flag bit positions
FLAG_THROTTLED = 0x01
FLAG_OVERHEATING = 0x02
FLAG_LOW_MEMORY = 0x04
FLAG_LOW_DISK = 0x08

# Thresholds for status flags
TEMP_THROTTLE_THRESHOLD = 80.0  # Celsius
TEMP_OVERHEAT_THRESHOLD = 85.0  # Celsius
MEMORY_LOW_THRESHOLD = 90  # Percent
DISK_LOW_THRESHOLD = 95  # Percent

# Temperature sensor paths to try (in order of preference)
TEMP_SENSOR_PATHS = [
    '/sys/class/thermal/thermal_zone0/temp',  # Most common (x86, Pi)
    '/sys/class/thermal/thermal_zone1/temp',  # Some systems
    '/sys/class/hwmon/hwmon0/temp1_input',    # Alternative path
    '/sys/class/hwmon/hwmon1/temp1_input',    # Nvidia Jetson
]

log = logging.getLogger(__name__)


class HealthMonitor:
    """Monitors companion computer health and sends MAVLink messages."""

    def __init__(self, device, baud, source_system, source_component):
        self.device = device
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.mav = None
        self.watchdog_seq = 0
        self.running = False
        self._temp_sensor_path = None
        self._temp_warning_logged = False

    def connect(self):
        """Establish MAVLink connection."""
        log.info("Connecting to %s", self.device)
        try:
            self.mav = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                source_system=self.source_system,
                source_component=self.source_component,
                dialect='ardupilotmega'
            )
            log.info("Connected successfully")
            return True
        except Exception as e:
            log.error("Failed to connect: %s", e)
            return False

    def get_cpu_load(self) -> int:
        """Return CPU load percentage (0-100)."""
        try:
            return int(psutil.cpu_percent(interval=None))
        except Exception:
            return 0

    def get_memory_used(self) -> int:
        """Return memory usage percentage (0-100)."""
        try:
            return int(psutil.virtual_memory().percent)
        except Exception:
            return 0

    def get_disk_used(self) -> int:
        """Return disk usage percentage (0-100)."""
        try:
            return int(psutil.disk_usage('/').percent)
        except Exception:
            return 0

    def get_temperature(self) -> int:
        """Return board temperature in celsius * 10, or 0 if unavailable."""
        # Try cached sensor path first
        if self._temp_sensor_path:
            try:
                with open(self._temp_sensor_path, 'r') as f:
                    temp_milli = int(f.read().strip())
                    return temp_milli // 100  # Convert millidegrees to decidegrees
            except (IOError, ValueError):
                self._temp_sensor_path = None

        # Try to find a working sensor
        for path in TEMP_SENSOR_PATHS:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        temp_milli = int(f.read().strip())
                        self._temp_sensor_path = path
                        return temp_milli // 100
                except (IOError, ValueError):
                    continue

        # Try psutil sensors as fallback
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for name, entries in temps.items():
                    if entries:
                        return int(entries[0].current * 10)
        except Exception:
            pass

        if not self._temp_warning_logged:
            log.warning("No temperature sensor found, reporting 0")
            self._temp_warning_logged = True
        return 0

    def get_gpu_load(self) -> int:
        """Return GPU load percentage (0-100), or 255 if unavailable."""
        # Nvidia GPU (nvidia-smi)
        try:
            import subprocess
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=utilization.gpu',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=1
            )
            if result.returncode == 0:
                return int(result.stdout.strip().split('\n')[0])
        except Exception:
            pass

        # Jetson GPU (tegrastats based)
        tegra_path = '/sys/devices/gpu.0/load'
        if os.path.exists(tegra_path):
            try:
                with open(tegra_path, 'r') as f:
                    return int(f.read().strip()) // 10  # Value is 0-1000
            except Exception:
                pass

        return 255  # GPU not available

    def get_status_flags(self, temperature_decidegrees: int,
                         memory_percent: int, disk_percent: int) -> int:
        """Calculate status flags based on current metrics."""
        flags = 0
        temp_celsius = temperature_decidegrees / 10.0

        if temp_celsius > TEMP_THROTTLE_THRESHOLD:
            flags |= FLAG_THROTTLED
        if temp_celsius > TEMP_OVERHEAT_THRESHOLD:
            flags |= FLAG_OVERHEATING
        if memory_percent > MEMORY_LOW_THRESHOLD:
            flags |= FLAG_LOW_MEMORY
        if disk_percent > DISK_LOW_THRESHOLD:
            flags |= FLAG_LOW_DISK

        return flags

    def send_health(self):
        """Collect metrics and send COMPANION_HEALTH message."""
        if not self.mav:
            return False

        cpu_load = self.get_cpu_load()
        memory_used = self.get_memory_used()
        disk_used = self.get_disk_used()
        temperature = self.get_temperature()
        gpu_load = self.get_gpu_load()
        status_flags = self.get_status_flags(temperature, memory_used, disk_used)

        try:
            self.mav.mav.companion_health_send(
                services_status=0,  # Not implemented yet
                watchdog_seq=self.watchdog_seq,
                temperature=temperature,
                cpu_load=cpu_load,
                memory_used=memory_used,
                disk_used=disk_used,
                gpu_load=gpu_load,
                status_flags=status_flags
            )
            self.watchdog_seq = (self.watchdog_seq + 1) % 65536
            log.debug(
                "Sent: cpu=%d%% mem=%d%% disk=%d%% temp=%.1fC gpu=%s flags=0x%02x seq=%d",
                cpu_load, memory_used, disk_used, temperature / 10.0,
                'N/A' if gpu_load == 255 else f'{gpu_load}%',
                status_flags, self.watchdog_seq
            )
            return True
        except Exception as e:
            log.error("Failed to send message: %s", e)
            return False

    def run(self, rate_hz: float):
        """Main loop: send health messages at the specified rate."""
        if not self.connect():
            return 1

        interval = 1.0 / rate_hz
        self.running = True

        # Initialize CPU measurement (first call always returns 0)
        psutil.cpu_percent(interval=None)

        log.info("Sending COMPANION_HEALTH at %.1f Hz", rate_hz)

        while self.running:
            start = time.monotonic()
            self.send_health()
            elapsed = time.monotonic() - start
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)

        log.info("Stopped")
        return 0

    def stop(self):
        """Signal the main loop to stop."""
        self.running = False


def parse_args():
    parser = argparse.ArgumentParser(
        description='Companion Computer Health Monitor for ArduPilot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Connect to SITL via UDP
  %(prog)s --device udpout:127.0.0.1:14550

  # Connect via USB serial
  %(prog)s --device /dev/ttyACM0 --baud 115200

  # Connect via UART with higher rate
  %(prog)s --device /dev/ttyS0 --baud 921600 --rate 2.0
"""
    )
    parser.add_argument(
        '--device', '-d',
        default='udpout:127.0.0.1:14550',
        help='MAVLink connection string (default: %(default)s)'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int, default=115200,
        help='Baud rate for serial connections (default: %(default)s)'
    )
    parser.add_argument(
        '--rate', '-r',
        type=float, default=1.0,
        help='Message send rate in Hz (default: %(default)s)'
    )
    parser.add_argument(
        '--source-system', '-s',
        type=int, default=1,
        help='MAVLink source system ID (default: %(default)s)'
    )
    parser.add_argument(
        '--source-component', '-c',
        type=int, default=191,
        help='MAVLink source component ID (default: %(default)s, MAV_COMP_ID_ONBOARD_COMPUTER)'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable debug logging'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        datefmt='%H:%M:%S'
    )

    monitor = HealthMonitor(
        device=args.device,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component
    )

    def signal_handler(signum, frame):
        log.info("Received signal %d, stopping...", signum)
        monitor.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    return monitor.run(args.rate)


if __name__ == '__main__':
    sys.exit(main())
