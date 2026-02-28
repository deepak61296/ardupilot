# Companion Computer Health Monitor

A MAVLink-based health monitoring system for ArduPilot companion computers. Sends `COMPANION_HEALTH` messages (ID 11061) to the flight controller at a configurable rate.

## Features

- Cross-platform metrics collection using `psutil`
- Supports serial (USB/UART) and UDP connections
- Configurable send rate and MAVLink parameters
- Automatic status flag calculation
- Temperature sensor auto-detection with fallbacks

## Prerequisites

- Python 3.10+
- Custom-built pymavlink with COMPANION_HEALTH message support

## Installation

### 1. Create Conda Environment

```bash
conda create -n companion_health python=3.10 -y
conda activate companion_health
pip install psutil pyserial lxml
```

### 2. Build Custom pymavlink

The standard pymavlink from pip does not include the COMPANION_HEALTH message. You must build it from the ArduPilot repository:

```bash
cd ~/ardupilot/modules/mavlink/pymavlink
MDEF=~/ardupilot/modules/mavlink/message_definitions pip install . -v
```

### 3. Verify Installation

```python
python3 -c "from pymavlink.dialects.v20 import ardupilotmega; print('COMPANION_HEALTH ID:', ardupilotmega.MAVLINK_MSG_ID_COMPANION_HEALTH)"
```

Expected output: `COMPANION_HEALTH ID: 11061`

## Usage

### SITL Testing (Recommended First Step)

1. Start ArduPilot SITL in one terminal:
   ```bash
   cd ~/ardupilot
   ./Tools/autotest/sim_vehicle.py -v Copter --console --map
   ```

2. Run the health monitor in another terminal:
   ```bash
   conda activate companion_health
   python health_monitor.py --device udpout:127.0.0.1:14550 --verbose
   ```

3. In MAVProxy console, monitor messages:
   ```
   message COMPANION_HEALTH
   ```
   Or enable all message logging:
   ```
   set shownoise 1
   ```

### Serial Connection (USB)

```bash
python health_monitor.py --device /dev/ttyACM0 --baud 115200
```

### UART Connection

```bash
python health_monitor.py --device /dev/ttyS0 --baud 921600
```

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--device`, `-d` | `udpout:127.0.0.1:14550` | MAVLink connection string |
| `--baud`, `-b` | `115200` | Baud rate for serial connections |
| `--rate`, `-r` | `1.0` | Message send rate in Hz |
| `--source-system`, `-s` | `1` | MAVLink system ID |
| `--source-component`, `-c` | `191` | MAVLink component ID |
| `--verbose`, `-v` | off | Enable debug logging |

## Message Format

The `COMPANION_HEALTH` message contains:

| Field | Type | Description |
|-------|------|-------------|
| `services_status` | uint32 | Bitmask of service states (0 = not implemented) |
| `watchdog_seq` | uint16 | Sequence counter for watchdog detection |
| `temperature` | int16 | Temperature in celsius * 10 |
| `cpu_load` | uint8 | CPU usage 0-100% |
| `memory_used` | uint8 | RAM usage 0-100% |
| `disk_used` | uint8 | Disk usage 0-100% |
| `gpu_load` | uint8 | GPU usage 0-100%, or 255 if unavailable |
| `status_flags` | uint8 | Health status flags |

### Status Flags

| Bit | Flag | Threshold |
|-----|------|-----------|
| 0 | Throttled | Temperature > 80C |
| 1 | Overheating | Temperature > 85C |
| 2 | Low Memory | RAM usage > 90% |
| 3 | Low Disk | Disk usage > 95% |

## Deployment to Companion Computer

### Raspberry Pi / Jetson

1. Install miniconda on the companion computer
2. Clone ArduPilot repo or copy only the required files:
   - `companion_script/health_monitor.py`
   - `modules/mavlink/pymavlink/` (for custom pymavlink build)
   - `modules/mavlink/message_definitions/` (for building pymavlink)
3. Follow the installation steps above
4. Configure serial connection to flight controller

### Systemd Service (Optional)

Create `/etc/systemd/system/companion-health.service`:

```ini
[Unit]
Description=Companion Computer Health Monitor
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/ardupilot/companion_script
ExecStart=/home/pi/miniconda3/envs/companion_health/bin/python health_monitor.py --device /dev/ttyACM0
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable companion-health
sudo systemctl start companion-health
```

## Troubleshooting

### "No module named 'pymavlink'"
Ensure the conda environment is activated and pymavlink is installed from source.

### "AttributeError: companion_health_send"
The pymavlink was not built with the COMPANION_HEALTH message. Rebuild from source with the MDEF environment variable set correctly.

### "No temperature sensor found"
The script tries multiple sysfs paths. If none work, temperature will report 0. This is expected on some systems.

### Connection refused (UDP)
Make sure SITL or MAVProxy is running and listening on the specified port.

## License

Same as ArduPilot (GPLv3)
