# ODrive Control

Python project for communicating with an ODrive v3.x motor controller over USB.

## Setup

1. Create and activate virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up USB permissions (Linux):
   ```bash
   sudo cp 91-odrive.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
   Then log out and back in.

## Usage

```bash
python main.py
```
