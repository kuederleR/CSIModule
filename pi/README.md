# Raspberry Pi Configuration Tools

This directory contains tools for configuring Raspberry Pi first-boot setup with customizable settings instead of hardcoded values.

## Files

- **`rc.local.template`** - Template file with placeholders for configuration values
- **`configure_pi.py`** - GUI configuration widget (requires tkinter)
- **`configure_pi_cli.py`** - Command-line configuration tool
- **`rc.local`** - Original hardcoded file (deprecated)

## Quick Start

### GUI Method (Recommended)

```bash
# Run the graphical configuration tool
python3 configure_pi.py
```

The GUI allows you to:
- Set username, WiFi credentials, and repository details
- Preview configuration before generating
- Save/load configuration presets
- Generate properly formatted rc.local files

### Command Line Method

```bash
# Interactive mode
python3 configure_pi_cli.py --interactive

# Direct command line
python3 configure_pi_cli.py --username myuser --ssid "My-Hotspot" --password "securepass123" --output rc.local

# Using a config file
python3 configure_pi_cli.py --config my_settings.json --output rc.local
```

## Configuration Options

| Parameter | Description | Default |
|-----------|-------------|---------|
| `username` | System user account | `lab` |
| `ap_ssid` | WiFi hotspot name | `CSI-Pi-1` |
| `ap_password` | WiFi hotspot password | `password` |
| `github_repo` | Repository to clone | `https://github.com/kuederleR/CSIModule.git` |
| `clone_path` | Where to clone repo | `CSIModule` |

## What the Generated rc.local Does

1. **Waits for internet connection** - Ensures network is available before proceeding
2. **Installs Docker** - Downloads and installs Docker for containerized applications
3. **Clones repository** - Downloads specified GitHub repository (optional)
4. **Configures WiFi hotspot** - Sets up access point with custom credentials
5. **Sets permissions** - Ensures proper file ownership and permissions
6. **Creates lock file** - Prevents script from running multiple times

## Usage Examples

### Example 1: Basic Setup
```bash
python3 configure_pi_cli.py \
  --username pi \
  --ssid "CSI-Lab-Pi" \
  --password "research123" \
  --output /tmp/rc.local
```

### Example 2: Research Lab Setup
```bash
python3 configure_pi_cli.py \
  --username researcher \
  --ssid "Lab-Station-01" \
  --password "science2024!" \
  --repo "https://github.com/mylab/csi-tools.git" \
  --clone-path "csi-tools" \
  --output rc.local
```

### Example 3: Configuration File
```bash
# Save a configuration
python3 configure_pi_cli.py \
  --username lab \
  --ssid "Mobile-CSI" \
  --password "fieldwork123" \
  --save-config mobile_config.json

# Use the saved configuration
python3 configure_pi_cli.py --config mobile_config.json --output rc.local
```

## Installation on Raspberry Pi

After generating your `rc.local` file:

1. **Copy to Pi**: 
   ```bash
   scp rc.local pi@your-pi-ip:/tmp/
   ```

2. **Install on Pi**:
   ```bash
   ssh pi@your-pi-ip
   sudo cp /tmp/rc.local /etc/rc.local
   sudo chmod +x /etc/rc.local
   ```

3. **Reboot**:
   ```bash
   sudo reboot
   ```

## Monitoring Setup Progress

After reboot, you can monitor the setup progress:

```bash
# SSH into the Pi and check the log
tail -f /home/yourusername/first-boot-setup.log

# Check if setup completed
ls -la /var/log/first-boot-setup-complete.lock
```

## Configuration File Format

Configuration files are saved as JSON:

```json
{
  "username": "lab",
  "ap_ssid": "CSI-Pi-1",
  "ap_password": "password",
  "github_repo": "https://github.com/kuederleR/CSIModule.git",
  "clone_path": "CSIModule"
}
```

## Troubleshooting

### Common Issues

1. **GUI won't start**: Install tkinter with `sudo apt install python3-tk`
2. **Permission denied**: Ensure scripts are executable with `chmod +x`
3. **Invalid password**: WiFi passwords must be at least 8 characters
4. **Repository clone fails**: Check repository URL and internet connectivity

### Log Files

- **Setup log**: `/home/username/first-boot-setup.log`
- **Lock file**: `/var/log/first-boot-setup-complete.lock`

### Reset Setup

To re-run the setup (for testing):

```bash
sudo rm /var/log/first-boot-setup-complete.lock
sudo reboot
```

## Advanced Usage

### Multiple Pi Configuration

Create different configurations for multiple Pis:

```bash
# Pi 1 - Receiver
python3 configure_pi_cli.py --ssid "CSI-RX-01" --save-config rx_config.json

# Pi 2 - Transmitter  
python3 configure_pi_cli.py --ssid "CSI-TX-01" --save-config tx_config.json

# Generate both
python3 configure_pi_cli.py --config rx_config.json --output rc_receiver.local
python3 configure_pi_cli.py --config tx_config.json --output rc_transmitter.local
```

### Custom Repository Setup

For private repositories or custom setups:

```bash
python3 configure_pi_cli.py \
  --repo "git@github.com:private/repo.git" \
  --clone-path "custom-project" \
  --username dev \
  --output rc.local
```

Note: For private repositories, ensure SSH keys are properly configured on the Pi.