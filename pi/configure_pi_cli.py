#!/usr/bin/env python3
"""
Raspberry Pi Setup Configuration CLI Tool

This script creates rc.local files for Raspberry Pi first-boot setup
using command-line interface for those who prefer not to use GUI.
"""

import argparse
import json
import os
import sys
from pathlib import Path


def load_template(template_path):
    """Load the rc.local template file"""
    if not template_path.exists():
        print(f"Error: Template file not found: {template_path}")
        sys.exit(1)
        
    with open(template_path, 'r') as f:
        return f.read()


def validate_config(config):
    """Validate configuration parameters"""
    errors = []
    
    if not config['username'].strip():
        errors.append("Username cannot be empty")
    
    if not config['ap_ssid'].strip():
        errors.append("WiFi SSID cannot be empty")
        
    if len(config['ap_password']) < 8:
        errors.append("WiFi password must be at least 8 characters")
        
    repo = config['github_repo'].strip()
    if repo and not (repo.startswith('http://') or repo.startswith('https://') or repo.startswith('git@')):
        errors.append("Repository URL must start with http://, https://, or git@")
        
    return errors


def generate_rc_local(config, output_file):
    """Generate rc.local file from template and config"""
    template_path = Path(__file__).parent / "rc.local.template"
    template_content = load_template(template_path)
    
    # Replace placeholders
    content = template_content.replace("{{USERNAME}}", config['username'])
    content = content.replace("{{AP_SSID}}", config['ap_ssid'])
    content = content.replace("{{AP_PASSWORD}}", config['ap_password'])
    content = content.replace("{{GITHUB_REPO}}", config['github_repo'])
    content = content.replace("{{CLONE_PATH}}", config['clone_path'])
    
    # Write output file
    with open(output_file, 'w') as f:
        f.write(content)
    
    # Make executable
    os.chmod(output_file, 0o755)
    
    print(f"✓ Generated rc.local file: {output_file}")
    print("✓ File has been made executable")
    print(f"✓ Copy this file to /etc/rc.local on your Raspberry Pi")


def interactive_config():
    """Interactive configuration mode"""
    print("=== Raspberry Pi Setup Configuration ===\\n")
    
    config = {}
    
    # User configuration
    print("User Configuration:")
    config['username'] = input("Username [lab]: ").strip() or "lab"
    
    # WiFi configuration
    print("\\nWiFi Hotspot Configuration:")
    config['ap_ssid'] = input("Hotspot SSID [CSI-Pi-1]: ").strip() or "CSI-Pi-1"
    
    while True:
        config['ap_password'] = input("Hotspot Password [password]: ").strip() or "password"
        if len(config['ap_password']) >= 8:
            break
        print("Error: Password must be at least 8 characters")
    
    # Repository configuration
    print("\\nRepository Configuration (optional):")
    config['github_repo'] = input("GitHub Repository URL [https://github.com/kuederleR/CSIModule.git]: ").strip()
    if not config['github_repo']:
        config['github_repo'] = "https://github.com/kuederleR/CSIModule.git"
    
    config['clone_path'] = input("Clone Path [CSIModule]: ").strip() or "CSIModule"
    
    return config


def print_config_summary(config):
    """Print configuration summary"""
    print("\\n=== Configuration Summary ===")
    print(f"Username: {config['username']}")
    print(f"WiFi SSID: {config['ap_ssid']}")
    print(f"WiFi Password: {'*' * len(config['ap_password'])}")
    print(f"Repository: {config['github_repo'] or 'None'}")
    print(f"Clone Path: {config['clone_path'] or 'N/A'}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Generate rc.local file for Raspberry Pi first-boot setup",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode
  %(prog)s --interactive

  # Command line mode with parameters
  %(prog)s --username lab --ssid "My-Pi-Hotspot" --password "secure123" --output rc.local

  # Using a configuration file
  %(prog)s --config config.json --output rc.local

  # Save current settings to config file
  %(prog)s --username lab --ssid "My-Pi" --password "pass123" --save-config settings.json
        """
    )
    
    # Configuration options
    parser.add_argument('--username', '-u', default='lab',
                       help='System username (default: lab)')
    parser.add_argument('--ssid', '-s', default='CSI-Pi-1',
                       help='WiFi hotspot SSID (default: CSI-Pi-1)')
    parser.add_argument('--password', '-p', default='password',
                       help='WiFi hotspot password (default: password)')
    parser.add_argument('--repo', '-r', default='https://github.com/kuederleR/CSIModule.git',
                       help='GitHub repository URL (default: CSIModule repo)')
    parser.add_argument('--clone-path', '-c', default='CSIModule',
                       help='Repository clone path (default: CSIModule)')
    
    # Input/Output options
    parser.add_argument('--config', metavar='FILE',
                       help='Load configuration from JSON file')
    parser.add_argument('--save-config', metavar='FILE',
                       help='Save configuration to JSON file')
    parser.add_argument('--output', '-o', default='rc.local',
                       help='Output file path (default: rc.local)')
    
    # Mode options
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Interactive configuration mode')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='Quiet mode (minimal output)')
    
    args = parser.parse_args()
    
    # Interactive mode
    if args.interactive:
        config = interactive_config()
    # Load from config file
    elif args.config:
        try:
            with open(args.config, 'r') as f:
                config = json.load(f)
        except FileNotFoundError:
            print(f"Error: Configuration file not found: {args.config}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in configuration file: {e}")
            sys.exit(1)
    # Command line arguments
    else:
        config = {
            'username': args.username,
            'ap_ssid': args.ssid,
            'ap_password': args.password,
            'github_repo': args.repo,
            'clone_path': args.clone_path,
        }
    
    # Validate configuration
    errors = validate_config(config)
    if errors:
        print("Configuration errors:")
        for error in errors:
            print(f"  ✗ {error}")
        sys.exit(1)
    
    # Save configuration if requested
    if args.save_config:
        try:
            with open(args.save_config, 'w') as f:
                json.dump(config, f, indent=2)
            if not args.quiet:
                print(f"✓ Configuration saved to: {args.save_config}")
        except Exception as e:
            print(f"Error saving configuration: {e}")
            sys.exit(1)
    
    # Show configuration summary
    if not args.quiet:
        print_config_summary(config)
    
    # Generate rc.local file
    try:
        generate_rc_local(config, args.output)
        if not args.quiet:
            print("\\n=== Setup Instructions ===")
            print("1. Copy the generated rc.local file to your Raspberry Pi")
            print("2. Place it at /etc/rc.local (requires sudo)")
            print("3. Ensure it's executable: sudo chmod +x /etc/rc.local")
            print("4. Reboot the Pi to trigger first-boot setup")
    except Exception as e:
        print(f"Error generating rc.local file: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()