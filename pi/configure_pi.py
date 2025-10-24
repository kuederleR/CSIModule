#!/usr/bin/env python3
"""
Raspberry Pi Setup Configuration Widget

This script creates a GUI configuration tool for generating a customized rc.local
file for Raspberry Pi first-boot setup. It allows users to specify WiFi credentials,
user settings, and repository details.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import sys
from pathlib import Path


class PiConfigWidget:
    def __init__(self, root):
        self.root = root
        self.root.title("Raspberry Pi Setup Configuration")
        self.root.geometry("600x700")
        self.root.resizable(True, True)
        
        # Configuration variables
        self.config = {
            'username': tk.StringVar(value='lab'),
            'ap_ssid': tk.StringVar(value='CSI-Pi-1'),
            'ap_password': tk.StringVar(value='password'),
            'github_repo': tk.StringVar(value='https://github.com/kuederleR/CSIModule.git'),
            'clone_path': tk.StringVar(value='CSIModule'),
        }
        
        self.create_widgets()
        
    def create_widgets(self):
        # Main frame with padding
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="Raspberry Pi Setup Configuration", 
                               font=('TkDefaultFont', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # User Configuration Section
        user_frame = ttk.LabelFrame(main_frame, text="User Configuration", padding="10")
        user_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        user_frame.columnconfigure(1, weight=1)
        
        ttk.Label(user_frame, text="Username:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        username_entry = ttk.Entry(user_frame, textvariable=self.config['username'], width=30)
        username_entry.grid(row=0, column=1, sticky=(tk.W, tk.E))
        ttk.Label(user_frame, text="(default system user)", foreground='gray').grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        
        # WiFi Hotspot Configuration Section
        wifi_frame = ttk.LabelFrame(main_frame, text="WiFi Hotspot Configuration", padding="10")
        wifi_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        wifi_frame.columnconfigure(1, weight=1)
        
        ttk.Label(wifi_frame, text="Hotspot SSID:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        ssid_entry = ttk.Entry(wifi_frame, textvariable=self.config['ap_ssid'], width=30)
        ssid_entry.grid(row=0, column=1, sticky=(tk.W, tk.E))
        ttk.Label(wifi_frame, text="(network name)", foreground='gray').grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(wifi_frame, text="Hotspot Password:").grid(row=1, column=0, sticky=tk.W, padx=(0, 10), pady=(10, 0))
        password_entry = ttk.Entry(wifi_frame, textvariable=self.config['ap_password'], width=30, show="*")
        password_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=(10, 0))
        ttk.Label(wifi_frame, text="(min 8 chars)", foreground='gray').grid(row=1, column=2, sticky=tk.W, padx=(10, 0), pady=(10, 0))
        
        # Show/Hide password button
        self.show_password = tk.BooleanVar()
        show_pwd_check = ttk.Checkbutton(wifi_frame, text="Show password", 
                                        variable=self.show_password,
                                        command=lambda: self.toggle_password_visibility(password_entry))
        show_pwd_check.grid(row=2, column=1, sticky=tk.W, pady=(5, 0))
        
        # Repository Configuration Section
        repo_frame = ttk.LabelFrame(main_frame, text="Repository Configuration (Optional)", padding="10")
        repo_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        repo_frame.columnconfigure(1, weight=1)
        
        ttk.Label(repo_frame, text="GitHub Repository:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        repo_entry = ttk.Entry(repo_frame, textvariable=self.config['github_repo'], width=50)
        repo_entry.grid(row=0, column=1, sticky=(tk.W, tk.E))
        ttk.Button(repo_frame, text="Clear", command=lambda: self.config['github_repo'].set(""),
                  width=8).grid(row=0, column=2, padx=(5, 0))
        
        ttk.Label(repo_frame, text="Clone Path:").grid(row=1, column=0, sticky=tk.W, padx=(0, 10), pady=(10, 0))
        path_entry = ttk.Entry(repo_frame, textvariable=self.config['clone_path'], width=30)
        path_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=(10, 0))
        ttk.Label(repo_frame, text="(relative to user home)", foreground='gray').grid(row=1, column=2, sticky=tk.W, padx=(10, 0), pady=(10, 0))
        
        # Preview Section
        preview_frame = ttk.LabelFrame(main_frame, text="Configuration Preview", padding="10")
        preview_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        preview_frame.columnconfigure(0, weight=1)
        
        self.preview_text = tk.Text(preview_frame, height=6, width=70, wrap=tk.WORD, 
                                   font=('TkFixedFont', 9), background='#f0f0f0')
        self.preview_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        # Scrollbar for preview
        preview_scroll = ttk.Scrollbar(preview_frame, orient="vertical", command=self.preview_text.yview)
        preview_scroll.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.preview_text.configure(yscrollcommand=preview_scroll.set)
        
        # Update preview initially and on changes
        self.update_preview()
        for var in self.config.values():
            var.trace('w', lambda *args: self.update_preview())
        
        # Buttons Section
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=5, column=0, columnspan=3, pady=(10, 0))
        
        ttk.Button(button_frame, text="Generate rc.local", 
                  command=self.generate_rc_local, style='Accent.TButton').pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Save Configuration", 
                  command=self.save_config).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Load Configuration", 
                  command=self.load_config).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Reset to Defaults", 
                  command=self.reset_defaults).pack(side=tk.LEFT)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 0))
        
    def toggle_password_visibility(self, entry_widget):
        """Toggle password visibility"""
        if self.show_password.get():
            entry_widget.configure(show="")
        else:
            entry_widget.configure(show="*")
    
    def update_preview(self):
        """Update the configuration preview"""
        preview_text = f"""Configuration Summary:
• Username: {self.config['username'].get()}
• WiFi Hotspot: {self.config['ap_ssid'].get()} (password: {'*' * len(self.config['ap_password'].get())})
• Repository: {self.config['github_repo'].get() or 'None'}
• Clone Path: {self.config['clone_path'].get() or 'N/A'}

The generated rc.local will:
1. Install Docker on first boot
2. Create WiFi hotspot with specified credentials
3. Clone repository (if specified) to /home/{self.config['username'].get()}/{self.config['clone_path'].get()}
4. Set appropriate file permissions"""
        
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(1.0, preview_text)
        
    def validate_config(self):
        """Validate the current configuration"""
        errors = []
        
        if not self.config['username'].get().strip():
            errors.append("Username cannot be empty")
        
        if not self.config['ap_ssid'].get().strip():
            errors.append("WiFi SSID cannot be empty")
            
        if len(self.config['ap_password'].get()) < 8:
            errors.append("WiFi password must be at least 8 characters")
            
        repo = self.config['github_repo'].get().strip()
        if repo and not (repo.startswith('http://') or repo.startswith('https://') or repo.startswith('git@')):
            errors.append("Repository URL must start with http://, https://, or git@")
            
        return errors
        
    def generate_rc_local(self):
        """Generate the rc.local file"""
        errors = self.validate_config()
        if errors:
            messagebox.showerror("Validation Error", "\\n".join(errors))
            return
            
        # Get template path
        template_path = Path(__file__).parent / "rc.local.template"
        if not template_path.exists():
            messagebox.showerror("Error", f"Template file not found: {template_path}")
            return
            
        try:
            # Read template
            with open(template_path, 'r') as f:
                template_content = f.read()
            
            # Replace placeholders
            content = template_content.replace("{{USERNAME}}", self.config['username'].get())
            content = content.replace("{{AP_SSID}}", self.config['ap_ssid'].get())
            content = content.replace("{{AP_PASSWORD}}", self.config['ap_password'].get())
            content = content.replace("{{GITHUB_REPO}}", self.config['github_repo'].get())
            content = content.replace("{{CLONE_PATH}}", self.config['clone_path'].get())
            
            # Ask user where to save
            output_file = filedialog.asksaveasfilename(
                title="Save rc.local file",
                defaultextension="",
                initialfile="rc.local",
                filetypes=[("Local files", "rc.local"), ("All files", "*.*")]
            )
            
            if output_file:
                with open(output_file, 'w') as f:
                    f.write(content)
                    
                # Make executable
                os.chmod(output_file, 0o755)
                
                self.status_var.set(f"Generated: {output_file}")
                messagebox.showinfo("Success", f"rc.local file generated successfully!\\n\\nSaved to: {output_file}\\n\\nDon't forget to copy this file to /etc/rc.local on your Raspberry Pi.")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to generate rc.local file:\\n{str(e)}")
            
    def save_config(self):
        """Save current configuration to a JSON file"""
        try:
            import json
            
            config_data = {key: var.get() for key, var in self.config.items()}
            
            output_file = filedialog.asksaveasfilename(
                title="Save Configuration",
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if output_file:
                with open(output_file, 'w') as f:
                    json.dump(config_data, f, indent=2)
                self.status_var.set(f"Configuration saved: {output_file}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save configuration:\\n{str(e)}")
            
    def load_config(self):
        """Load configuration from a JSON file"""
        try:
            import json
            
            input_file = filedialog.askopenfilename(
                title="Load Configuration",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if input_file:
                with open(input_file, 'r') as f:
                    config_data = json.load(f)
                
                for key, value in config_data.items():
                    if key in self.config:
                        self.config[key].set(value)
                        
                self.status_var.set(f"Configuration loaded: {input_file}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load configuration:\\n{str(e)}")
            
    def reset_defaults(self):
        """Reset configuration to defaults"""
        defaults = {
            'username': 'lab',
            'ap_ssid': 'CSI-Pi-1',
            'ap_password': 'password',
            'github_repo': 'https://github.com/kuederleR/CSIModule.git',
            'clone_path': 'CSIModule',
        }
        
        for key, value in defaults.items():
            self.config[key].set(value)
            
        self.status_var.set("Reset to default values")


def main():
    """Main function to run the configuration widget"""
    root = tk.Tk()
    
    # Set the application icon and styling
    try:
        root.tk.call('source', 'azure.tcl')
        root.tk.call('set_theme', 'light')
    except:
        pass  # Fall back to default theme if Azure theme not available
    
    app = PiConfigWidget(root)
    
    # Center the window
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f"+{x}+{y}")
    
    root.mainloop()


if __name__ == "__main__":
    main()