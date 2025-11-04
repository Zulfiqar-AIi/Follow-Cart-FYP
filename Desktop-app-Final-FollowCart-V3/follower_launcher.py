import tkinter as tk
from tkinter import ttk, messagebox, PhotoImage
from PIL import Image, ImageTk
import subprocess
import sys
import os

class FollowerSystemLauncher:
    def __init__(self, root):
        self.root = root
        self.root.title("FollowCart: Your Smart Moving Companion")
        self.root.geometry("600x700")  # Reduced height since no debug checkbox
        self.root.resizable(False, False)
        
        # Configure style for modern look
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TCombobox', padding=5, relief='flat')
        self.style.configure('TButton', font=('Helvetica', 10), padding=10)
        
        # Create main container with padding
        self.main_frame = tk.Frame(root, bg='#f0f2f5', padx=20, pady=20)
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Header with logo and title
        self.create_header()
        
        # COM Port Selection
        self.create_com_port_section()
        
        # System Type Selection
        self.create_system_type_section()
        
        # Launch Button
        self.create_launch_button()
        
        # Info Label
        self.create_info_label()
        
        # Team Information Section
        self.create_team_section()
        
        # Footer
        self.create_footer()
        
        # Set default selection
        self.system_type.current(0)
    
    def create_header(self):
        """Create the header section with logo and title"""
        header_frame = tk.Frame(self.main_frame, bg='#f0f2f5')
        header_frame.pack(fill=tk.X, pady=(0, 15))
        
        # Try to load and display the logo
        try:
            logo_img = Image.open('duet_logo.png')
            logo_img = logo_img.resize((80, 80), Image.LANCZOS)
            self.logo = ImageTk.PhotoImage(logo_img)
            logo_label = tk.Label(header_frame, image=self.logo, bg='#f0f2f5')
            logo_label.pack(side=tk.LEFT, padx=(0, 15))
        except Exception as e:
            print(f"Could not load logo: {e}")
            # Create a placeholder if logo fails to load
            logo_placeholder = tk.Label(header_frame, text="DUET", font=('Helvetica', 12, 'bold'), 
                                       bg='#f0f2f5', fg='#333')
            logo_placeholder.pack(side=tk.LEFT, padx=(0, 15))
        
        # Main title
        title_frame = tk.Frame(header_frame, bg='#f0f2f5')
        title_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        main_title = tk.Label(title_frame, text="FollowCart", font=('Helvetica', 24, 'bold'), 
                             bg='#f0f2f5', fg='#2c3e50')
        main_title.pack(anchor=tk.W)
        
        subtitle = tk.Label(title_frame, text="Your Smart Moving Companion", font=('Helvetica', 12), 
                           bg='#f0f2f5', fg='#7f8c8d')
        subtitle.pack(anchor=tk.W)
        
        # University name
        uni_label = tk.Label(title_frame, text="Dawood University of Engineering & Technology", 
                            font=('Helvetica', 10), bg='#f0f2f5', fg='#3498db')
        uni_label.pack(anchor=tk.W, pady=(5, 0))
        
        # FYP label
        fyp_label = tk.Label(title_frame, text="Final Year Project - Computer Science", 
                           font=('Helvetica', 9, 'italic'), bg='#f0f2f5', fg='#e67e22')
        fyp_label.pack(anchor=tk.W, pady=(2, 0))
    
    def create_com_port_section(self):
        """Create the COM port selection section"""
        com_frame = tk.Frame(self.main_frame, bg='#f0f2f5')
        com_frame.pack(fill=tk.X, pady=(10, 5))
        
        tk.Label(com_frame, text="COM Port:", font=('Helvetica', 10, 'bold'), 
                bg='#f0f2f5', fg='#2c3e50').pack(anchor=tk.W, pady=(0, 5))
        
        self.com_port = ttk.Combobox(
            com_frame, 
            values=[f"COM{i}" for i in range(1, 21)],
            font=('Helvetica', 10),
            height=15
        )
        self.com_port.set("COM3")
        self.com_port.pack(fill=tk.X, padx=5, pady=5, ipady=8)
        
        # Add some styling to the combobox
        self.com_port.bind("<FocusIn>", lambda e: self.com_port.config(foreground='black'))
        self.com_port.bind("<FocusOut>", lambda e: self.com_port.config(foreground='black'))
    
    def create_system_type_section(self):
        """Create the system type selection section"""
        system_frame = tk.Frame(self.main_frame, bg='#f0f2f5')
        system_frame.pack(fill=tk.X, pady=(15, 5))
        
        tk.Label(system_frame, text="Select Follower System:", font=('Helvetica', 10, 'bold'), 
                bg='#f0f2f5', fg='#2c3e50').pack(anchor=tk.W, pady=(0, 5))
        
        self.system_type = ttk.Combobox(
            system_frame, 
            state="readonly",
            values=[
                "Hybrid Cart with 1 Camera  (Basic Following Obstacle Avoidance)",
                "ODRR Cart with 1 Camera (YOLO Obstacle Avoidance)",
                "ODRR Cart with 2 Cameras (YOLO Obstacle Avoidance)",
                "Simple Cart with 1 Camera (Basic Following)",
                "Simple Cart with 2 Cameras (Basic Following)",
                "ODCR Demo with 1 Camera (Obstacle Detection)"
            ],
            font=('Helvetica', 10),
            height=15
        )
        self.system_type.pack(fill=tk.X, padx=5, pady=5, ipady=8)
        
        # Add description label for selected system
        self.description_label = tk.Label(
            system_frame,
            text="",
            font=('Helvetica', 9),
            bg='#f0f2f5',
            fg='#555555',
            wraplength=550,
            justify=tk.LEFT
        )
        self.description_label.pack(fill=tk.X, pady=(5, 0))
        
        # Bind selection change to update description
        self.system_type.bind('<<ComboboxSelected>>', self.update_description)
        
        # Set default and update description
        self.system_type.current(0)
        self.update_description()
    
    def update_description(self, event=None):
        """Update description based on selected system"""
        selection = self.system_type.get()
        
        descriptions = {
            "Hybrid Cart with 1 Camera  (Basic Following Obstacle Avoidance)": 
                "Hybrid cart system with basic following capabilities and obstacle avoidance using single camera input.",
            
            "ODRR Cart with 1 Camera (YOLO Obstacle Avoidance)": 
                "Omnidirectional cart with YOLO-based object detection for intelligent obstacle avoidance using single camera input.",
            
            "ODRR Cart with 2 Cameras (YOLO Obstacle Avoidance)": 
                "Advanced omnidirectional cart with dual camera setup for enhanced YOLO object detection and superior obstacle avoidance.",
            
            "Simple Cart with 1 Camera (Basic Following)": 
                "Basic cart following system with single camera for straightforward subject tracking without advanced obstacle detection.",
            
            "Simple Cart with 2 Cameras (Basic Following)": 
                "Enhanced basic cart system with dual cameras for improved tracking accuracy and wider field of view.",
            
            "ODCR Demo with 1 Camera (Obstacle Detection)": 
                "Demonstration version focusing on obstacle detection and collision avoidance using single camera input."
        }
        
        description = descriptions.get(selection, "Select a system to see description.")
        self.description_label.config(text=description)
    
    def create_launch_button(self):
        """Create the launch button with modern styling"""
        button_frame = tk.Frame(self.main_frame, bg='#f0f2f5')
        button_frame.pack(fill=tk.X, pady=(25, 15))
        
        self.launch_btn = tk.Button(
            button_frame, 
            text="🚀 LAUNCH SYSTEM", 
            command=self.launch_system,
            font=('Helvetica', 12, 'bold'),
            bg='#3498db',
            fg='white',
            activebackground='#2980b9',
            activeforeground='white',
            relief=tk.FLAT,
            bd=0,
            padx=20,
            pady=12
        )
        self.launch_btn.pack(fill=tk.X, ipady=5)
        
        # Add hover effects
        self.launch_btn.bind("<Enter>", lambda e: self.launch_btn.config(bg='#2980b9'))
        self.launch_btn.bind("<Leave>", lambda e: self.launch_btn.config(bg='#3498db'))
    
    def create_info_label(self):
        """Create the info display label"""
        self.info_label = tk.Label(
            self.main_frame, 
            text="💡 Select your preferred system configuration and click Launch to start!", 
            font=('Helvetica', 10),
            bg='#e8f4fc',
            fg='#2c3e50',
            wraplength=550,
            justify=tk.LEFT,
            relief=tk.RIDGE,
            bd=1,
            padx=10,
            pady=8
        )
        self.info_label.pack(fill=tk.X, pady=(10, 5))
    
    def create_team_section(self):
        """Create the team information section"""
        team_frame = tk.Frame(self.main_frame, bg='#f0f2f5', padx=10, pady=10)
        team_frame.pack(fill=tk.X, pady=(5, 10))
        
        # Team title
        tk.Label(
            team_frame, 
            text="Project Team:", 
            font=('Helvetica', 10, 'bold'),
            bg='#f0f2f5',
            fg='#2c3e50'
        ).pack(anchor=tk.W, pady=(0, 5))
        
        # Team members container
        members_frame = tk.Frame(team_frame, bg='#e8f4fc', relief=tk.GROOVE, bd=1)
        members_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Team members
        team_members = [
            "M. Asadullah Sohail (M-21/F-BSCS-04)",
            "Muhammad Daniyal (M-21/F-BSCS-26)",
            "Zulfiqar Ali (M-21/F-BSCS-40)",
            "Hafiz Sameer Khan (M-21/F-BSCS-99)"
        ]
        
        for member in team_members:
            tk.Label(
                members_frame, 
                text=member, 
                font=('Helvetica', 9),
                bg='#e8f4fc',
                fg='#2c3e50',
                anchor=tk.W
            ).pack(fill=tk.X, padx=10, pady=3)
    
    def create_footer(self):
        """Create the footer section"""
        footer_frame = tk.Frame(self.main_frame, bg='#f0f2f5')
        footer_frame.pack(fill=tk.X, pady=(5, 0))
        
        tk.Label(
            footer_frame, 
            text="© 2023 Dawood University of Engineering & Technology - Computer Science Department", 
            font=('Helvetica', 8),
            bg='#f0f2f5',
            fg='#95a5a6'
        ).pack(side=tk.BOTTOM, pady=(5, 0))
    
    def launch_system(self):
        com_port = self.com_port.get().strip()
        system_type = self.system_type.get()
        
        if not com_port:
            messagebox.showerror("Error", "Please select a COM port")
            return
            
        if not system_type:
            messagebox.showerror("Error", "Please select a system type")
            return
        
        # Updated script mapping
        script_map = {
            "Hybrid Cart with 1 Camera  (Basic Following Obstacle Avoidance)": "hybrid_cart_1_cam.py",
            "ODRR Cart with 1 Camera (YOLO Obstacle Avoidance)": "ODRR_cart_with_1_cam_new.py",
            "ODRR Cart with 2 Cameras (YOLO Obstacle Avoidance)": "ODRR_cart_with_2_cam_new.py",
            "Simple Cart with 1 Camera (Basic Following)": "simple_cart_1_cam_new.py",
            "Simple Cart with 2 Cameras (Basic Following)": "simple_cart_2_cam_new.py",
            "ODCR Demo with 1 Camera (Obstacle Detection)": "ODCR_demo_1_cam.PY"
        }
        
        script_name = script_map.get(system_type)
        if not script_name:
            messagebox.showerror("Error", "Invalid system selection")
            return
        
        # Check if script file exists
        if not os.path.exists(script_name):
            messagebox.showerror("Error", f"Script file '{script_name}' not found!\n\nPlease ensure the file is in the same directory as this launcher.")
            return
            
        try:
            # Change button to show loading state
            self.launch_btn.config(text="🚀 LAUNCHING...", state=tk.DISABLED)
            self.root.update()
            
            # Prepare command arguments - debug mode is always disabled
            cmd_args = [sys.executable, script_name, com_port]
            # Note: Debug mode is intentionally NOT added to ensure it's always unchecked/disabled
            
            # Launch the selected script with arguments
            subprocess.Popen(cmd_args)
            
            # Update info display
            status_text = f"Successfully launched:\n{system_type}\n\nCOM Port: {com_port}\nDebug Mode: DISABLED"
            
            self.info_label.config(
                text=status_text,
                fg='#27ae60',
                bg='#d5f4e6'
            )
            
            # Show success message
            messagebox.showinfo("System Launched", 
                f"🎯 {system_type} Launched Successfully!\n\n"
                f"COM Port: {com_port}\n"
                f"Debug Mode: DISABLED")
            
            # Reset button after short delay
            self.root.after(3000, lambda: self.launch_btn.config(text="🚀 LAUNCH SYSTEM", state=tk.NORMAL))
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch system:\n{str(e)}")
            self.launch_btn.config(text="🚀 LAUNCH SYSTEM", state=tk.NORMAL)
            self.info_label.config(
                text=f"Error: {str(e)}", 
                fg='#e74c3c',
                bg='#f8d7da'
            )

if __name__ == "__main__":
    root = tk.Tk()
    
    # Set window icon if available
    try:
        root.iconbitmap('duet_icon.ico')  # You should provide this icon file
    except:
        pass
        
    app = FollowerSystemLauncher(root)
    root.mainloop()