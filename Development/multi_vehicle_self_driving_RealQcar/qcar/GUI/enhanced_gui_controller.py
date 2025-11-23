"""
Enhanced GUI Controller for Multiple Physical QCars
Updated to work with the new CommandHandler system

Features:
- Full compatibility with new command format
- Platoon control support
- Command validation
- Better error handling and feedback
- Statistics tracking
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
import json
from datetime import datetime
from enhanced_remote_controller import QCarRemoteController


class EnhancedQCarGUIController:
    """Enhanced Graphical User Interface for controlling multiple QCars"""
    
    def __init__(self, root, num_cars=2, host_ip='127.0.0.1', base_port=5000):
        self.root = root
        self.num_cars = num_cars
        self.max_cars = 10
        self.controller = QCarRemoteController(host_ip, base_port)
        self.car_panels = {}
        self.car_expanded = {}
        self.connected_cars = set()  # Track connected cars
        self.v2v_status = {}  # Track V2V status for each car
        
        # Statistics tracking
        self.commands_sent_gui = 0
        self.commands_failed_gui = 0
        self.start_time = time.time()
        
        # Setup window
        self.root.title("🚗 Enhanced QCar Fleet Controller")
        self.root.geometry("1400x900")  # Wider window for better horizontal layout
        self.root.configure(bg='#1e1e1e')
        
        # Style configuration
        self.setup_styles()
        
        # Start the remote controller
        self.controller.start_server(self.num_cars)
        
        # Store reference to this GUI in controller for V2V status forwarding
        self.controller.gui_controller = self
        
        # Build GUI
        self.build_gui()
        
        # Don't initialize car panels initially - will be created when cars connect
        # self.update_car_panels()
        
        # Start update loop
        self.running = True
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_styles(self):
        """Configure enhanced ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Enhanced colors
        bg_dark = '#1e1e1e'
        bg_medium = '#2d2d2d'
        bg_light = '#3d3d3d'
        bg_panel = '#252525'
        fg_color = '#ffffff'
        fg_dim = '#cccccc'
        accent_green = '#4caf50'
        accent_red = '#f44336'
        accent_blue = '#2196f3'
        accent_orange = '#ff9800'
        accent_purple = '#9c27b0'
        
        # Configure enhanced styles
        style.configure('Title.TLabel', 
                       background=bg_dark, 
                       foreground=fg_color, 
                       font=('Segoe UI', 24, 'bold'))
        
        style.configure('Subtitle.TLabel',
                       background=bg_medium,
                       foreground=fg_color,
                       font=('Segoe UI', 14, 'bold'))
        
        style.configure('Info.TLabel',
                       background=bg_medium,
                       foreground=fg_dim,
                       font=('Segoe UI', 11))
        
        style.configure('Status.TLabel',
                       background=bg_light,
                       foreground=fg_color,
                       font=('Segoe UI', 10))
        
        style.configure('CarFrame.TFrame',
                       background=bg_medium,
                       relief='raised',
                       borderwidth=2)
        
        # Button styles
        style.configure('Start.TButton',
                       background=accent_green,
                       foreground='white',
                       font=('Segoe UI', 11, 'bold'))
        
        style.configure('Stop.TButton',
                       background=accent_red,
                       foreground='white',
                       font=('Segoe UI', 11, 'bold'))
        
        style.configure('Command.TButton',
                       background=accent_blue,
                       foreground='white',
                       font=('Segoe UI', 10))
        
        style.configure('Platoon.TButton',
                       background=accent_purple,
                       foreground='white',
                       font=('Segoe UI', 10))
        
        style.configure('Emergency.TButton',
                       background=accent_orange,
                       foreground='white',
                       font=('Segoe UI', 11, 'bold'))
    
    def build_gui(self):
        """Build the enhanced GUI layout"""
        
        # Enhanced header with statistics
        header_frame = tk.Frame(self.root, bg='#0d0d0d', height=80)
        header_frame.pack(fill='x', padx=0, pady=0)
        header_frame.pack_propagate(False)
        
        title_frame = tk.Frame(header_frame, bg='#0d0d0d')
        title_frame.pack(fill='both', expand=True)
        
        title = ttk.Label(title_frame, 
                         text="🚗 Enhanced QCar Fleet Controller", 
                         style='Title.TLabel')
        title.pack(pady=10)
        
        # Statistics bar
        self.stats_label = tk.Label(title_frame,
                                   text="Commands: 0 sent, 0 failed | Uptime: 0s",
                                   bg='#0d0d0d',
                                   fg='#888888',
                                   font=('Segoe UI', 10))
        self.stats_label.pack(pady=(0, 5))
        
        # Main content area with better layout
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill='both', expand=True, padx=15, pady=10)
        
        # Left panel - Car controls with enhanced scrolling
        left_panel = tk.Frame(main_frame, bg='#1e1e1e')
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        # Enhanced car count control
        self.create_car_count_panel(left_panel)
        
        # Scrollable car panels area
        canvas_frame = tk.Frame(left_panel, bg='#1e1e1e')
        canvas_frame.pack(fill='both', expand=True, pady=(10, 0))
        
        self.car_canvas = tk.Canvas(canvas_frame, bg='#1e1e1e', highlightthickness=0)
        scrollbar = tk.Scrollbar(canvas_frame, orient='vertical', command=self.car_canvas.yview)
        self.scrollable_frame = tk.Frame(self.car_canvas, bg='#1e1e1e')
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.car_canvas.configure(scrollregion=self.car_canvas.bbox("all"))
        )
        
        self.car_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.car_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.car_canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Enhanced mouse wheel scrolling
        self.car_canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        
        # Enhanced fleet controls
        fleet_frame = self.create_enhanced_fleet_controls(left_panel)
        fleet_frame.pack(fill='x', pady=(10, 5))
        
        # Right panel - Enhanced log and status
        right_panel = tk.Frame(main_frame, bg='#1e1e1e', width=450)
        right_panel.pack(side='right', fill='both', padx=(10, 0))
        right_panel.pack_propagate(False)
        
        # Enhanced connection status
        status_frame = self.create_enhanced_status_panel(right_panel)
        status_frame.pack(fill='x', pady=(0, 10))
        
        # Enhanced log area
        log_frame = self.create_enhanced_log_panel(right_panel)
        log_frame.pack(fill='both', expand=True)
    
    def _on_mousewheel(self, event):
        """Enhanced mouse wheel scrolling"""
        self.car_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    def create_car_count_panel(self, parent):
        """Enhanced car count control panel"""
        frame = tk.Frame(parent, bg='#2d2d2d', relief='raised', bd=2)
        frame.pack(fill='x', pady=(0, 5))
        
        content = tk.Frame(frame, bg='#2d2d2d')
        content.pack(fill='x', padx=15, pady=10)
        
        tk.Label(content,
                text="Fleet Size:",
                bg='#2d2d2d',
                fg='white',
                font=('Segoe UI', 12, 'bold')).pack(side='left', padx=(0, 10))
        
        # Enhanced car count spinbox
        self.car_count_var = tk.StringVar(value=str(self.num_cars))
        spinbox = tk.Spinbox(content,
                            from_=1,
                            to=self.max_cars,
                            textvariable=self.car_count_var,
                            width=5,
                            bg='#3d3d3d',
                            fg='white',
                            font=('Segoe UI', 11),
                            buttonbackground='#4d4d4d',
                            relief='flat',
                            insertbackground='white')
        spinbox.pack(side='left', padx=(0, 15))
        
        # Enhanced apply button
        apply_btn = tk.Button(content,
                             text="Apply",
                             bg='#2196f3',
                             fg='white',
                             font=('Segoe UI', 10, 'bold'),
                             command=self.apply_car_count,
                             cursor='hand2',
                             relief='flat',
                             padx=20,
                             pady=5)
        apply_btn.pack(side='left', padx=(0, 15))
        
        # Enhanced info label
        self.car_count_info = tk.Label(content,
                                       text=f"Active: {self.num_cars} cars",
                                       bg='#2d2d2d',
                                       fg='#4caf50',
                                       font=('Segoe UI', 10, 'bold'))
        self.car_count_info.pack(side='left')
    
    def apply_car_count(self):
        """Apply the new car count with enhanced validation"""
        try:
            new_count = int(self.car_count_var.get())
            if 1 <= new_count <= self.max_cars:
                old_count = self.num_cars
                self.num_cars = new_count
                
                # Update controller
                self.controller.start_server(self.num_cars)
                
                # Update GUI
                self.update_car_panels()
                self.car_count_info.config(text=f"Active: {self.num_cars} cars")
                
                self.log(f"Fleet size changed from {old_count} to {new_count} cars", 'INFO')
            else:
                messagebox.showerror("Invalid Input", f"Number of cars must be between 1 and {self.max_cars}")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number")
    
    def update_car_panels(self):
        """Update car panels - only show connected vehicles"""
        # Get current connection status
        current_connected = set()
        for car_id in range(self.num_cars):
            status = self.controller.get_car_status(car_id)
            if status['status'] == 'connected':
                current_connected.add(car_id)
        
        # Remove panels for disconnected cars
        for car_id in list(self.car_panels.keys()):
            if car_id not in current_connected:
                self.car_panels[car_id].destroy()
                del self.car_panels[car_id]
                if car_id in self.car_expanded:
                    del self.car_expanded[car_id]
        
        # Add panels for newly connected cars
        for car_id in current_connected:
            if car_id not in self.car_panels:
                panel = self.create_enhanced_car_panel(self.scrollable_frame, car_id)
                panel.pack(fill='x', pady=(0, 15))  # Increased spacing
                self.car_panels[car_id] = panel
        
        # Update connected cars tracking
        self.connected_cars = current_connected
        
        # If no cars connected, show a message
        if not current_connected:
            if not hasattr(self, 'no_cars_label'):
                self.no_cars_label = tk.Label(self.scrollable_frame,
                                            text="⏳ Waiting for vehicles to connect...\n\nNo vehicles currently connected",
                                            bg='#1e1e1e',
                                            fg='#888888',
                                            font=('Segoe UI', 14),
                                            justify='center',
                                            pady=50)
                self.no_cars_label.pack(fill='both', expand=True)
        else:
            # Remove the waiting message if it exists
            if hasattr(self, 'no_cars_label'):
                self.no_cars_label.destroy()
                delattr(self, 'no_cars_label')
    
    def create_enhanced_car_panel(self, parent, car_id):
        """Create enhanced control panel for a single car with improved horizontal layout"""
        # Main enhanced frame - larger and wider
        frame = tk.Frame(parent, bg='#2d2d2d', relief='raised', bd=2)
        
        # Enhanced header with status indicators
        header = tk.Frame(frame, bg='#1a1a1a', height=70)  # Taller header
        header.pack(fill='x')
        header.pack_propagate(False)
        
        # Enhanced expand/collapse button
        is_expanded = self.car_expanded.get(car_id, True)
        expand_btn = tk.Label(header,
                             text="▼" if is_expanded else "▶",
                             bg='#1a1a1a',
                             fg='#4caf50',
                             font=('Segoe UI', 20, 'bold'),  # Larger icon
                             cursor='hand2',
                             padx=20,
                             pady=20)
        expand_btn.pack(side='left', padx=(15, 10))
        
        # Enhanced car title with status
        title_frame = tk.Frame(header, bg='#1a1a1a')
        title_frame.pack(side='left', fill='y', expand=True, padx=15)
        
        title = tk.Label(title_frame, 
                        text=f"🚗 Car {car_id}",
                        bg='#1a1a1a',
                        fg='white',
                        font=('Segoe UI', 18, 'bold'),  # Larger title
                        cursor='hand2')
        title.pack(anchor='w', pady=(12, 0))
        
        # Car state indicator
        if not hasattr(self, 'car_state_labels'):
            self.car_state_labels = {}
        
        state_label = tk.Label(title_frame,
                              text="State: Unknown",
                              bg='#1a1a1a',
                              fg='#888888',
                              font=('Segoe UI', 12),  # Larger state text
                              cursor='hand2')
        state_label.pack(anchor='w')
        self.car_state_labels[car_id] = state_label
        
        # Enhanced connection indicator
        conn_indicator = tk.Label(header,
                                 text="🟢 Connected",
                                 bg='#1a1a1a',
                                 fg='#4caf50',
                                 font=('Segoe UI', 14, 'bold'),  # Larger connection text
                                 cursor='hand2',
                                 padx=20)
        conn_indicator.pack(side='right', padx=20, pady=20)
        
        # V2V Status Indicator (New)
        v2v_indicator = tk.Label(header,
                                text="📡 V2V: Off",
                                bg='#1a1a1a',
                                fg='#888888', # Gray initially
                                font=('Segoe UI', 12, 'bold'),
                                padx=10)
        v2v_indicator.pack(side='right', padx=(0, 10), pady=20)
        
        if not hasattr(self, 'v2v_indicators'):
            self.v2v_indicators = {}
        self.v2v_indicators[car_id] = v2v_indicator
        
        # Store reference
        if not hasattr(self, 'conn_indicators'):
            self.conn_indicators = {}
        self.conn_indicators[car_id] = conn_indicator
        
        # Enhanced content area with better horizontal layout
        content = tk.Frame(frame, bg='#2d2d2d')
        
        # Enhanced toggle functionality
        def toggle_panel(event=None):
            self.toggle_car_panel(car_id, content, expand_btn)
        
        # Bind click events to all header elements
        for widget in [expand_btn, header, title, state_label, conn_indicator]:
            widget.bind('<Button-1>', toggle_panel)
            
            # Enhanced hover effects
            def on_enter(e, w=widget):
                if w != expand_btn:
                    w.config(bg='#333333')
            
            def on_leave(e, w=widget):
                if w != expand_btn:
                    w.config(bg='#1a1a1a')
            
            widget.bind('<Enter>', on_enter)
            widget.bind('<Leave>', on_leave)
        
        # Show/hide content
        if is_expanded:
            content.pack(fill='x', padx=10, pady=10)  # Reduced padding
        
        # Store references
        frame.content_frame = content
        frame.expand_btn = expand_btn
        
        # COMPACT HORIZONTAL LAYOUT - Split into LEFT and RIGHT sections
        main_layout = tk.Frame(content, bg='#2d2d2d')
        main_layout.pack(fill='both', expand=True, pady=(0, 10))
        
        # LEFT SECTION: Telemetry + Start/Stop buttons (flexible width)
        left_section = tk.Frame(main_layout, bg='#2d2d2d')
        left_section.pack(side='left', fill='both', expand=True, padx=(0, 8))
        
        # Compact telemetry display (moved to left)
        telemetry_frame = tk.LabelFrame(left_section,
                                       text="📊 Telemetry",
                                       bg='#2d2d2d',
                                       fg='white',
                                       font=('Segoe UI', 12, 'bold'))
        telemetry_frame.pack(fill='x', pady=(0, 8))
        
        # Compact telemetry grid
        telemetry_grid = tk.Frame(telemetry_frame, bg='#2d2d2d')
        telemetry_grid.pack(fill='x', padx=10, pady=8)
        
        # Create compact telemetry labels
        labels = {}
        telemetry_data = [
            ('position', 'Position (m):', '(0.00, 0.00)'),
            ('velocity', 'Velocity (m/s):', '0.00'),
            ('heading', 'Heading (rad):', '0.00'),
            ('throttle', 'Throttle:', '0.00'),
            ('steering', 'Steering:', '0.00'),
            ('state', 'Vehicle State:', 'Unknown')
        ]
        
        for i, (key, label_text, default_value) in enumerate(telemetry_data):
            row = i // 2
            col = i % 2
            
            label_frame = tk.Frame(telemetry_grid, bg='#2d2d2d')
            label_frame.grid(row=row, column=col, padx=8, pady=3, sticky='w')
            
            tk.Label(label_frame,
                    text=label_text,
                    bg='#2d2d2d',
                    fg='#cccccc',
                    font=('Segoe UI', 10)).pack(side='left')  # Smaller font
            
            value_label = tk.Label(label_frame,
                                  text=default_value,
                                  bg='#2d2d2d',
                                  fg='white',
                                  font=('Segoe UI', 10, 'bold'))  # Smaller font
            value_label.pack(side='left', padx=(5, 0))
            
            labels[key] = value_label
        
        if not hasattr(self, 'telemetry_labels'):
            self.telemetry_labels = {}
        self.telemetry_labels[car_id] = labels
        
        # Compact control buttons (moved to left section below telemetry)
        button_frame = tk.Frame(left_section, bg='#2d2d2d')
        button_frame.pack(fill='x', pady=(5, 0))
        
        # Only Start and Stop buttons - more compact
        main_buttons = tk.Frame(button_frame, bg='#2d2d2d')
        main_buttons.pack(fill='x')
        
        start_btn = tk.Button(main_buttons,
                             text="▶ START",
                             bg='#4caf50',
                             fg='white',
                             font=('Segoe UI', 11, 'bold'),  # Smaller font
                             command=lambda: self.start_car_with_feedback(car_id),
                             cursor='hand2',
                             relief='flat',
                             padx=20,
                             pady=6)  # Smaller padding
        start_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        stop_btn = tk.Button(main_buttons,
                            text="⬛ STOP",
                            bg='#f44336',
                            fg='white',
                            font=('Segoe UI', 11, 'bold'),  # Smaller font
                            command=lambda: self.stop_car_with_feedback(car_id),
                            cursor='hand2',
                            relief='flat',
                            padx=20,
                            pady=6)  # Smaller padding
        stop_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        # RIGHT SECTION: Velocity + Path + Platoon controls (flexible width)
        right_section = tk.Frame(main_layout, bg='#2d2d2d')
        right_section.pack(side='right', fill='both', expand=True, padx=(8, 0))
        
        # Container for the three control panels
        controls_container = tk.Frame(right_section, bg='#2d2d2d')
        controls_container.pack(fill='both', expand=True)
        
        # Compact velocity control (top of right section)
        vel_frame = tk.LabelFrame(controls_container,
                                 text="🎯 Velocity",
                                 bg='#2d2d2d',
                                 fg='white',
                                 font=('Segoe UI', 11, 'bold'))
        vel_frame.pack(fill='x', pady=(0, 5))
        
        vel_content = tk.Frame(vel_frame, bg='#2d2d2d')
        vel_content.pack(fill='x', padx=8, pady=6)
        
        tk.Label(vel_content,
                text="Target:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        vel_entry = tk.Entry(vel_content,
                            width=8,
                            bg='#3d3d3d',
                            fg='white',
                            font=('Segoe UI', 10),  # Smaller font
                            insertbackground='white',
                            relief='flat')
        vel_entry.insert(0, "1.0")
        vel_entry.pack(side='left', padx=(0, 8))
        
        vel_btn = tk.Button(vel_content,
                           text="Set",
                           bg='#2196f3',
                           fg='white',
                           font=('Segoe UI', 9, 'bold'),  # Smaller font
                           command=lambda: self.set_velocity_with_feedback(car_id, vel_entry.get()),
                           cursor='hand2',
                           relief='flat',
                           padx=12,
                           pady=4)
        vel_btn.pack(side='left')
        
        # Compact path control (middle of right section)
        path_frame = tk.LabelFrame(controls_container,
                                  text="🛤️ Path",
                                  bg='#2d2d2d',
                                  fg='white',
                                  font=('Segoe UI', 11, 'bold'))
        path_frame.pack(fill='x', pady=(0, 5))
        
        path_content = tk.Frame(path_frame, bg='#2d2d2d')
        path_content.pack(fill='x', padx=8, pady=6)
        
        tk.Label(path_content,
                text="Nodes:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        path_entry = tk.Entry(path_content,
                             width=15,
                             bg='#3d3d3d',
                             fg='white',
                             font=('Segoe UI', 10),  # Smaller font
                             insertbackground='white',
                             relief='flat')
        path_entry.insert(0, "10 4 20 10" if car_id == 0 else "4 13 9 4")
        path_entry.pack(side='left', padx=(0, 8))
        
        path_btn = tk.Button(path_content,
                            text="Set",
                            bg='#2196f3',
                            fg='white',
                            font=('Segoe UI', 9, 'bold'),  # Smaller font
                            command=lambda: self.set_path_with_feedback(car_id, path_entry.get()),
                            cursor='hand2',
                            relief='flat',
                            padx=12,
                            pady=4)
        path_btn.pack(side='left')
        
        # Compact platoon control (bottom of right section)
        platoon_frame = tk.LabelFrame(controls_container,
                                     text="🚗 Platoon",
                                     bg='#2d2d2d',
                                     fg='white',
                                     font=('Segoe UI', 11, 'bold'))
        platoon_frame.pack(fill='x')
        
        platoon_content = tk.Frame(platoon_frame, bg='#2d2d2d')
        platoon_content.pack(fill='x', padx=8, pady=6)
        
        # Role selection - more compact
        role_frame = tk.Frame(platoon_content, bg='#2d2d2d')
        role_frame.pack(fill='x', pady=(0, 5))
        
        tk.Label(role_frame,
                text="Role:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        role_var = tk.StringVar(value="follower")
        leader_radio = tk.Radiobutton(role_frame,
                                     text="Leader",
                                     variable=role_var,
                                     value="leader",
                                     bg='#2d2d2d',
                                     fg='white',
                                     selectcolor='#3d3d3d',
                                     activebackground='#2d2d2d',
                                     activeforeground='white',
                                     font=('Segoe UI', 9))  # Smaller font
        leader_radio.pack(side='left', padx=(0, 15))
        
        follower_radio = tk.Radiobutton(role_frame,
                                       text="Follower",
                                       variable=role_var,
                                       value="follower",
                                       bg='#2d2d2d',
                                       fg='white',
                                       selectcolor='#3d3d3d',
                                       activebackground='#2d2d2d',
                                       activeforeground='white',
                                       font=('Segoe UI', 9))  # Smaller font
        follower_radio.pack(side='left')
        
        # Leader ID for follower mode - more compact
        leader_id_frame = tk.Frame(platoon_content, bg='#2d2d2d')
        leader_id_frame.pack(fill='x', pady=(0, 5))
        
        tk.Label(leader_id_frame,
                text="Follow ID:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        leader_id_entry = tk.Entry(leader_id_frame,
                                  width=4,
                                  bg='#3d3d3d',
                                  fg='white',
                                  font=('Segoe UI', 10),  # Smaller font
                                  insertbackground='white',
                                  relief='flat')
        leader_id_entry.insert(0, "0" if car_id != 0 else "1")
        leader_id_entry.pack(side='left')
        
        # Compact platoon buttons
        platoon_btn_frame = tk.Frame(platoon_content, bg='#2d2d2d')
        platoon_btn_frame.pack(fill='x', pady=(5, 0))
        
        enable_platoon_btn = tk.Button(platoon_btn_frame,
                                      text="Enable",
                                      bg='#9c27b0',
                                      fg='white',
                                      font=('Segoe UI', 9, 'bold'),  # Smaller font
                                      command=lambda: self.enable_platoon_with_feedback(car_id, role_var.get(), leader_id_entry.get()),
                                      cursor='hand2',
                                      relief='flat',
                                      padx=12,
                                      pady=4)
        enable_platoon_btn.pack(side='left', expand=True, fill='x', padx=(0, 4))
        
        disable_platoon_btn = tk.Button(platoon_btn_frame,
                                       text="Disable",
                                       bg='#607d8b',
                                       fg='white',
                                       font=('Segoe UI', 9, 'bold'),  # Smaller font
                                       command=lambda: self.disable_platoon_with_feedback(car_id),
                                       cursor='hand2',
                                       relief='flat',
                                       padx=12,
                                       pady=4)
        disable_platoon_btn.pack(side='left', expand=True, fill='x', padx=(4, 0))
        
        return frame
    
    def toggle_car_panel(self, car_id, content_frame, expand_btn):
        """Compact panel toggle"""
        is_expanded = self.car_expanded.get(car_id, True)
        self.car_expanded[car_id] = not is_expanded
        
        if not is_expanded:
            content_frame.pack(fill='x', padx=10, pady=10)
            expand_btn.config(text="▼")
        else:
            content_frame.pack_forget()
            expand_btn.config(text="▶")
    
    def create_enhanced_fleet_controls(self, parent):
        """Enhanced fleet control panel"""
        frame = tk.LabelFrame(parent,
                             text="🚁 Fleet Operations",
                             bg='#2d2d2d',
                             fg='white',
                             font=('Segoe UI', 14, 'bold'))
        
        content = tk.Frame(frame, bg='#2d2d2d')
        content.pack(fill='x', padx=15, pady=10)
        
        # Row 1: Basic fleet controls
        row1 = tk.Frame(content, bg='#2d2d2d')
        row1.pack(fill='x', pady=(0, 8))
        
        start_all_btn = tk.Button(row1,
                                 text="▶ Start All",
                                 bg='#4caf50',
                                 fg='white',
                                 font=('Segoe UI', 11, 'bold'),
                                 command=self.start_all_cars_with_feedback,
                                 cursor='hand2',
                                 relief='flat',
                                 padx=20,
                                 pady=8)
        start_all_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        stop_all_btn = tk.Button(row1,
                                text="⬛ Stop All",
                                bg='#f44336',
                                fg='white',
                                font=('Segoe UI', 11, 'bold'),
                                command=self.stop_all_cars_with_feedback,
                                cursor='hand2',
                                relief='flat',
                                padx=20,
                                pady=8)
        stop_all_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        # Row 2: Emergency and platoon controls
        row2 = tk.Frame(content, bg='#2d2d2d')
        row2.pack(fill='x', pady=(8, 0))
        
        emergency_all_btn = tk.Button(row2,
                                     text="🚨 EMERGENCY STOP ALL",
                                     bg='#ff5722',
                                     fg='white',
                                     font=('Segoe UI', 11, 'bold'),
                                     command=self.emergency_stop_all_with_feedback,
                                     cursor='hand2',
                                     relief='flat',
                                     padx=20,
                                     pady=8)
        emergency_all_btn.pack(fill='x', pady=(0, 8))
        
        # Platoon controls - First row
        platoon_row = tk.Frame(row2, bg='#2d2d2d')
        platoon_row.pack(fill='x', pady=(0, 5))
        
        convoy_btn = tk.Button(platoon_row,
                              text="🚗🚗 Setup Convoy",
                              bg='#9c27b0',
                              fg='white',
                              font=('Segoe UI', 10, 'bold'),
                              command=self.setup_convoy_with_feedback,
                              cursor='hand2',
                              relief='flat',
                              padx=15,
                              pady=6)
        convoy_btn.pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        self.v2v_btn = tk.Button(platoon_row,
                           text="📡 V2V Active",
                           bg='#ff9800',
                           fg='white',
                           font=('Segoe UI', 10, 'bold'),
                           command=self.activate_v2v_with_feedback,
                           cursor='hand2',
                           relief='flat',
                           padx=15,
                           pady=6)
        self.v2v_btn.pack(side='left', expand=True, fill='x', padx=(3, 0))
        
        # Platoon controls - Second row
        platoon_row2 = tk.Frame(row2, bg='#2d2d2d')
        platoon_row2.pack(fill='x')
        
        disable_all_platoons_btn = tk.Button(platoon_row2,
                                           text="Disable All Platoons",
                                           bg='#607d8b',
                                           fg='white',
                                           font=('Segoe UI', 10, 'bold'),
                                           command=self.disable_all_platoons_with_feedback,
                                           cursor='hand2',
                                           relief='flat',
                                           padx=15,
                                           pady=6)
        disable_all_platoons_btn.pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        self.disable_v2v_btn = tk.Button(platoon_row2,
                                   text="📡 Disable V2V",
                                   bg='#795548',
                                   fg='white',
                                   font=('Segoe UI', 10, 'bold'),
                                   command=self.disable_v2v_with_feedback,
                                   cursor='hand2',
                                   relief='flat',
                                   padx=15,
                                   pady=6)
        self.disable_v2v_btn.pack(side='left', expand=True, fill='x', padx=(3, 0))
        self.disable_v2v_btn.config(state='disabled', bg='#4d4d4d') # Disabled initially
        
        return frame
    
    def create_enhanced_status_panel(self, parent):
        """Enhanced connection status panel"""
        frame = tk.LabelFrame(parent,
                             text="📡 System Status",
                             bg='#2d2d2d',
                             fg='white',
                             font=('Segoe UI', 14, 'bold'))
        
        content = tk.Frame(frame, bg='#2d2d2d')
        content.pack(fill='x', padx=15, pady=10)
        
        # Fleet status
        self.fleet_status_label = tk.Label(content,
                                          text="Fleet: 0/0 connected",
                                          bg='#2d2d2d',
                                          fg='white',
                                          font=('Segoe UI', 12, 'bold'))
        self.fleet_status_label.pack(anchor='w', pady=(0, 5))
        
        # Command statistics
        self.command_stats_label = tk.Label(content,
                                           text="Commands: 0 sent, 0 failed",
                                           bg='#2d2d2d',
                                           fg='#cccccc',
                                           font=('Segoe UI', 10))
        self.command_stats_label.pack(anchor='w', pady=(0, 5))
        
        # Success rate
        self.success_rate_label = tk.Label(content,
                                          text="Success rate: 100%",
                                          bg='#2d2d2d',
                                          fg='#4caf50',
                                          font=('Segoe UI', 10))
        self.success_rate_label.pack(anchor='w', pady=(0, 5))
        
        # Server info
        self.server_info_label = tk.Label(content,
                                         text=f"Server: {self.controller.host_ip}:{self.controller.base_port}",
                                         bg='#2d2d2d',
                                         fg='#888888',
                                         font=('Segoe UI', 9))
        self.server_info_label.pack(anchor='w')
        
        return frame
    
    def create_enhanced_log_panel(self, parent):
        """Enhanced log panel with filtering"""
        frame = tk.LabelFrame(parent,
                             text="📝 Activity Log",
                             bg='#2d2d2d',
                             fg='white',
                             font=('Segoe UI', 14, 'bold'))
        
        # Log controls
        controls = tk.Frame(frame, bg='#2d2d2d')
        controls.pack(fill='x', padx=15, pady=10)
        
        clear_btn = tk.Button(controls,
                             text="Clear Log",
                             bg='#607d8b',
                             fg='white',
                             font=('Segoe UI', 9),
                             command=self.clear_log,
                             cursor='hand2',
                             relief='flat',
                             padx=15,
                             pady=4)
        clear_btn.pack(side='right')
        
        # Enhanced log text area
        self.log_text = scrolledtext.ScrolledText(frame,
                                                 width=50,
                                                 height=25,
                                                 bg='#1a1a1a',
                                                 fg='#ffffff',
                                                 font=('Consolas', 9),
                                                 insertbackground='white',
                                                 selectbackground='#4d4d4d')
        self.log_text.pack(fill='both', expand=True, padx=15, pady=(0, 15))
        
        return frame
    
    # ===== ENHANCED COMMAND METHODS WITH FEEDBACK =====
    
    def start_car_with_feedback(self, car_id):
        """Start car with enhanced feedback"""
        success = self.controller.start_car(car_id)
        if success:
            self.commands_sent_gui += 1
            self.log(f"✅ Started Car {car_id}", 'SUCCESS')
        else:
            self.commands_failed_gui += 1
            self.log(f"❌ Failed to start Car {car_id}", 'ERROR')
    
    def stop_car_with_feedback(self, car_id):
        """Stop car with enhanced feedback"""
        success = self.controller.stop_car(car_id)
        if success:
            self.commands_sent_gui += 1
            self.log(f"🛑 Stopped Car {car_id}", 'SUCCESS')
        else:
            self.commands_failed_gui += 1
            self.log(f"❌ Failed to stop Car {car_id}", 'ERROR')
    
    def set_velocity_with_feedback(self, car_id, velocity_str):
        """Set velocity with enhanced validation and feedback"""
        try:
            velocity = float(velocity_str)
            if 0 <= velocity <= 2.0:
                success = self.controller.set_velocity(car_id, velocity)
                if success:
                    self.commands_sent_gui += 1
                    self.log(f"🎯 Set Car {car_id} velocity to {velocity:.2f} m/s", 'SUCCESS')
                else:
                    self.commands_failed_gui += 1
                    self.log(f"❌ Failed to set velocity for Car {car_id}", 'ERROR')
            else:
                self.log(f"❌ Invalid velocity {velocity:.2f} (must be 0.0-2.0 m/s)", 'ERROR')
        except ValueError:
            self.log(f"❌ Invalid velocity value: {velocity_str}", 'ERROR')
    
    def set_path_with_feedback(self, car_id, path_str):
        """Set path with enhanced validation and feedback"""
        try:
            nodes = [int(n) for n in path_str.split()]
            if len(nodes) >= 2:
                success = self.controller.set_path(car_id, nodes)
                if success:
                    self.commands_sent_gui += 1
                    self.log(f"🛤️ Set Car {car_id} path: {nodes}", 'SUCCESS')
                else:
                    self.commands_failed_gui += 1
                    self.log(f"❌ Failed to set path for Car {car_id}", 'ERROR')
            else:
                self.log(f"❌ Path must have at least 2 nodes", 'ERROR')
        except ValueError:
            self.log(f"❌ Invalid path format: {path_str}", 'ERROR')
    
    def enable_platoon_with_feedback(self, car_id, role, leader_id_str):
        """Enable platoon with enhanced feedback"""
        if role == 'leader':
            success = self.controller.enable_platoon_leader(car_id)
            if success:
                self.commands_sent_gui += 1
                self.log(f"🚗 Enabled Car {car_id} as platoon LEADER", 'SUCCESS')
            else:
                self.commands_failed_gui += 1
                self.log(f"❌ Failed to enable platoon leader for Car {car_id}", 'ERROR')
        else:
            try:
                leader_id = int(leader_id_str)
                if leader_id != car_id and 0 <= leader_id < self.num_cars:
                    success = self.controller.enable_platoon_follower(car_id, leader_id)
                    if success:
                        self.commands_sent_gui += 1
                        self.log(f"🚗 Enabled Car {car_id} as FOLLOWER of Car {leader_id}", 'SUCCESS')
                    else:
                        self.commands_failed_gui += 1
                        self.log(f"❌ Failed to enable platoon follower for Car {car_id}", 'ERROR')
                else:
                    self.log(f"❌ Invalid leader ID {leader_id} for Car {car_id}", 'ERROR')
            except ValueError:
                self.log(f"❌ Invalid leader ID: {leader_id_str}", 'ERROR')
    
    def disable_platoon_with_feedback(self, car_id):
        """Disable platoon with enhanced feedback"""
        success = self.controller.disable_platoon(car_id)
        if success:
            self.commands_sent_gui += 1
            self.log(f"🚗 Disabled platoon for Car {car_id}", 'SUCCESS')
        else:
            self.commands_failed_gui += 1
            self.log(f"❌ Failed to disable platoon for Car {car_id}", 'ERROR')
    
    # ===== FLEET OPERATIONS WITH FEEDBACK =====
    
    def start_all_cars_with_feedback(self):
        """Start all cars with enhanced feedback"""
        results = self.controller.start_all_cars()
        successes = sum(1 for success in results.values() if success)
        self.commands_sent_gui += successes
        self.commands_failed_gui += len(results) - successes
        self.log(f"▶️ Start all: {successes}/{len(results)} cars started successfully", 'INFO')
    
    def stop_all_cars_with_feedback(self):
        """Stop all cars with enhanced feedback"""
        results = self.controller.stop_all_cars()
        successes = sum(1 for success in results.values() if success)
        self.commands_sent_gui += successes
        self.commands_failed_gui += len(results) - successes
        self.log(f"⬛ Stop all: {successes}/{len(results)} cars stopped successfully", 'INFO')
    
    def emergency_stop_all_with_feedback(self):
        """Emergency stop all cars with enhanced feedback"""
        result = messagebox.askquestion("Emergency Stop", 
                                       "🚨 EMERGENCY STOP ALL CARS?\n\nThis will immediately stop all vehicles.",
                                       icon='warning')
        if result == 'yes':
            results = self.controller.emergency_stop_all()
            successes = sum(1 for success in results.values() if success)
            self.commands_sent_gui += successes
            self.commands_failed_gui += len(results) - successes
            self.log(f"🚨 EMERGENCY STOP ALL: {successes}/{len(results)} cars stopped", 'WARNING')
    
    def setup_convoy_with_feedback(self):
        """Setup convoy with enhanced feedback"""
        if self.num_cars < 2:
            self.log("❌ Need at least 2 cars for convoy", 'ERROR')
            return
        
        # Simple convoy: Car 0 as leader, others as followers
        leader_id = 0
        follower_ids = list(range(1, self.num_cars))
        
        success = self.controller.setup_convoy(leader_id, follower_ids)
        if success:
            self.commands_sent_gui += len(follower_ids) + 1
            self.log(f"🚗🚗 Convoy setup: Car {leader_id} leading Cars {follower_ids}", 'SUCCESS')
        else:
            self.commands_failed_gui += 1
            self.log("❌ Failed to setup convoy", 'ERROR')
    
    def disable_all_platoons_with_feedback(self):
        """Disable all platoons with enhanced feedback"""
        results = self.controller.disable_all_platoons()
        successes = sum(1 for success in results.values() if success)
        self.commands_sent_gui += successes
        self.commands_failed_gui += len(results) - successes
        self.log(f"🚗 Disabled platoons: {successes}/{len(results)} cars", 'INFO')
    
    def activate_v2v_with_feedback(self):
        """Activate V2V communication for all connected vehicles"""
        # Disable button immediately to prevent double-clicking
        self.v2v_btn.config(state='disabled', bg='#4d4d4d', text="📡 Activating...")
        
        self.log("📡 Activating V2V communication for all vehicles...", 'INFO')
        
        # Get list of connected vehicles
        connected_cars = []
        for car_id in range(self.num_cars):
            if self.controller.is_car_connected(car_id):
                connected_cars.append(car_id)
        
        if len(connected_cars) < 2:
            self.log("❌ V2V requires at least 2 connected vehicles", 'ERROR')
            messagebox.showwarning("V2V Error", "V2V communication requires at least 2 connected vehicles")
            return
        
        # Reset V2V status tracking
        self.v2v_status = {car_id: {'status': 'activating', 'peers': 0} for car_id in connected_cars}
        
        # Send V2V activation command with list of all connected vehicle IPs
        success_count = 0
        for car_id in connected_cars:
            # Filter out self from peer list to ensure correct mapping
            peers = [cid for cid in connected_cars if cid != car_id]
            
            # Get REAL vehicle IP addresses for the peers from the controller
            vehicle_ips = []
            for peer_id in peers:
                status = self.controller.get_car_status(peer_id)
                if status and status.get('address'):
                    # address is a tuple (ip, port), we need the IP
                    vehicle_ips.append(status['address'][0])
                else:
                    # Fallback if address not found (should not happen for connected cars)
                    self.log(f"Warning: Could not find IP for peer {peer_id}, using default", 'WARNING')
                    vehicle_ips.append(f"192.168.1.{100 + peer_id}")
            
            command = {
                'command': 'activate_v2v',
                'peer_vehicles': peers,
                'peer_ips': vehicle_ips,
                'my_id': car_id
            }
            
            success = self.controller.send_command(car_id, command)
            if success:
                success_count += 1
                self.commands_sent_gui += 1
            else:
                self.commands_failed_gui += 1
                self.v2v_status[car_id] = {'status': 'failed', 'peers': 0}
        
        if success_count > 0:
            self.log(f"V2V activation sent to {success_count}/{len(connected_cars)} vehicles", 'SUCCESS')
            self.log(f"Expected vehicles: {connected_cars}", 'INFO')
            self.log(f"Waiting for V2V connection reports...", 'INFO')
            
            # Start timeout timer to re-enable button if no response
            # Set timeout to re-enable button if no response
            self._v2v_timeout_id = self.root.after(10000, self._v2v_activation_timeout)  # 10 second timeout for better reliability
        else:
            self.log("Failed to send V2V activation to any vehicle", 'ERROR')
            # Re-enable button on failure
            self.v2v_btn.config(state='normal', bg='#ff9800', text="📡 V2V Active")
    
    def _v2v_activation_timeout(self):
        """Handle V2V activation timeout"""
        if self.v2v_btn['text'] == '📡 Activating...':
            self.v2v_btn.config(state='normal', bg='#ff9800', text='📡 V2V Active')
            self.log('⏰ V2V activation timeout after 15 seconds - button re-enabled', 'WARNING')
            # Clear timeout reference
            if hasattr(self, '_v2v_timeout_id'):
                delattr(self, '_v2v_timeout_id')
    
    def disable_v2v_with_feedback(self):
        """Disable V2V communication for all vehicles"""
        self.log(" Disabling V2V communication for all vehicles...", 'INFO')
        
        success_count = 0
        for car_id in range(self.num_cars):
            if self.controller.is_car_connected(car_id):
                success = self.controller.send_command(car_id, {'command': 'disable_v2v'})
                if success:
                    success_count += 1
                    self.commands_sent_gui += 1
                else:
                    self.commands_failed_gui += 1
        
        if success_count > 0:
            self.log(f"✅ V2V disabled for {success_count} vehicles", 'SUCCESS')
            # Reset button states
            self.v2v_btn.config(state='normal', bg='#ff9800', text='📡 V2V Active')
            self.disable_v2v_btn.config(state='disabled', bg='#4d4d4d')
            # Reset status tracking
            self.v2v_status = {}
            if hasattr(self, '_v2v_success_logged'):
                delattr(self, '_v2v_success_logged')
        else:
            self.log("❌ Failed to disable V2V", 'ERROR')
    
    # ===== ENHANCED LOGGING =====
    
    def log(self, message, level='INFO'):
        """Enhanced logging with color coding"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color coding based on level
        colors = {
            'INFO': '#ffffff',
            'SUCCESS': '#4caf50',
            'WARNING': '#ff9800',
            'ERROR': '#f44336'
        }
        
        formatted_msg = f"[{timestamp}] {message}\n"
        
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, formatted_msg)
        
        # Apply color to the last line
        line_start = self.log_text.index("end-2c linestart")
        line_end = self.log_text.index("end-1c")
        self.log_text.tag_add(level, line_start, line_end)
        self.log_text.tag_config(level, foreground=colors.get(level, '#ffffff'))
        
        self.log_text.config(state='disabled')
        self.log_text.see(tk.END)
    
    def clear_log(self):
        """Clear the log"""
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state='disabled')
        self.log("Log cleared", 'INFO')
    
    # ===== ENHANCED UPDATE LOOP =====
    
    def update_loop(self):
        """Enhanced update loop with better status tracking"""
        while self.running:
            try:
                # Check for connection changes and update car panels accordingly
                self.update_car_panels()
                
                # Update connection indicators and telemetry for connected cars only
                for car_id in self.connected_cars:
                    if car_id in self.conn_indicators:
                        status = self.controller.get_car_status(car_id)
                        telemetry = self.controller.get_telemetry(car_id)
                        
                        # Update connection indicator (should always be connected if in this list)
                        self.conn_indicators[car_id].config(
                            text="� Connected",
                            fg='#4caf50'
                        )
                        
                        # Update car state
                        if car_id in self.car_state_labels:
                            state = telemetry.get('state', 'Unknown') if telemetry else 'Unknown'
                            self.car_state_labels[car_id].config(text=f"State: {state}")
                        
                        # Update telemetry
                        if car_id in self.telemetry_labels and telemetry:
                            labels = self.telemetry_labels[car_id]
                            
                            # Position
                            x, y = telemetry.get('x', 0), telemetry.get('y', 0)
                            labels['position'].config(text=f"({x:.2f}, {y:.2f})")
                            
                            # Velocity
                            velocity = telemetry.get('v', 0)
                            labels['velocity'].config(text=f"{velocity:.2f}")
                            
                            # Heading
                            heading = telemetry.get('th', 0)
                            labels['heading'].config(text=f"{heading:.2f}")
                            
                            # Throttle
                            throttle = telemetry.get('u', 0)
                            labels['throttle'].config(text=f"{throttle:.2f}")
                            
                            # Steering
                            steering = telemetry.get('delta', 0)
                            labels['steering'].config(text=f"{steering:.2f}")
                            
                            # State
                            state = telemetry.get('state', 'Unknown')
                            labels['state'].config(text=state)
                
                # Update fleet status
                fleet_stats = self.controller.get_fleet_status()
                connected_count = len(self.connected_cars)
                total_count = fleet_stats['total_cars']
                
                self.fleet_status_label.config(
                    text=f"Fleet: {connected_count}/{total_count} connected"
                )
                
                # Update command statistics
                total_sent = self.commands_sent_gui + fleet_stats['commands_sent_total']
                total_failed = self.commands_failed_gui + fleet_stats['commands_failed_total']
                
                self.command_stats_label.config(
                    text=f"Commands: {total_sent} sent, {total_failed} failed"
                )
                
                # Update success rate
                if total_sent + total_failed > 0:
                    success_rate = (total_sent / (total_sent + total_failed)) * 100
                    color = '#4caf50' if success_rate > 90 else '#ff9800' if success_rate > 70 else '#f44336'
                    self.success_rate_label.config(
                        text=f"Success rate: {success_rate:.1f}%",
                        fg=color
                    )
                
                # Update header statistics
                uptime = time.time() - self.start_time
                self.stats_label.config(
                    text=f"Commands: {total_sent} sent, {total_failed} failed | Uptime: {uptime:.0f}s"
                )
                
                # Check V2V status across fleet using both telemetry and status reports
                v2v_fully_connected = False
                if self.connected_cars and len(self.connected_cars) >= 2:
                    cars_with_v2v = 0
                    debug_info = []
                    
                    for car_id in self.connected_cars:
                        # Check both telemetry data and V2V status reports
                        telemetry = self.controller.get_telemetry(car_id)
                        v2v_status = self.v2v_status.get(car_id, {})
                        
                        # Use telemetry if available, fallback to status reports
                        v2v_active = False
                        v2v_peers = 0
                        
                        if telemetry and telemetry.get('v2v_active', False):
                            v2v_active = True
                            v2v_peers = telemetry.get('v2v_peers', 0)
                        elif v2v_status.get('status') == 'connected':
                            v2v_active = True
                            v2v_peers = v2v_status.get('peers', 0)
                        
                        debug_info.append(f"Car {car_id}: active={v2v_active}, peers={v2v_peers}")
                        
                        if v2v_active and v2v_peers >= len(self.connected_cars) - 1:
                            cars_with_v2v += 1
                    
                    # Debug: Log V2V status every 30 seconds
                    if hasattr(self, '_debug_counter'):
                        self._debug_counter += 1
                    else:
                        self._debug_counter = 0
                        
                    if self._debug_counter % 30 == 0:  # Every 30 seconds
                        self.log(f"[DEBUG] V2V Status Check: {'; '.join(debug_info)} | Fully connected: {cars_with_v2v}/{len(self.connected_cars)}", 'INFO')
                    
                    if cars_with_v2v == len(self.connected_cars):
                        v2v_fully_connected = True
                
                # Update V2V buttons based on status
                if v2v_fully_connected:
                    if self.disable_v2v_btn['state'] == 'disabled':
                        self.disable_v2v_btn.config(state='normal', bg='#795548')
                        self.v2v_btn.config(state='disabled', bg='#4d4d4d', text="📡 V2V Connected")
                        if not hasattr(self, '_v2v_success_logged'):
                            self.log("✅ V2V Network Fully Established - All cars connected", 'SUCCESS')
                            self.log("🔘 Disable V2V button is now available", 'INFO')
                            self._v2v_success_logged = True
                            
                            # Cancel pending timeout since V2V succeeded
                            if hasattr(self, '_v2v_timeout_id'):
                                try:
                                    self.root.after_cancel(self._v2v_timeout_id)
                                    delattr(self, '_v2v_timeout_id')
                                except:
                                    pass
                else:
                    if self.disable_v2v_btn['state'] == 'normal':
                        self.disable_v2v_btn.config(state='disabled', bg='#4d4d4d')
                        self.log("🔘 Disable V2V button hidden - not fully connected", 'INFO')
                        if hasattr(self, '_v2v_success_logged'):
                            delattr(self, '_v2v_success_logged')
                    # Only re-enable V2V button if not currently activating
                    if self.v2v_btn['text'] != '📡 Activating...' and self.v2v_btn['state'] != 'normal':
                        self.v2v_btn.config(state='normal', bg='#ff9800', text="📡 V2V Active")
                
                time.sleep(1.0)  # Slower update rate to reduce panel flickering
                
            except Exception as e:
                print(f"Update loop error: {e}")
                time.sleep(1.0)
    
    def process_v2v_status(self, car_id: int, v2v_data: dict):
        """Process V2V status reports from vehicles"""
        try:
            status = v2v_data.get('status', 'unknown')
            self.log(f"📡 Car {car_id}: V2V status update - {status}", 'INFO')
            
            # Debug: Log detailed V2V data
            if status in ['connected', 'disconnected']:
                peers = v2v_data.get('connected_peers', 0)
                expected = v2v_data.get('expected_peers', 0)
                self.log(f"[DEBUG] Car {car_id} V2V details: {peers}/{expected} peers, status={status}", 'INFO')
            
            if status == 'connected':
                expected_peers = v2v_data.get('expected_peers', 0)
                connected_peers = v2v_data.get('connected_peers', 0)
                peer_list = v2v_data.get('peer_list', [])
                
                # Update V2V status
                self.v2v_status[car_id] = {
                    'status': 'connected',
                    'peers': connected_peers,
                    'expected': expected_peers,
                    'peer_list': peer_list
                }
                
                # Update GUI indicator
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: ON", fg='#4caf50')
                
                self.log(f"Car {car_id}: V2V connected to {connected_peers}/{expected_peers} peers: {peer_list}", 'SUCCESS')
                
                # Check if all vehicles have reported successful connections
                self.check_v2v_network_status()
                
            elif status == 'active':
                # Periodic status update
                connected_peers = v2v_data.get('connected_peers', 0)
                messages_sent = v2v_data.get('messages_sent', 0)
                messages_received = v2v_data.get('messages_received', 0)
                
                if car_id in self.v2v_status:
                    self.v2v_status[car_id]['peers'] = connected_peers
                    self.v2v_status[car_id]['msg_sent'] = messages_sent
                    self.v2v_status[car_id]['msg_recv'] = messages_received
                
                # Update GUI indicator for active state too if connected
                if connected_peers > 0 and hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: ON", fg='#4caf50')
                
            elif status == 'failed':
                error = v2v_data.get('error', 'unknown')
                self.v2v_status[car_id] = {'status': 'failed', 'error': error}
                
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: Err", fg='#f44336')
                     
                self.log(f"Car {car_id}: V2V connection failed - {error}", 'ERROR')
                
            elif status == 'disconnected':
                self.v2v_status[car_id] = {'status': 'disconnected', 'peers': 0}
                
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: Off", fg='#888888')
                     
                self.log(f"Car {car_id}: V2V disconnected", 'WARNING')
                
        except Exception as e:
            self.log(f"Error processing V2V status from car {car_id}: {e}", 'ERROR')
    
    def check_v2v_network_status(self):
        """Check if all vehicles have successfully connected via V2V"""
        try:
            connected_vehicles = [car_id for car_id in self.v2v_status 
                                if self.v2v_status[car_id].get('status') == 'connected']
            
            if len(connected_vehicles) < 2:
                return  # Need at least 2 vehicles
            
            # Check if all vehicles report the same number of connections
            peer_counts = [self.v2v_status[car_id].get('peers', 0) for car_id in connected_vehicles]
            expected_peers = len(connected_vehicles) - 1  # Each car should connect to all others
            
            if all(count == expected_peers for count in peer_counts):
                # Only log if this is a new success (not duplicate)
                if not hasattr(self, '_last_v2v_success_vehicles') or self._last_v2v_success_vehicles != set(connected_vehicles):
                    self.log(f" V2V NETWORK SUCCESS! All {len(connected_vehicles)} vehicles connected", 'SUCCESS')
                    self.log(f"Network topology: Each vehicle connected to {expected_peers} peers", 'SUCCESS')
                    self._last_v2v_success_vehicles = set(connected_vehicles)
            else:
                self.log(f"V2V network incomplete - peer counts: {dict(zip(connected_vehicles, peer_counts))}", 'WARNING')
                if hasattr(self, '_last_v2v_success_vehicles'):
                    delattr(self, '_last_v2v_success_vehicles')
                
        except Exception as e:
            self.log(f"Error checking V2V network status: {e}", 'ERROR')
    
    def on_closing(self):
        """Enhanced cleanup on window close"""
        self.running = False
        
        # Log shutdown
        self.log("🛑 Shutting down Ground Station...", 'INFO')
        
        # Close controller
        self.controller.close()
        
        # Destroy window
        self.root.destroy()


# Configuration
HOST_IP = '0.0.0.0'  # Listen on all network interfaces
BASE_PORT = 5000
NUM_CARS = 2


def main():
    """Main entry point for enhanced GUI"""
    root = tk.Tk()
    app = EnhancedQCarGUIController(root, num_cars=NUM_CARS, host_ip=HOST_IP, base_port=BASE_PORT)
    
    # Enhanced startup logging
    app.log(f"Enhanced QCar Fleet Controller started", 'SUCCESS')
    app.log(f"Listening on ports {BASE_PORT}-{BASE_PORT + NUM_CARS - 1}", 'INFO')
    app.log(f"Waiting for {NUM_CARS} cars to connect...", 'INFO')
    app.log(f"Enhanced features: Command validation, platoon control, statistics", 'INFO')
    
    root.mainloop()


if __name__ == '__main__':
    main()