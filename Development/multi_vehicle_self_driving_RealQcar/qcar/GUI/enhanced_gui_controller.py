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

try:
    import pygame  # For steering wheel support
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("Warning: pygame not available. Steering wheel support disabled.")

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
        
        # Platoon configuration tracking
        self.platoon_config = {}  # car_id -> {'position': int, 'is_leader': bool, 'setup_complete': bool}
        self.platoon_indicators = {}  # car_id -> Label widget for platoon status display
        
        # Manual mode control type tracking
        self.manual_control_types = {}  # car_id -> 'keyboard' or 'wheel'
        self.control_type_vars = {}  # car_id -> tkinter StringVar for control type selection
        
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
        
        # Statistics bar with telemetry rate
        self.stats_label = tk.Label(title_frame,
                                   text="Commands: 0 sent, 0 failed | Uptime: 0s | Telemetry: 0.0 Hz",
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
        
        # Scrollable car panels area (car panels spawn automatically when vehicles connect)
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
    
    # def create_car_count_panel(self, parent):
    #     """Enhanced car count control panel"""
    #     frame = tk.Frame(parent, bg='#2d2d2d', relief='raised', bd=2)
    #     frame.pack(fill='x', pady=(0, 5))
        
    #     content = tk.Frame(frame, bg='#2d2d2d')
    #     content.pack(fill='x', padx=15, pady=10)
        
    #     tk.Label(content,
    #             text="Fleet Size:",
    #             bg='#2d2d2d',
    #             fg='white',
    #             font=('Segoe UI', 12, 'bold')).pack(side='left', padx=(0, 10))
        
    #     # Enhanced car count spinbox
    #     self.car_count_var = tk.StringVar(value=str(self.num_cars))
    #     spinbox = tk.Spinbox(content,
    #                         from_=1,
    #                         to=self.max_cars,
    #                         textvariable=self.car_count_var,
    #                         width=5,
    #                         bg='#3d3d3d',
    #                         fg='white',
    #                         font=('Segoe UI', 11),
    #                         buttonbackground='#4d4d4d',
    #                         relief='flat',
    #                         insertbackground='white')
    #     spinbox.pack(side='left', padx=(0, 15))
        
    #     # Enhanced apply button
    #     apply_btn = tk.Button(content,
    #                          text="Apply",
    #                          bg='#2196f3',
    #                          fg='white',
    #                          font=('Segoe UI', 10, 'bold'),
    #                          command=self.apply_car_count,
    #                          cursor='hand2',
    #                          relief='flat',
    #                          padx=20,
    #                          pady=5)
    #     apply_btn.pack(side='left', padx=(0, 15))
        
    #     # Enhanced info label
    #     self.car_count_info = tk.Label(content,
    #                                    text=f"Active: {self.num_cars} cars",
    #                                    bg='#2d2d2d',
    #                                    fg='#4caf50',
    #                                    font=('Segoe UI', 10, 'bold'))
    #     self.car_count_info.pack(side='left')
    
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
        
        # V2V Status Indicator
        v2v_indicator = tk.Label(header,
                                text="📡 V2V: OFF",
                                bg='#1a1a1a',
                                fg='#888888', # Gray initially
                                font=('Segoe UI', 12, 'bold'),
                                padx=10)
        v2v_indicator.pack(side='right', padx=(0, 10), pady=20)
        
        if not hasattr(self, 'v2v_indicators'):
            self.v2v_indicators = {}
        self.v2v_indicators[car_id] = v2v_indicator
        
        # Platoon Status Indicator (New)
        platoon_indicator = tk.Label(header,
                                     text="🚗 Solo",
                                     bg='#1a1a1a',
                                     fg='#888888', # Gray initially
                                     font=('Segoe UI', 12, 'bold'),
                                     padx=10)
        platoon_indicator.pack(side='right', padx=(0, 10), pady=20)
        
        if not hasattr(self, 'platoon_indicators'):
            self.platoon_indicators = {}
        self.platoon_indicators[car_id] = platoon_indicator
        
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
            
            # Set fixed width based on label type to prevent shifting
            if key == 'position':
                label_width = 14  # For "(X.XX, Y.YY)"
            elif key == 'state':
                label_width = 15  # For vehicle state text
            else:
                label_width = 8   # For numeric values
            
            value_label = tk.Label(label_frame,
                                  text=default_value,
                                  bg='#2d2d2d',
                                  fg='white',
                                  font=('Segoe UI', 10, 'bold'),
                                  width=label_width,
                                  anchor='w')  # Left-align text within fixed width
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
        
        # Calibrate button
        calibrate_btn = tk.Button(main_buttons,
                                 text="🔧 Calibrate",
                                 bg='#ff9800',
                                 fg='white',
                                 font=('Segoe UI', 11, 'bold'),
                                 command=lambda: self.calibrate_car_with_feedback(car_id),
                                 cursor='hand2',
                                 relief='flat',
                                 padx=15,
                                 pady=6)
        calibrate_btn.pack(side='left', expand=True, fill='x', padx=(5, 5))
        
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
        
        # Manual Mode Control Panel
        manual_frame = tk.LabelFrame(left_section,
                                    text="🎮 Manual Control",
                                    bg='#2d2d2d',
                                    fg='white',
                                    font=('Segoe UI', 11, 'bold'))
        manual_frame.pack(fill='x', pady=(8, 0))
        
        manual_content = tk.Frame(manual_frame, bg='#2d2d2d')
        manual_content.pack(fill='x', padx=8, pady=6)
        
        # Control type selection
        control_type_frame = tk.Frame(manual_content, bg='#2d2d2d')
        control_type_frame.pack(fill='x', pady=(0, 5))
        
        tk.Label(control_type_frame,
                text="Control:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        # Store control type variable
        control_type_var = tk.StringVar(value='keyboard')
        self.control_type_vars[car_id] = control_type_var
        self.manual_control_types[car_id] = 'keyboard'
        
        # Radio buttons for control type
        keyboard_radio = tk.Radiobutton(control_type_frame,
                                       text="⌨️ Keyboard",
                                       variable=control_type_var,
                                       value='keyboard',
                                       bg='#2d2d2d',
                                       fg='white',
                                       selectcolor='#3d3d3d',
                                       font=('Segoe UI', 9),
                                       command=lambda: self._update_control_type(car_id, 'keyboard'))
        keyboard_radio.pack(side='left', padx=(0, 10))
        
        wheel_radio = tk.Radiobutton(control_type_frame,
                                    text="🎡 Wheel",
                                    variable=control_type_var,
                                    value='wheel',
                                    bg='#2d2d2d',
                                    fg='white',
                                    selectcolor='#3d3d3d',
                                    font=('Segoe UI', 9),
                                    command=lambda: self._update_control_type(car_id, 'wheel'))
        wheel_radio.pack(side='left')
        
        # Manual mode toggle button
        manual_btn = tk.Button(manual_content,
                              text="🎮 Manual Mode",
                              bg='#9c27b0',
                              fg='white',
                              font=('Segoe UI', 10, 'bold'),
                              command=lambda: self.toggle_manual_mode_with_feedback(car_id),
                              cursor='hand2',
                              relief='flat',
                              padx=15,
                              pady=5)
        manual_btn.pack(fill='x', pady=(5, 0))
        
        # Store manual mode button reference for updating state
        if not hasattr(self, 'manual_mode_buttons'):
            self.manual_mode_buttons = {}
        self.manual_mode_buttons[car_id] = manual_btn
        
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
                                  text="🛤️ Path & Position",
                                  bg='#2d2d2d',
                                  fg='white',
                                  font=('Segoe UI', 11, 'bold'))
        path_frame.pack(fill='x', pady=(0, 5))
        
        path_content = tk.Frame(path_frame, bg='#2d2d2d')
        path_content.pack(fill='x', padx=8, pady=6)
        
        # Initial Position Row
        init_pos_row = tk.Frame(path_content, bg='#2d2d2d')
        init_pos_row.pack(fill='x', pady=(0, 5))
        
        tk.Label(init_pos_row,
                text="Init Pos:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 5))
        
        # X position
        tk.Label(init_pos_row,
                text="X:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 9)).pack(side='left', padx=(0, 2))
        
        init_x_entry = tk.Entry(init_pos_row,
                               width=5,
                               bg='#3d3d3d',
                               fg='white',
                               font=('Segoe UI', 9),
                               insertbackground='white',
                               relief='flat')
        init_x_entry.insert(0, "0.0")
        init_x_entry.pack(side='left', padx=(0, 5))
        
        # Y position
        tk.Label(init_pos_row,
                text="Y:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 9)).pack(side='left', padx=(0, 2))
        
        init_y_entry = tk.Entry(init_pos_row,
                               width=5,
                               bg='#3d3d3d',
                               fg='white',
                               font=('Segoe UI', 9),
                               insertbackground='white',
                               relief='flat')
        init_y_entry.insert(0, "0.0")
        init_y_entry.pack(side='left', padx=(0, 5))
        
        # Theta (heading)
        tk.Label(init_pos_row,
                text="θ:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 9)).pack(side='left', padx=(0, 2))
        
        init_theta_entry = tk.Entry(init_pos_row,
                                   width=5,
                                   bg='#3d3d3d',
                                   fg='white',
                                   font=('Segoe UI', 9),
                                   insertbackground='white',
                                   relief='flat')
        init_theta_entry.insert(0, "0.0")
        init_theta_entry.pack(side='left', padx=(0, 5))
        
        # Calibrate GPS checkbox
        init_calibrate_var = tk.BooleanVar(value=False)  # Default to False
        init_calibrate_check = tk.Checkbutton(init_pos_row,
                                             text="Calibrate GPS",
                                             variable=init_calibrate_var,
                                             bg='#2d2d2d',
                                             fg='#4caf50',
                                             selectcolor='#1e1e1e',
                                             activebackground='#2d2d2d',
                                             activeforeground='#4caf50',
                                             font=('Segoe UI', 8))
        init_calibrate_check.pack(side='left', padx=(0, 5))
        
        # Set button for initial position
        init_pos_btn = tk.Button(init_pos_row,
                                text="Set",
                                bg='#2196f3',
                                fg='white',
                                font=('Segoe UI', 9, 'bold'),
                                command=lambda: self.set_initial_position_with_feedback(car_id, init_x_entry.get(), init_y_entry.get(), init_theta_entry.get(), init_calibrate_var),
                                cursor='hand2',
                                relief='flat',
                                padx=10,
                                pady=3)
        init_pos_btn.pack(side='left')
        
        # Path Nodes Row
        path_row = tk.Frame(path_content, bg='#2d2d2d')
        path_row.pack(fill='x')
        
        tk.Label(path_row,
                text="Nodes:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        path_entry = tk.Entry(path_row,
                             width=15,
                             bg='#3d3d3d',
                             fg='white',
                             font=('Segoe UI', 10),  # Smaller font
                             insertbackground='white',
                             relief='flat')
        path_entry.insert(0, "10, 2, 4, 6, 8, 10" if car_id == 0 else "10, 2, 4, 6, 8, 10")
        path_entry.pack(side='left', padx=(0, 8))
        
        path_btn = tk.Button(path_row,
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
        
        # Position selection - simplified
        position_frame = tk.Frame(platoon_content, bg='#2d2d2d')
        position_frame.pack(fill='x', pady=(0, 5))
        
        tk.Label(position_frame,
                text="Position:",
                bg='#2d2d2d',
                fg='#cccccc',
                font=('Segoe UI', 10)).pack(side='left', padx=(0, 8))
        
        position_var = tk.StringVar(value=str(car_id + 1))  # Default: Car 0 is position 1, Car 1 is position 2, etc.
        position_entry = tk.Entry(position_frame,
                                 textvariable=position_var,
                                 width=3,
                                 bg='#3d3d3d',
                                 fg='white',
                                 font=('Segoe UI', 10),
                                 insertbackground='white',
                                 relief='flat')
        position_entry.pack(side='left', padx=(0, 8))
        position_entry.bind('<KeyRelease>', lambda e: self.update_platoon_config(car_id, position_var.get()))
        
        # Role indicator - shows what position 1 means
        role_indicator = tk.Label(position_frame,
                                 text="(1=Leader, 2,3...=Followers)",
                                 bg='#2d2d2d',
                                 fg='#888888',
                                 font=('Segoe UI', 9))
        role_indicator.pack(side='left', padx=(8, 0))
        
        # Initialize platoon config for this car
        self.update_platoon_config(car_id, position_var.get())
        
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
        
        # emergency_all_btn = tk.Button(row2,
        #                              text="🚨 EMERGENCY STOP ALL",
        #                              bg='#ff5722',
        #                              fg='white',
        #                              font=('Segoe UI', 11, 'bold'),
        #                              command=self.emergency_stop_all_with_feedback,
        #                              cursor='hand2',
        #                              relief='flat',
        #                              padx=20,
        #                              pady=8)
        # emergency_all_btn.pack(fill='x', pady=(0, 8))
        
        # Platoon controls - First row
        platoon_row = tk.Frame(row2, bg='#2d2d2d')
        platoon_row.pack(fill='x', pady=(0, 5))
        
        setup_platoon_btn = tk.Button(platoon_row,
                                     text="⚙️ Setup Platoon",
                                     bg='#2196f3',
                                     fg='white',
                                     font=('Segoe UI', 10, 'bold'),
                                     command=self.setup_platoon_formation_with_feedback,
                                     cursor='hand2',
                                     relief='flat',
                                     padx=15,
                                     pady=6)
        setup_platoon_btn.pack(side='left', expand=True, fill='x', padx=(0, 2))
        
        trigger_platoon_btn = tk.Button(platoon_row,
                                       text="🚗🚗 Trigger Platoon",
                                       bg='#9c27b0',
                                       fg='white',
                                       font=('Segoe UI', 10, 'bold'),
                                       command=self.trigger_platoon_start_with_feedback,
                                       cursor='hand2',
                                       relief='flat',
                                       padx=15,
                                       pady=6)
        trigger_platoon_btn.pack(side='left', expand=True, fill='x', padx=(2, 0))
        
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
    
    def update_platoon_config(self, car_id: int, position_str: str):
        """Update platoon configuration for a car"""
        try:
            position = int(position_str) if position_str.strip() else car_id + 1
            if position < 1:
                position = 1
        except ValueError:
            position = car_id + 1
        
        self.platoon_config[car_id] = {
            'position': position
        }
        
        # Determine role based on position
        role = "LEADER" if position == 1 else "FOLLOWER"
        
        # Log configuration change
        self.log(f"Car {car_id} platoon config: Position {position} ({role})", 'CONFIG')
    
    def setup_platoon_formation_with_feedback(self):
        """Send global platoon formation to all vehicles (setup only, no start)"""
        if not self.platoon_config:
            self.log("❌ No platoon positions configured. Please set positions first.", 'ERROR')
            return
        
        # Validate positions - must start from 1 and be sequential
        positions = sorted([pos['position'] for pos in self.platoon_config.values() if pos.get('position') is not None])
        if not positions or positions[0] != 1:
            self.log("❌ Position 1 (leader) must be assigned", 'ERROR')
            return
        
        # Check for gaps in positions
        for i in range(len(positions) - 1):
            if positions[i + 1] - positions[i] > 1:
                self.log(f"❌ Gap in positions: missing position {positions[i] + 1}", 'ERROR')
                return
        
        # Create formation mapping: car_id -> position
        formation = {}
        leader_id = None
        
        for car_id, config in self.platoon_config.items():
            if config.get('position') is not None:
                formation[car_id] = config['position']
                if config['position'] == 1:
                    leader_id = car_id
        
        if leader_id is None:
            self.log("❌ No leader assigned", 'ERROR')
            return
        
        # Send global formation command to all vehicles
        self.log(f"📊 Setting up platoon formation: {formation}", 'INFO')
        self.log(f"👑 Leader: Car {leader_id}", 'INFO')
        
        # Send formation using the existing method
        results = self.controller.setup_global_platoon_formation(formation)
        
        # Count successes and log results
        success_count = 0
        for car_id, success in results.items():
            position = formation.get(car_id, 0)
            role_name = "LEADER" if position == 1 else f"FOLLOWER (pos {position})"
            
            if success:
                success_count += 1
                self.log(f"✅ Car {car_id}: Formation configured as {role_name}", 'SUCCESS')
            else:
                self.log(f"❌ Car {car_id}: Failed to configure {role_name}", 'ERROR')
        
        if success_count == len(formation):
            self.log(f"🎉 Platoon formation setup complete! {success_count}/{len(formation)} vehicles configured", 'SUCCESS')
            self.log(f"✅ Formation configured successfully! Leader: Car {leader_id}, Followers: {len(formation)-1} vehicles", 'SUCCESS')
            # Store setup state for trigger button
            self.platoon_setup_complete = True
            self.platoon_formation = formation
            self.platoon_leader_id = leader_id
        else:
            self.log(f"⚠️ Partial setup: {success_count}/{len(formation)} vehicles configured", 'WARNING')
            self.log(f"⚠️ Only {success_count}/{len(formation)} vehicles configured successfully. Check connections and try again.", 'WARNING')
            self.platoon_setup_complete = False

    def trigger_platoon_start_with_feedback(self):
        """Trigger platoon start after formation has been set up"""
        if not hasattr(self, 'platoon_setup_complete') or not self.platoon_setup_complete:
            self.log("❌ Platoon not set up - run Setup Platoon first", 'ERROR')
            return
        
        if not hasattr(self, 'platoon_formation') or not self.platoon_formation:
            self.log("❌ No platoon formation data available. Please setup platoon first", 'ERROR')
            return
        
        formation = self.platoon_formation
        leader_id = self.platoon_leader_id
        
        self.log(f"🚀 Triggering platoon start with formation: {formation}", 'INFO')
        self.log(f"👑 Leader: Car {leader_id}", 'INFO')
        
        # Send start platoon command to all vehicles in formation
        success_count = 0
        for car_id in formation.keys():
            try:
                # Send START_PLATOON command to trigger transition to following_leader_state
                result = self.controller.start_platoon_mode(car_id, leader_id)
                if result.get('status') == 'success':
                    role = "LEADER" if car_id == leader_id else "FOLLOWER"
                    self.log(f"✅ Car {car_id}: Platoon started ({role})", 'SUCCESS')
                    success_count += 1
                else:
                    self.log(f"❌ Car {car_id}: {result.get('message', 'Failed to start platoon')}", 'ERROR')
            except Exception as e:
                self.log(f"❌ Car {car_id}: Error starting platoon - {str(e)}", 'ERROR')
        
        if success_count == len(formation):
            self.log(f"🎉 Platoon started successfully! {success_count}/{len(formation)} vehicles active", 'SUCCESS')
            self.log(f"🚗🚗 Platoon is now active! {success_count}/{len(formation)} vehicles started, Leader: Car {leader_id}", 'SUCCESS')
            self.log(f"Vehicles should transition to following_leader_state", 'INFO')
        else:
            self.log(f"⚠️ Partial start: {success_count}/{len(formation)} vehicles started", 'WARNING')
            self.log(f"⚠️ Only {success_count}/{len(formation)} vehicles started successfully. Check vehicle status and connections.", 'WARNING')
    
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
    
    def calibrate_car_with_feedback(self, car_id):
        """Calibrate GPS with enhanced feedback"""
        success = self.controller.send_command(car_id, {'type': 'calibrate'})
        if success:
            self.commands_sent_gui += 1
            self.log(f"📍 GPS Calibration started for Car {car_id}", 'INFO')
        else:
            self.commands_failed_gui += 1
            self.log(f"❌ Failed to send calibrate command to Car {car_id}", 'ERROR')
    
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
            nodes = [int(n.strip()) for n in path_str.replace(',', ' ').split()]
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
    
    def set_initial_position_with_feedback(self, car_id, x_str, y_str, theta_str, calibrate_var=None):
        """Set initial position with enhanced validation and feedback
        
        Args:
            car_id: ID of the car
            x_str: X coordinate as string
            y_str: Y coordinate as string
            theta_str: Heading angle as string
            calibrate_var: tkinter BooleanVar for calibrate checkbox (None = default True)
        """
        try:
            x = float(x_str)
            y = float(y_str)
            theta = float(theta_str)
            
            # Get calibrate flag from checkbox or default to True
            calibrate = calibrate_var.get() if calibrate_var else True
            
            success = self.controller.set_initial_position(car_id, x, y, theta, calibrate)
            if success:
                self.commands_sent_gui += 1
                mode = "with GPS calibration" if calibrate else "without GPS calibration"
                self.log(f"📍 Set initial position for Car {car_id}: ({x:.2f}, {y:.2f}, θ={theta:.2f}) {mode}", 'SUCCESS')
            else:
                self.commands_failed_gui += 1
                self.log(f"❌ Failed to set initial position for Car {car_id}", 'ERROR')
        except ValueError:
            self.log(f"❌ Invalid position values: X={x_str}, Y={y_str}, θ={theta_str}", 'ERROR')
    
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
                        
                        # Update platoon indicator from telemetry (if available)
                        if car_id in self.platoon_indicators and telemetry:
                            platoon_enabled = telemetry.get('platoon_enabled', False)
                            platoon_is_leader = telemetry.get('platoon_is_leader', False)
                            platoon_position = telemetry.get('platoon_position')
                            platoon_setup_complete = telemetry.get('platoon_setup_complete', False)
                            
                            if platoon_enabled and platoon_setup_complete and platoon_position is not None:
                                if platoon_is_leader:
                                    indicator_text = "🚗 LEADER"
                                    indicator_color = '#ffa726'  # Orange
                                else:
                                    indicator_text = f"🚗 FOLLOWER-{platoon_position}"
                                    indicator_color = '#42a5f5'  # Blue
                                
                                self.platoon_indicators[car_id].config(
                                    text=indicator_text,
                                    fg=indicator_color
                                )
                            elif platoon_enabled and not platoon_setup_complete:
                                self.platoon_indicators[car_id].config(
                                    text="🚗 Setup...",
                                    fg='#ff9800'
                                )
                            else:
                                self.platoon_indicators[car_id].config(
                                    text="🚗 Solo",
                                    fg='#888888'
                                )
                        
                        # Update V2V indicator from telemetry data (throttled to avoid rapid flickering)
                        if car_id in self.v2v_indicators:
                            v2v_active = telemetry.get('v2v_active', False) if telemetry else False
                            v2v_peers = telemetry.get('v2v_peers', 0) if telemetry else 0
                            
                            # Throttle V2V indicator updates to prevent frequent changes
                            if not hasattr(self, '_v2v_update_counters'):
                                self._v2v_update_counters = {}
                            if car_id not in self._v2v_update_counters:
                                self._v2v_update_counters[car_id] = 0
                            
                            self._v2v_update_counters[car_id] += 1
                            
                            # Only update V2V indicator every 2 seconds (40 updates at 20Hz) to reduce flickering
                            if self._v2v_update_counters[car_id] % 40 == 0:  # Every 40 updates (2 seconds at 20Hz)
                                if v2v_active and v2v_peers > 0:
                                    new_text = f"📡 V2V: ON ({v2v_peers})"
                                    new_color = '#4caf50'
                                else:
                                    new_text = "📡 V2V: OFF"
                                    new_color = '#888888'
                                
                                # Only update if text actually changed to prevent unnecessary redraws
                                current_text = self.v2v_indicators[car_id]['text']
                                if current_text != new_text:
                                    self.v2v_indicators[car_id].config(text=new_text, fg=new_color)
                            
                            # Debug: Log V2V indicator updates (only occasionally)
                            if car_id == 0 and self._v2v_update_counters[car_id] % 200 == 0:  # Every 200 updates (10 seconds at 20Hz)
                                self.log(f"[DEBUG] Car {car_id} V2V status: active={v2v_active}, peers={v2v_peers}", 'INFO')
                        else:
                            # Debug: Log missing V2V indicator
                            if car_id == 0 and hasattr(self, '_missing_indicator_logged'):
                                pass  # Don't log repeatedly
                            elif car_id == 0:
                                self.log(f"[DEBUG] Car {car_id} V2V indicator missing from self.v2v_indicators", 'WARNING')
                                self._missing_indicator_logged = True
                
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
                
                # Update header statistics with telemetry rate
                uptime = time.time() - self.start_time
                avg_rate = fleet_stats.get('avg_telemetry_rate_hz', 0.0)
                self.stats_label.config(
                    text=f"Commands: {total_sent} sent, {total_failed} failed | Uptime: {uptime:.0f}s | Telemetry: {avg_rate:.1f} Hz"
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
                        else:
                            # Either disconnected or not active
                            v2v_active = False
                            v2v_peers = 0
                        
                        debug_info.append(f"Car {car_id}: active={v2v_active}, peers={v2v_peers}")
                        
                        if v2v_active and v2v_peers >= len(self.connected_cars) - 1:
                            cars_with_v2v += 1
                    
                    # Debug: Log V2V status every 30 seconds
                    if hasattr(self, '_debug_counter'):
                        self._debug_counter += 1
                    else:
                        self._debug_counter = 0
                        
                    if self._debug_counter % 400 == 0:  # Every 400 updates (20 seconds at 20Hz)
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
                
                time.sleep(0.05)  # 20 Hz GUI update rate - fast enough for smooth display, efficient for 50Hz telemetry
                
            except Exception as e:
                print(f"Update loop error: {e}")
                time.sleep(0.1)
    
    def process_platoon_setup_confirmation(self, car_id: int, platoon_data: dict):
        """Process platoon setup confirmation from vehicles"""
        try:
            position = platoon_data.get('position')
            is_leader = platoon_data.get('is_leader', False)
            leader_id = platoon_data.get('leader_id')
            setup_complete = platoon_data.get('setup_complete', False)
            
            # Update platoon config
            self.platoon_config[car_id] = {
                'position': position,
                'is_leader': is_leader,
                'leader_id': leader_id,
                'setup_complete': setup_complete
            }
            
            # Update platoon indicator
            if car_id in self.platoon_indicators:
                if setup_complete:
                    if is_leader:
                        indicator_text = f"🚗 LEADER"
                        indicator_color = '#ffa726'  # Orange for leader
                    else:
                        indicator_text = f"🚗 FOLLOWER-{position}"
                        indicator_color = '#42a5f5'  # Blue for followers
                    
                    self.platoon_indicators[car_id].config(
                        text=indicator_text,
                        fg=indicator_color
                    )
                    
                    role_str = "LEADER" if is_leader else f"FOLLOWER-{position}"
                    self.log(f"✅ Car {car_id} platoon setup confirmed: {role_str} (Leader ID: {leader_id})", 'SUCCESS')
                else:
                    self.platoon_indicators[car_id].config(
                        text="🚗 Setup...",
                        fg='#ff9800'
                    )
                    self.log(f"⏳ Car {car_id} platoon setup in progress...", 'INFO')
            
        except Exception as e:
            self.log(f"Error processing platoon confirmation for Car {car_id}: {e}", 'ERROR')
    
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
                
                # Update GUI indicator - show peer count
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text=f"📡 V2V: ON ({connected_peers})", fg='#4caf50')
                
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
                     self.v2v_indicators[car_id].config(text=f"📡 V2V: ON ({connected_peers})", fg='#4caf50')
                
            elif status == 'failed':
                error = v2v_data.get('error', 'unknown')
                self.v2v_status[car_id] = {'status': 'failed', 'error': error}
                
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: ERROR", fg='#f44336')
                     
                self.log(f"Car {car_id}: V2V connection failed - {error}", 'ERROR')
                
            elif status == 'disconnected':
                self.v2v_status[car_id] = {'status': 'disconnected', 'peers': 0}
                
                if hasattr(self, 'v2v_indicators') and car_id in self.v2v_indicators:
                     self.v2v_indicators[car_id].config(text="📡 V2V: OFF", fg='#888888')
                     
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
        
        # Stop manual mode if active
        if hasattr(self, 'manual_mode_active'):
            for car_id in list(self.manual_mode_active.keys()):
                if self.manual_mode_active.get(car_id, False):
                    self.disable_manual_mode(car_id)
        
        # Log shutdown
        self.log("🛑 Shutting down Ground Station...", 'INFO')
        
        # Close controller
        self.controller.close()
        
        # Destroy window
        self.root.destroy()
    
    # ===== MANUAL MODE CONTROL METHODS =====
    
    def _update_control_type(self, car_id: int, control_type: str):
        """Update the control type for a specific car"""
        self.manual_control_types[car_id] = control_type
        self.log(f"Car {car_id}: Manual control type set to {control_type.upper()}", 'INFO')
    
    def toggle_manual_mode_with_feedback(self, car_id: int):
        """Toggle manual mode on/off for a specific car"""
        if not hasattr(self, 'manual_mode_active'):
            self.manual_mode_active = {}
        
        is_active = self.manual_mode_active.get(car_id, False)
        
        if is_active:
            # Disable manual mode
            self.disable_manual_mode(car_id)
        else:
            # Enable manual mode
            self.enable_manual_mode(car_id)
    
    def enable_manual_mode(self, car_id: int):
        """Enable manual mode for a car"""
        try:
            if not hasattr(self, 'manual_mode_active'):
                self.manual_mode_active = {}
            
            # Get selected control type
            control_type = self.manual_control_types.get(car_id, 'keyboard')
            
            # Send enable manual mode command with control type
            success = self.controller.enable_manual_mode(car_id, control_type=control_type)
            
            if success:
                self.manual_mode_active[car_id] = True
                
                # Update button appearance
                if hasattr(self, 'manual_mode_buttons') and car_id in self.manual_mode_buttons:
                    self.manual_mode_buttons[car_id].config(
                        text="🎮 Manual: ON",
                        bg='#4caf50'  # Green when active
                    )
                
                # Setup input controls based on control type
                if not hasattr(self, 'manual_control_setup'):
                    if control_type == 'keyboard':
                        self.setup_keyboard_controls()
                    elif control_type == 'wheel':
                        self.setup_wheel_controls()
                    self.manual_control_setup = True
                    self.active_manual_car = car_id
                    # Start the control loop now that manual mode is active
                    self.manual_control_loop()
                else:
                    self.active_manual_car = car_id
                
                control_msg = "Use WASD keys" if control_type == 'keyboard' else "Use steering wheel"
                self.log(f"Car {car_id}: Manual mode ENABLED ({control_type.upper()}) - {control_msg}", 'SUCCESS')
                self.commands_sent_gui += 1
            else:
                self.log(f"Car {car_id}: Failed to enable manual mode", 'ERROR')
                self.commands_failed_gui += 1
                
        except Exception as e:
            self.log(f"Car {car_id}: Error enabling manual mode - {e}", 'ERROR')
            self.commands_failed_gui += 1
    
    def disable_manual_mode(self, car_id: int):
        """Disable manual mode for a car"""
        try:
            if not hasattr(self, 'manual_mode_active'):
                self.manual_mode_active = {}
            
            # Send disable manual mode command
            success = self.controller.disable_manual_mode(car_id)
            
            if success:
                self.manual_mode_active[car_id] = False
                
                # Update button appearance
                if hasattr(self, 'manual_mode_buttons') and car_id in self.manual_mode_buttons:
                    self.manual_mode_buttons[car_id].config(
                        text="🎮 Manual Mode",
                        bg='#9c27b0'  # Purple when inactive
                    )
                
                # Stop sending commands for this car
                if hasattr(self, 'active_manual_car') and self.active_manual_car == car_id:
                    self.active_manual_car = None
                    # Note: control loop continues running but won't send commands when active_manual_car is None
                
                self.log(f"Car {car_id}: Manual mode DISABLED", 'INFO')
                self.commands_sent_gui += 1
            else:
                self.log(f"Car {car_id}: Failed to disable manual mode", 'ERROR')
                self.commands_failed_gui += 1
                
        except Exception as e:
            self.log(f"Car {car_id}: Error disabling manual mode - {e}", 'ERROR')
            self.commands_failed_gui += 1
    
    def setup_keyboard_controls(self):
        """Setup keyboard event handlers for manual control"""
        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.keys_pressed = set()
        
        # Gradual steering parameters
        self.max_steering = 0.5          # Maximum steering angle
        self.steering_increment = 0.015  # Steering increase per update (at 50Hz)
        self.steering_decay = 0.95       # Decay rate when no input (smooth return to center)
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
        # Note: manual_control_loop will be started when manual mode is enabled
        
        self.log("Keyboard controls: W=Forward, S=Backward, A=Left, D=Right, Space=Stop", 'INFO')
        self.log("Steering: Hold A/D to gradually increase steering angle", 'INFO')
    
    def setup_wheel_controls(self):
        """Setup steering wheel support using pygame"""
        if not PYGAME_AVAILABLE:
            self.log("ERROR: pygame not available - cannot use steering wheel", 'ERROR')
            return False
        
        try:
            # Initialize pygame
            pygame.init()
            pygame.joystick.init()
            
            # Check for connected joysticks/wheels
            if pygame.joystick.get_count() == 0:
                self.log("ERROR: No steering wheel or gamepad detected!", 'ERROR')
                self.log("Please connect your steering wheel and try again.", 'WARNING')
                return False
            
            # Initialize the first joystick (steering wheel)
            self.wheel = pygame.joystick.Joystick(0)
            self.wheel.init()
            
            self.log(f"Steering wheel connected: {self.wheel.get_name()}", 'SUCCESS')
            self.log(f"Axes: {self.wheel.get_numaxes()}, Buttons: {self.wheel.get_numbuttons()}", 'INFO')
            
            # Steering wheel configuration (from Vehicle_wheel.py)
            self.STEERING_AXIS = 0         # Axis for steering wheel rotation
            self.ACCELERATOR_AXIS = 5      # Axis for accelerator pedal
            self.BRAKE_AXIS = 4            # Axis for brake pedal
            self.STEERING_SCALE = 0.5      # Maximum steering angle
            self.THROTTLE_SCALE = 0.3      # Maximum throttle
            self.DEADZONE = 0.05           # Deadzone for steering and pedals
            
            # Note: manual_control_loop will be started when manual mode is enabled
            
            self.log("Steering wheel controls: Rotate for steering, Pedals for throttle/brake", 'INFO')
            return True
            
        except Exception as e:
            self.log(f"ERROR: Failed to initialize steering wheel - {e}", 'ERROR')
            return False
    
    def on_key_press(self, event):
        """Handle key press events"""
        key = event.keysym.lower()
        self.keys_pressed.add(key)
    
    def on_key_release(self, event):
        """Handle key release events"""
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
    
    def manual_control_loop(self):
        """Continuous loop to send manual control commands based on keyboard or wheel input"""
        if not self.running:
            return
        
        # Only send commands if manual mode is active for at least one car
        if hasattr(self, 'active_manual_car') and self.active_manual_car is not None:
            car_id = self.active_manual_car
            control_type = self.manual_control_types.get(car_id, 'keyboard')
            
            throttle = 0.0
            steering = 0.0
            
            if control_type == 'keyboard':
                # Keyboard control (WASD) with gradual steering
                if 'w' in self.keys_pressed:
                    throttle = 0.15  # Forward
                if 's' in self.keys_pressed:
                    throttle = -0.15  # Backward
                
                # Gradual steering increase
                if 'a' in self.keys_pressed:
                    # Increase steering to the left (positive direction)
                    self.current_steering = min(self.current_steering + self.steering_increment, self.max_steering)
                elif 'd' in self.keys_pressed:
                    # Increase steering to the right (negative direction)
                    self.current_steering = max(self.current_steering - self.steering_increment, -self.max_steering)
                else:
                    # No steering input - gradually return to center
                    self.current_steering *= self.steering_decay
                    if abs(self.current_steering) < 0.01:
                        self.current_steering = 0.0
                
                steering = self.current_steering
                
                # Emergency stop overrides everything
                if 'space' in self.keys_pressed:
                    throttle = 0.0
                    steering = 0.0
                    self.current_steering = 0.0
            
            elif control_type == 'wheel' and hasattr(self, 'wheel'):
                # Steering wheel control (pygame)
                try:
                    # Process pygame events (required for pygame to work)
                    pygame.event.pump()
                    
                    # Read steering from wheel rotation
                    if self.wheel.get_numaxes() > self.STEERING_AXIS:
                        steering_input = self.wheel.get_axis(self.STEERING_AXIS)
                        # Apply deadzone
                        if abs(steering_input) < self.DEADZONE:
                            steering_input = 0
                        steering = steering_input * self.STEERING_SCALE
                    
                    # Read accelerator pedal
                    if self.wheel.get_numaxes() > self.ACCELERATOR_AXIS:
                        accelerator_input = self.wheel.get_axis(self.ACCELERATOR_AXIS)
                        # Normalize to 0 to 1 range (pedals report -1 to 1)
                        accelerator_value = (accelerator_input + 1) / 2
                        if accelerator_value < self.DEADZONE:
                            accelerator_value = 0
                    else:
                        accelerator_value = 0
                    
                    # Read brake pedal
                    if self.wheel.get_numaxes() > self.BRAKE_AXIS:
                        brake_input = self.wheel.get_axis(self.BRAKE_AXIS)
                        # Normalize to 0 to 1 range
                        brake_value = (brake_input + 1) / 2
                        if brake_value < self.DEADZONE:
                            brake_value = 0
                    else:
                        brake_value = 0
                    
                    # Calculate throttle (forward from accelerator, backward from brake)
                    throttle = (accelerator_value - brake_value) * self.THROTTLE_SCALE
                    
                    # Emergency stop with Button 0
                    if self.wheel.get_numbuttons() > 0 and self.wheel.get_button(0):
                        throttle = 0.0
                        steering = 0.0
                    
                except Exception as e:
                    # Fallback to stopped if wheel fails
                    throttle = 0.0
                    steering = 0.0
            
            # Send manual control command
            if hasattr(self, 'manual_mode_active') and self.manual_mode_active.get(car_id, False):
                self.controller.send_manual_control(car_id, throttle, steering)
        
        # Schedule next update (50 Hz for smooth control)
        self.root.after(20, self.manual_control_loop)


# Configuration
HOST_IP = '0.0.0.0'  # Listen on all network interfaces
BASE_PORT = 5000
NUM_CARS = 5


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