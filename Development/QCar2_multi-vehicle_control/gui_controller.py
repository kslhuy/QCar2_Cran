"""
GUI Controller for Multiple Physical QCars
A tkinter-based graphical interface to control multiple QCars from your Host PC.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
from datetime import datetime
from remote_controller import QCarRemoteController

class QCarGUIController:
    """Graphical User Interface for controlling multiple QCars"""
    
    def __init__(self, root, num_cars=2, host_ip='127.0.0.1', base_port=5000):
        self.root = root
        self.num_cars = num_cars
        self.max_cars = 10  # Maximum number of cars supported
        self.controller = QCarRemoteController(host_ip, base_port)
        self.car_panels = {}
        self.car_expanded = {}  # Track which panels are expanded
        
        # Setup window
        self.root.title("QCar Fleet Controller")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # Style configuration
        self.setup_styles()
        
        # Start the remote controller
        self.controller.start_server(self.num_cars)
        
        # Build GUI
        self.build_gui()
        
        # Initialize car panels
        self.update_car_panels()
        
        # Start update loop
        self.running = True
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_styles(self):
        """Configure ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Colors
        bg_dark = '#2b2b2b'
        bg_medium = '#3c3c3c'
        bg_light = '#4d4d4d'
        fg_color = '#ffffff'
        accent_green = '#4caf50'
        accent_red = '#f44336'
        accent_blue = '#2196f3'
        accent_yellow = '#ffc107'
        
        # Configure styles
        style.configure('Title.TLabel', 
                       background=bg_dark, 
                       foreground=fg_color, 
                       font=('Arial', 20, 'bold'))
        
        style.configure('Subtitle.TLabel',
                       background=bg_medium,
                       foreground=fg_color,
                       font=('Arial', 12, 'bold'))
        
        style.configure('Info.TLabel',
                       background=bg_medium,
                       foreground=fg_color,
                       font=('Arial', 10))
        
        style.configure('Status.TLabel',
                       background=bg_light,
                       foreground=fg_color,
                       font=('Arial', 9))
        
        style.configure('CarFrame.TFrame',
                       background=bg_medium,
                       relief='raised',
                       borderwidth=2)
        
        style.configure('Start.TButton',
                       background=accent_green,
                       foreground='white',
                       font=('Arial', 10, 'bold'))
        
        style.configure('Stop.TButton',
                       background=accent_red,
                       foreground='white',
                       font=('Arial', 10, 'bold'))
        
        style.configure('Command.TButton',
                       background=accent_blue,
                       foreground='white',
                       font=('Arial', 9))
    
    def build_gui(self):
        """Build the main GUI layout"""
        
        # Header
        header_frame = tk.Frame(self.root, bg='#1a1a1a', height=60)
        header_frame.pack(fill='x', padx=0, pady=0)
        header_frame.pack_propagate(False)
        
        title = ttk.Label(header_frame, 
                         text="🚗 QCar Fleet Controller", 
                         style='Title.TLabel')
        title.pack(pady=15)
        
        # Main content area
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Left panel - Car controls with scrollbar
        left_panel = tk.Frame(main_frame, bg='#2b2b2b')
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Car count control panel
        self.create_car_count_panel(left_panel)
        
        # Create canvas and scrollbar for car panels
        canvas_frame = tk.Frame(left_panel, bg='#2b2b2b')
        canvas_frame.pack(fill='both', expand=True, pady=(10, 0))
        
        self.car_canvas = tk.Canvas(canvas_frame, bg='#2b2b2b', highlightthickness=0)
        scrollbar = tk.Scrollbar(canvas_frame, orient='vertical', command=self.car_canvas.yview)
        self.scrollable_frame = tk.Frame(self.car_canvas, bg='#2b2b2b')
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.car_canvas.configure(scrollregion=self.car_canvas.bbox("all"))
        )
        
        self.car_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.car_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.car_canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Mouse wheel scrolling
        self.car_canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        
        # Fleet controls
        fleet_frame = self.create_fleet_controls(left_panel)
        fleet_frame.pack(fill='x', pady=(10, 5))
        
        # Fleet platoon controls
        fleet_platoon_frame = self.create_fleet_platoon_controls(left_panel)
        fleet_platoon_frame.pack(fill='x', pady=(5, 10))
        
        # Right panel - Log and status
        right_panel = tk.Frame(main_frame, bg='#2b2b2b', width=400)
        right_panel.pack(side='right', fill='both', padx=(5, 0))
        right_panel.pack_propagate(False)
        
        # Connection status
        status_frame = self.create_status_panel(right_panel)
        status_frame.pack(fill='x', pady=(0, 10))
        
        # Log area
        log_frame = self.create_log_panel(right_panel)
        log_frame.pack(fill='both', expand=True)
    
    def _on_mousewheel(self, event):
        """Handle mouse wheel scrolling"""
        self.car_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    def create_car_count_panel(self, parent):
        """Create panel to control number of cars"""
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        frame.pack(fill='x', pady=(0, 5))
        
        content = tk.Frame(frame, bg='#3c3c3c')
        content.pack(fill='x', padx=10, pady=8)
        
        tk.Label(content,
                text="Number of Cars:",
                bg='#3c3c3c',
                fg='white',
                font=('Arial', 10, 'bold')).pack(side='left', padx=(0, 10))
        
        # Car count spinbox
        self.car_count_var = tk.StringVar(value=str(self.num_cars))
        spinbox = tk.Spinbox(content,
                            from_=1,
                            to=self.max_cars,
                            textvariable=self.car_count_var,
                            width=5,
                            bg='#4d4d4d',
                            fg='white',
                            font=('Arial', 10),
                            buttonbackground='#5d5d5d',
                            relief='flat')
        spinbox.pack(side='left', padx=(0, 10))
        
        # Apply button
        apply_btn = tk.Button(content,
                             text="Apply",
                             bg='#2196f3',
                             fg='white',
                             font=('Arial', 9, 'bold'),
                             command=self.apply_car_count,
                             cursor='hand2',
                             relief='flat',
                             padx=15,
                             pady=3)
        apply_btn.pack(side='left')
        
        # Info label
        self.car_count_info = tk.Label(content,
                                       text=f"Active: {self.num_cars}",
                                       bg='#3c3c3c',
                                       fg='#4caf50',
                                       font=('Arial', 9))
        self.car_count_info.pack(side='left', padx=(15, 0))
    
    def apply_car_count(self):
        """Apply the new car count"""
        try:
            new_count = int(self.car_count_var.get())
            if 1 <= new_count <= self.max_cars:
                self.num_cars = new_count
                self.car_count_info.config(text=f"Active: {self.num_cars}")
                self.update_car_panels()
                self.log(f"Number of cars set to {self.num_cars}", 'SUCCESS')
            else:
                self.log(f"Number of cars must be between 1 and {self.max_cars}", 'WARNING')
        except ValueError:
            self.log("Invalid car count value", 'ERROR')
    
    def update_car_panels(self):
        """Update car panels based on num_cars"""
        # Remove all existing panels
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()
        
        self.car_panels.clear()
        
        # Create new panels with proper spacing
        for i in range(self.num_cars):
            if i not in self.car_expanded:
                self.car_expanded[i] = True  # Default to expanded
            panel = self.create_car_panel(self.scrollable_frame, i)
            # Add more spacing between panels and ensure minimum size
            panel.pack(fill='x', pady=8, padx=5)
            panel.pack_propagate(True)  # Allow frame to size based on content
            self.car_panels[i] = panel
    
    def create_car_panel(self, parent, car_id):
        """Create control panel for a single car"""
        # Main frame
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        
        # Header (always visible) - increased height for easier clicking
        header = tk.Frame(frame, bg='#2d2d2d', height=50)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        # Expand/Collapse button - larger and more visible
        is_expanded = self.car_expanded.get(car_id, True)
        expand_btn = tk.Label(header,
                             text="▼" if is_expanded else "▶",
                             bg='#2d2d2d',
                             fg='#4caf50',
                             font=('Arial', 16, 'bold'),
                             cursor='hand2',
                             padx=10,
                             pady=10)
        expand_btn.pack(side='left', padx=(5, 5), pady=5)
        
        title = tk.Label(header, 
                        text=f"Car {car_id}",
                        bg='#2d2d2d',
                        fg='white',
                        font=('Arial', 14, 'bold'),
                        cursor='hand2',
                        padx=5,
                        pady=10)
        title.pack(side='left', padx=5, pady=5, fill='y')
        
        # Connection indicator - larger for visibility
        conn_indicator = tk.Label(header,
                                 text="⚫ Disconnected",
                                 bg='#2d2d2d',
                                 fg='#f44336',
                                 font=('Arial', 10, 'bold'),
                                 cursor='hand2',
                                 pady=10)
        conn_indicator.pack(side='right', padx=10, pady=5)
        
        # Store reference
        if not hasattr(self, 'conn_indicators'):
            self.conn_indicators = {}
        self.conn_indicators[car_id] = conn_indicator
        
        # Content area (collapsible)
        content = tk.Frame(frame, bg='#3c3c3c')
        
        # Bind click event to toggle - make entire header clickable
        def toggle_panel(event=None):
            self.toggle_car_panel(car_id, content, expand_btn)
        
        expand_btn.bind('<Button-1>', toggle_panel)
        header.bind('<Button-1>', toggle_panel)
        title.bind('<Button-1>', toggle_panel)
        conn_indicator.bind('<Button-1>', toggle_panel)
        
        # Visual feedback on hover
        def on_enter(e):
            header.config(bg='#3d3d3d')
            expand_btn.config(bg='#3d3d3d')
            title.config(bg='#3d3d3d')
            conn_indicator.config(bg='#3d3d3d')
        
        def on_leave(e):
            header.config(bg='#2d2d2d')
            expand_btn.config(bg='#2d2d2d')
            title.config(bg='#2d2d2d')
            conn_indicator.config(bg='#2d2d2d')
        
        header.bind('<Enter>', on_enter)
        header.bind('<Leave>', on_leave)
        expand_btn.bind('<Enter>', on_enter)
        expand_btn.bind('<Leave>', on_leave)
        title.bind('<Enter>', on_enter)
        title.bind('<Leave>', on_leave)
        conn_indicator.bind('<Enter>', on_enter)
        conn_indicator.bind('<Leave>', on_leave)
        
        # Show/hide content based on expanded state
        if is_expanded:
            content.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Store reference to content frame
        frame.content_frame = content
        frame.expand_btn = expand_btn
        
        # Telemetry display
        telemetry_frame = tk.LabelFrame(content,
                                       text="Telemetry",
                                       bg='#3c3c3c',
                                       fg='white',
                                       font=('Arial', 10, 'bold'))
        telemetry_frame.pack(fill='x', pady=(0, 10))
        
        # Telemetry labels
        telemetry_grid = tk.Frame(telemetry_frame, bg='#3c3c3c')
        telemetry_grid.pack(fill='x', padx=5, pady=5)
        
        labels = {}
        row = 0
        for key, label_text in [('position', 'Position:'), 
                                ('velocity', 'Velocity:'),
                                ('heading', 'Heading:'),
                                ('throttle', 'Throttle:')]:
            tk.Label(telemetry_grid, 
                    text=label_text,
                    bg='#3c3c3c',
                    fg='#aaaaaa',
                    font=('Arial', 9),
                    anchor='w').grid(row=row, column=0, sticky='w', padx=5, pady=2)
            
            value_label = tk.Label(telemetry_grid,
                                  text='N/A',
                                  bg='#3c3c3c',
                                  fg='white',
                                  font=('Arial', 9, 'bold'),
                                  anchor='w')
            value_label.grid(row=row, column=1, sticky='w', padx=5, pady=2)
            labels[key] = value_label
            row += 1
        
        if not hasattr(self, 'telemetry_labels'):
            self.telemetry_labels = {}
        self.telemetry_labels[car_id] = labels
        
        # Control buttons
        button_frame = tk.Frame(content, bg='#3c3c3c')
        button_frame.pack(fill='x', pady=(0, 10))
        
        start_btn = tk.Button(button_frame,
                             text="▶ START",
                             bg='#4caf50',
                             fg='white',
                             font=('Arial', 10, 'bold'),
                             command=lambda: self.start_car(car_id),
                             cursor='hand2',
                             relief='flat',
                             padx=20,
                             pady=5)
        start_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        stop_btn = tk.Button(button_frame,
                            text="⬛ STOP",
                            bg='#f44336',
                            fg='white',
                            font=('Arial', 10, 'bold'),
                            command=lambda: self.stop_car(car_id),
                            cursor='hand2',
                            relief='flat',
                            padx=20,
                            pady=5)
        stop_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        # Velocity control
        vel_frame = tk.LabelFrame(content,
                                 text="Velocity Control",
                                 bg='#3c3c3c',
                                 fg='white',
                                 font=('Arial', 10, 'bold'))
        vel_frame.pack(fill='x', pady=(0, 10))
        
        vel_content = tk.Frame(vel_frame, bg='#3c3c3c')
        vel_content.pack(fill='x', padx=5, pady=5)
        
        tk.Label(vel_content,
                text="Target (m/s):",
                bg='#3c3c3c',
                fg='#aaaaaa',
                font=('Arial', 9)).pack(side='left', padx=(0, 5))
        
        vel_entry = tk.Entry(vel_content,
                            width=8,
                            bg='#4d4d4d',
                            fg='white',
                            font=('Arial', 10),
                            insertbackground='white')
        vel_entry.insert(0, "0.9")
        vel_entry.pack(side='left', padx=(0, 5))
        
        vel_btn = tk.Button(vel_content,
                           text="Set",
                           bg='#2196f3',
                           fg='white',
                           font=('Arial', 9),
                           command=lambda: self.set_velocity(car_id, vel_entry.get()),
                           cursor='hand2',
                           relief='flat',
                           padx=15,
                           pady=2)
        vel_btn.pack(side='left')
        
        # Path control
        path_frame = tk.LabelFrame(content,
                                  text="Path Control",
                                  bg='#3c3c3c',
                                  fg='white',
                                  font=('Arial', 10, 'bold'))
        path_frame.pack(fill='x', pady=(0, 10))
        
        path_content = tk.Frame(path_frame, bg='#3c3c3c')
        path_content.pack(fill='x', padx=5, pady=5)
        
        tk.Label(path_content,
                text="Nodes:",
                bg='#3c3c3c',
                fg='#aaaaaa',
                font=('Arial', 9)).pack(side='left', padx=(0, 5))
        
        path_entry = tk.Entry(path_content,
                             width=15,
                             bg='#4d4d4d',
                             fg='white',
                             font=('Arial', 10),
                             insertbackground='white')
        path_entry.insert(0, "10 4 20 10" if car_id == 0 else "4 13 9 4")
        path_entry.pack(side='left', padx=(0, 5))
        
        path_btn = tk.Button(path_content,
                            text="Set Path",
                            bg='#2196f3',
                            fg='white',
                            font=('Arial', 9),
                            command=lambda: self.set_path(car_id, path_entry.get()),
                            cursor='hand2',
                            relief='flat',
                            padx=15,
                            pady=2)
        path_btn.pack(side='left')
        
        # Platoon control
        platoon_frame = tk.LabelFrame(content,
                                     text="Platoon Control",
                                     bg='#3c3c3c',
                                     fg='white',
                                     font=('Arial', 10, 'bold'))
        platoon_frame.pack(fill='x')
        
        platoon_content = tk.Frame(platoon_frame, bg='#3c3c3c')
        platoon_content.pack(fill='x', padx=5, pady=5)
        
        # Role selection
        role_frame = tk.Frame(platoon_content, bg='#3c3c3c')
        role_frame.pack(fill='x', pady=(0, 5))
        
        tk.Label(role_frame,
                text="Role:",
                bg='#3c3c3c',
                fg='#aaaaaa',
                font=('Arial', 9)).pack(side='left', padx=(0, 5))
        
        role_var = tk.StringVar(value="follower")
        leader_radio = tk.Radiobutton(role_frame,
                                     text="Leader",
                                     variable=role_var,
                                     value="leader",
                                     bg='#3c3c3c',
                                     fg='white',
                                     selectcolor='#4d4d4d',
                                     activebackground='#3c3c3c',
                                     activeforeground='white',
                                     font=('Arial', 9))
        leader_radio.pack(side='left', padx=5)
        
        follower_radio = tk.Radiobutton(role_frame,
                                       text="Follower",
                                       variable=role_var,
                                       value="follower",
                                       bg='#3c3c3c',
                                       fg='white',
                                       selectcolor='#4d4d4d',
                                       activebackground='#3c3c3c',
                                       activeforeground='white',
                                       font=('Arial', 9))
        follower_radio.pack(side='left', padx=5)
        
        # Platoon buttons
        platoon_btn_frame = tk.Frame(platoon_content, bg='#3c3c3c')
        platoon_btn_frame.pack(fill='x', pady=(5, 0))
        
        enable_platoon_btn = tk.Button(platoon_btn_frame,
                                      text="Enable Platoon",
                                      bg='#ff9800',
                                      fg='white',
                                      font=('Arial', 9, 'bold'),
                                      command=lambda: self.enable_platoon(car_id, role_var.get()),
                                      cursor='hand2',
                                      relief='flat',
                                      padx=10,
                                      pady=3)
        enable_platoon_btn.pack(side='left', expand=True, fill='x', padx=(0, 3))
        
        disable_platoon_btn = tk.Button(platoon_btn_frame,
                                       text="Disable",
                                       bg='#9e9e9e',
                                       fg='white',
                                       font=('Arial', 9, 'bold'),
                                       command=lambda: self.disable_platoon(car_id),
                                       cursor='hand2',
                                       relief='flat',
                                       padx=10,
                                       pady=3)
        disable_platoon_btn.pack(side='left', expand=True, fill='x', padx=(3, 0))
        
        # Platoon status label
        platoon_status = tk.Label(platoon_content,
                                 text="⚫ Platoon: Inactive",
                                 bg='#3c3c3c',
                                 fg='#9e9e9e',
                                 font=('Arial', 8))
        platoon_status.pack(fill='x', pady=(5, 0))
        
        # Store platoon status reference
        if not hasattr(self, 'platoon_status_labels'):
            self.platoon_status_labels = {}
        self.platoon_status_labels[car_id] = platoon_status
        
        # Store references for updates
        frame.car_id = car_id
        
        return frame
    
    def toggle_car_panel(self, car_id, content_frame, expand_btn):
        """Toggle expand/collapse of car panel"""
        is_expanded = self.car_expanded.get(car_id, True)
        
        if is_expanded:
            # Collapse
            content_frame.pack_forget()
            expand_btn.config(text="▶", fg='#ff9800')
            self.car_expanded[car_id] = False
        else:
            # Expand
            content_frame.pack(fill='both', expand=True, padx=10, pady=10)
            expand_btn.config(text="▼", fg='#4caf50')
            self.car_expanded[car_id] = True
    
    def create_fleet_controls(self, parent):
        """Create fleet-wide control panel"""
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        
        header = tk.Frame(frame, bg='#2d2d2d', height=35)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        title = tk.Label(header,
                        text="Fleet Controls",
                        bg='#2d2d2d',
                        fg='white',
                        font=('Arial', 12, 'bold'))
        title.pack(side='left', padx=10, pady=5)
        
        content = tk.Frame(frame, bg='#3c3c3c')
        content.pack(fill='x', padx=10, pady=10)
        
        # Fleet buttons
        btn_frame = tk.Frame(content, bg='#3c3c3c')
        btn_frame.pack(fill='x')
        
        start_all_btn = tk.Button(btn_frame,
                                 text="▶ START ALL",
                                 bg='#4caf50',
                                 fg='white',
                                 font=('Arial', 11, 'bold'),
                                 command=self.start_all_cars,
                                 cursor='hand2',
                                 relief='flat',
                                 padx=20,
                                 pady=8)
        start_all_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        stop_all_btn = tk.Button(btn_frame,
                                text="⬛ STOP ALL",
                                bg='#f44336',
                                fg='white',
                                font=('Arial', 11, 'bold'),
                                command=self.stop_all_cars,
                                cursor='hand2',
                                relief='flat',
                                padx=20,
                                pady=8)
        stop_all_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        # Emergency stop (larger, more prominent)
        emergency_btn = tk.Button(content,
                                 text="🚨 EMERGENCY STOP 🚨",
                                 bg='#d32f2f',
                                 fg='white',
                                 font=('Arial', 12, 'bold'),
                                 command=self.emergency_stop,
                                 cursor='hand2',
                                 relief='raised',
                                 bd=3,
                                 padx=20,
                                 pady=10)
        emergency_btn.pack(fill='x', pady=(10, 0))
        
        return frame
    
    def create_fleet_platoon_controls(self, parent):
        """Create fleet platoon control panel"""
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        
        header = tk.Frame(frame, bg='#2d2d2d', height=35)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        title = tk.Label(header,
                        text="🚗 Fleet Platoon Setup",
                        bg='#2d2d2d',
                        fg='white',
                        font=('Arial', 12, 'bold'))
        title.pack(side='left', padx=10, pady=5)
        
        content = tk.Frame(frame, bg='#3c3c3c')
        content.pack(fill='x', padx=10, pady=10)
        
        # Leader selection
        leader_frame = tk.Frame(content, bg='#3c3c3c')
        leader_frame.pack(fill='x', pady=(0, 10))
        
        tk.Label(leader_frame,
                text="Select Leader:",
                bg='#3c3c3c',
                fg='white',
                font=('Arial', 10, 'bold')).pack(side='left', padx=(0, 10))
        
        self.leader_var = tk.StringVar(value="0")
        leader_dropdown = ttk.Combobox(leader_frame,
                                      textvariable=self.leader_var,
                                      values=[str(i) for i in range(self.max_cars)],
                                      state='readonly',
                                      width=10,
                                      font=('Arial', 10))
        leader_dropdown.pack(side='left')
        
        # Fleet platoon buttons
        btn_frame = tk.Frame(content, bg='#3c3c3c')
        btn_frame.pack(fill='x')
        
        enable_fleet_btn = tk.Button(btn_frame,
                                    text="Enable Fleet Platoon",
                                    bg='#ff9800',
                                    fg='white',
                                    font=('Arial', 11, 'bold'),
                                    command=self.enable_fleet_platoon,
                                    cursor='hand2',
                                    relief='flat',
                                    padx=20,
                                    pady=8)
        enable_fleet_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        
        disable_fleet_btn = tk.Button(btn_frame,
                                     text="Disable All",
                                     bg='#9e9e9e',
                                     fg='white',
                                     font=('Arial', 11, 'bold'),
                                     command=self.disable_fleet_platoon,
                                     cursor='hand2',
                                     relief='flat',
                                     padx=20,
                                     pady=8)
        disable_fleet_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        # Info label
        info_label = tk.Label(content,
                             text="⚡ One-click setup: Select leader, all others become followers",
                             bg='#3c3c3c',
                             fg='#ffc107',
                             font=('Arial', 9, 'italic'),
                             wraplength=350)
        info_label.pack(fill='x', pady=(10, 0))
        
        return frame
    
    def create_status_panel(self, parent):
        """Create connection status panel"""
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        
        header = tk.Frame(frame, bg='#2d2d2d', height=35)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        title = tk.Label(header,
                        text="Connection Status",
                        bg='#2d2d2d',
                        fg='white',
                        font=('Arial', 11, 'bold'))
        title.pack(side='left', padx=10, pady=5)
        
        content = tk.Frame(frame, bg='#3c3c3c')
        content.pack(fill='x', padx=10, pady=10)
        
        self.status_text = tk.Text(content,
                                   height=4,
                                   bg='#4d4d4d',
                                   fg='white',
                                   font=('Consolas', 9),
                                   relief='flat',
                                   state='disabled')
        self.status_text.pack(fill='x')
        
        return frame
    
    def create_log_panel(self, parent):
        """Create log panel"""
        frame = tk.Frame(parent, bg='#3c3c3c', relief='raised', bd=2)
        
        header = tk.Frame(frame, bg='#2d2d2d', height=35)
        header.pack(fill='x')
        header.pack_propagate(False)
        
        title = tk.Label(header,
                        text="Activity Log",
                        bg='#2d2d2d',
                        fg='white',
                        font=('Arial', 11, 'bold'))
        title.pack(side='left', padx=10, pady=5)
        
        clear_btn = tk.Button(header,
                             text="Clear",
                             bg='#5d5d5d',
                             fg='white',
                             font=('Arial', 8),
                             command=self.clear_log,
                             cursor='hand2',
                             relief='flat',
                             padx=10,
                             pady=2)
        clear_btn.pack(side='right', padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(frame,
                                                  bg='#1a1a1a',
                                                  fg='#00ff00',
                                                  font=('Consolas', 9),
                                                  relief='flat',
                                                  state='disabled')
        self.log_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        return frame
    
    def log(self, message, level='INFO'):
        """Add message to log"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        # Color based on level
        if level == 'ERROR':
            color = '#ff5555'
        elif level == 'WARNING':
            color = '#ffaa00'
        elif level == 'SUCCESS':
            color = '#00ff00'
        else:
            color = '#00ffff'
        
        self.log_text.configure(state='normal')
        self.log_text.insert('end', f"[{timestamp}] ", 'timestamp')
        self.log_text.insert('end', f"{level}: ", level)
        self.log_text.insert('end', f"{message}\n", 'message')
        
        # Configure tags
        self.log_text.tag_config('timestamp', foreground='#888888')
        self.log_text.tag_config(level, foreground=color, font=('Consolas', 9, 'bold'))
        self.log_text.tag_config('message', foreground='#ffffff')
        
        self.log_text.see('end')
        self.log_text.configure(state='disabled')
    
    def clear_log(self):
        """Clear the log"""
        self.log_text.configure(state='normal')
        self.log_text.delete('1.0', 'end')
        self.log_text.configure(state='disabled')
    
    def start_car(self, car_id):
        """Start a specific car"""
        if self.controller.start_car(car_id):
            self.log(f"Started Car {car_id}", 'SUCCESS')
        else:
            self.log(f"Failed to start Car {car_id} - not connected", 'ERROR')
    
    def stop_car(self, car_id):
        """Stop a specific car"""
        if self.controller.stop_car(car_id):
            self.log(f"Stopped Car {car_id}", 'SUCCESS')
        else:
            self.log(f"Failed to stop Car {car_id} - not connected", 'ERROR')
    
    def set_velocity(self, car_id, velocity_str):
        """Set velocity for a car"""
        try:
            velocity = float(velocity_str)
            if 0 <= velocity <= 2.0:
                if self.controller.set_velocity(car_id, velocity):
                    self.log(f"Set Car {car_id} velocity to {velocity} m/s", 'SUCCESS')
                else:
                    self.log(f"Failed to set velocity for Car {car_id}", 'ERROR')
            else:
                self.log(f"Velocity must be between 0 and 2.0 m/s", 'WARNING')
        except ValueError:
            self.log(f"Invalid velocity value: {velocity_str}", 'ERROR')
    
    def set_path(self, car_id, path_str):
        """Set path for a car"""
        try:
            nodes = [int(n) for n in path_str.split()]
            if len(nodes) >= 2:
                if self.controller.set_path(car_id, nodes):
                    self.log(f"Set Car {car_id} path to {nodes}", 'SUCCESS')
                else:
                    self.log(f"Failed to set path for Car {car_id}", 'ERROR')
            else:
                self.log(f"Path must have at least 2 nodes", 'WARNING')
        except ValueError:
            self.log(f"Invalid path format: {path_str}", 'ERROR')
    
    def start_all_cars(self):
        """Start all cars"""
        for car_id in range(self.num_cars):
            self.start_car(car_id)
        self.log(f"Started all {self.num_cars} cars", 'INFO')
    
    def stop_all_cars(self):
        """Stop all cars"""
        for car_id in range(self.num_cars):
            self.stop_car(car_id)
        self.log(f"Stopped all {self.num_cars} cars", 'INFO')
    
    def emergency_stop(self):
        """Emergency stop all cars"""
        result = messagebox.askyesno("Emergency Stop",
                                     "Emergency stop ALL cars?",
                                     icon='warning')
        if result:
            self.controller.emergency_stop_all()
            self.log("EMERGENCY STOP ACTIVATED", 'ERROR')
    
    def enable_platoon(self, car_id, role):
        """Enable platoon mode for a car"""
        # Validation: Check if leader already exists
        if role == 'leader':
            # Check if another car is already a leader
            for i in range(self.num_cars):
                if i != car_id and i in self.controller.cars:
                    data = self.controller.cars[i].get('last_data')
                    if data and data.get('platoon_active') and data.get('platoon_role') == 'Leader':
                        self.log(f"❌ Car {i} is already the leader! Only one leader allowed.", 'ERROR')
                        messagebox.showerror("Platoon Error", 
                                           f"Car {i} is already the platoon leader.\nOnly one leader is allowed per platoon.")
                        return
        
        elif role == 'follower':
            # Check if any leader exists
            leader_exists = False
            leader_id = None
            for i in range(self.num_cars):
                if i != car_id and i in self.controller.cars:
                    data = self.controller.cars[i].get('last_data')
                    if data and data.get('platoon_active') and data.get('platoon_role') == 'Leader':
                        leader_exists = True
                        leader_id = i
                        break
            
            if not leader_exists:
                self.log(f"⚠️  No leader found! Follower will search for leader via YOLO.", 'WARNING')
                # Still allow, but warn user
                result = messagebox.askyesno("No Leader Detected",
                                           "No active leader detected in the system.\n\n"
                                           "The follower will search for a leader using YOLO camera.\n"
                                           "Make sure a leader vehicle is visible ahead.\n\n"
                                           "Continue anyway?")
                if not result:
                    return
        
        command = {
            'type': 'enable_platoon',
            'role': role,
            'leader_id': leader_id if role == 'follower' else None
        }
        
        if self.controller.send_command(car_id, command):
            self.log(f"🚗 Car {car_id}: Platoon enabled as {role.upper()}", 'INFO')
        else:
            self.log(f"❌ Car {car_id}: Failed to enable platoon", 'ERROR')
    
    def disable_platoon(self, car_id):
        """Disable platoon mode for a car"""
        command = {'type': 'disable_platoon'}
        
        if self.controller.send_command(car_id, command):
            self.log(f"🚗 Car {car_id}: Platoon disabled", 'INFO')
        else:
            self.log(f"❌ Car {car_id}: Failed to disable platoon", 'ERROR')
    
    def enable_fleet_platoon(self):
        """Enable platoon mode for entire fleet - one leader, rest followers"""
        if self.num_cars < 2:
            messagebox.showwarning("Platoon Error", "Need at least 2 cars for platoon mode")
            return
        
        try:
            leader_id = int(self.leader_var.get())
            
            if leader_id >= self.num_cars:
                messagebox.showerror("Invalid Leader", f"Car {leader_id} does not exist")
                return
            
            # Confirm action
            follower_ids = [i for i in range(self.num_cars) if i != leader_id]
            result = messagebox.askyesno("Enable Fleet Platoon",
                                       f"This will configure:\n\n"
                                       f"Leader: Car {leader_id}\n"
                                       f"Followers: {follower_ids}\n\n"
                                       f"Continue?")
            if not result:
                return
            
            self.log("="*50, 'INFO')
            self.log("🚗 Enabling Fleet Platoon Mode", 'INFO')
            
            # Step 1: Enable leader first
            leader_cmd = {
                'type': 'enable_platoon',
                'role': 'leader',
                'leader_id': None
            }
            
            if self.controller.send_command(leader_id, leader_cmd):
                self.log(f"✓ Car {leader_id}: Leader enabled", 'SUCCESS')
            else:
                self.log(f"✗ Car {leader_id}: Failed to enable as leader", 'ERROR')
                return
            
            # Step 2: Wait a moment for leader to initialize
            time.sleep(0.2)
            
            # Step 3: Enable followers
            for follower_id in follower_ids:
                follower_cmd = {
                    'type': 'enable_platoon',
                    'role': 'follower',
                    'leader_id': leader_id
                }
                
                if self.controller.send_command(follower_id, follower_cmd):
                    self.log(f"✓ Car {follower_id}: Follower enabled", 'SUCCESS')
                else:
                    self.log(f"✗ Car {follower_id}: Failed to enable as follower", 'ERROR')
                
                time.sleep(0.1)  # Small delay between commands
            
            self.log("🚗 Fleet Platoon Setup Complete!", 'SUCCESS')
            self.log("="*50, 'INFO')
            
        except Exception as e:
            self.log(f"Error enabling fleet platoon: {e}", 'ERROR')
    
    def disable_fleet_platoon(self):
        """Disable platoon mode for all vehicles"""
        result = messagebox.askyesno("Disable Fleet Platoon",
                                   "Disable platoon mode for ALL cars?")
        if not result:
            return
        
        self.log("Disabling platoon mode for all vehicles...", 'INFO')
        
        for car_id in range(self.num_cars):
            self.disable_platoon(car_id)
            time.sleep(0.1)
        
        self.log("Fleet platoon disabled", 'SUCCESS')
    
    def update_loop(self):
        """Background thread to update GUI"""
        while self.running:
            try:
                # Update connection indicators and telemetry
                for car_id in range(self.num_cars):
                    if car_id in self.controller.cars:
                        car_data = self.controller.cars[car_id]
                        status = car_data.get('status', 'disconnected')
                        
                        # Update connection indicator
                        if car_id in self.conn_indicators:
                            if status == 'connected':
                                self.conn_indicators[car_id].config(
                                    text="🟢 Connected",
                                    fg='#4caf50'
                                )
                            else:
                                self.conn_indicators[car_id].config(
                                    text="⚫ Disconnected",
                                    fg='#f44336'
                                )
                        
                        # Update telemetry
                        if car_id in self.telemetry_labels and car_data.get('last_data'):
                            data = car_data['last_data']
                            labels = self.telemetry_labels[car_id]
                            
                            # # Debug: print to verify data is being received
                            # if car_id == 0:  # Only print for car 0 to avoid spam
                            #     print(f"[GUI Update] Car {car_id} data: x={data.get('x', 'N/A')}, y={data.get('y', 'N/A')}, v={data.get('v', 'N/A')}")
                            
                            x = data.get('x', 0)
                            y = data.get('y', 0)
                            labels['position'].config(text=f"({x:.2f}, {y:.2f}) m")
                            
                            v = data.get('v', 0)
                            v_ref = data.get('v_ref', 0)
                            labels['velocity'].config(text=f"{v:.2f} / {v_ref:.2f} m/s")
                            
                            th = data.get('th', 0)
                            labels['heading'].config(text=f"{th:.2f} rad")
                            
                            u = data.get('u', 0)
                            labels['throttle'].config(text=f"{u:.2f}")
                        
                        # Update platoon status
                        if car_id in self.platoon_status_labels and car_data.get('last_data'):
                            data = car_data['last_data']
                            
                            platoon_active = data.get('platoon_active', False)
                            platoon_role = data.get('platoon_role', 'None')
                            
                            if platoon_active:
                                leader_detected = data.get('leader_detected', False)
                                leader_distance = data.get('leader_distance', 0)
                                formation_ready = data.get('formation_ready', False)
                                
                                if platoon_role == 'Leader':
                                    status_text = "🟢 Platoon: Leader"
                                    status_color = '#4caf50'
                                elif platoon_role == 'Follower':
                                    if formation_ready:
                                        status_text = f"🟢 Platoon: Active ({leader_distance:.1f}m)"
                                        status_color = '#4caf50'
                                    elif leader_detected:
                                        status_text = f"🟡 Platoon: Forming ({leader_distance:.1f}m)"
                                        status_color = '#ffc107'
                                    else:
                                        status_text = "🔴 Platoon: Searching"
                                        status_color = '#f44336'
                                else:
                                    status_text = f"🟢 Platoon: {platoon_role}"
                                    status_color = '#4caf50'
                            else:
                                status_text = "⚫ Platoon: Inactive"
                                status_color = '#9e9e9e'
                            
                            self.platoon_status_labels[car_id].config(
                                text=status_text,
                                fg=status_color
                            )
                
                # Update status text
                connected = sum(1 for car_id, c in self.controller.cars.items() 
                              if car_id < self.num_cars and c.get('status') == 'connected')
                status_msg = f"Connected Cars: {connected}/{self.num_cars}\n"
                status_msg += f"Active Cars: {self.num_cars}\n"
                status_msg += f"Listening on ports: {BASE_PORT}-{BASE_PORT + self.max_cars - 1}"
                
                self.status_text.configure(state='normal')
                self.status_text.delete('1.0', 'end')
                self.status_text.insert('1.0', status_msg)
                self.status_text.configure(state='disabled')
                
            except Exception as e:
                print(f"Update error: {e}")
            
            time.sleep(0.1)  # Update at 10 Hz
    
    def on_closing(self):
        """Handle window close"""
        if messagebox.askokcancel("Quit", "Stop all cars and quit?"):
            self.running = False
            self.controller.emergency_stop_all()
            time.sleep(0.5)
            self.controller.close()
            self.root.destroy()


# Configuration
# HOST_IP = '192.168.2.200'  # Specific interface (use if needed)
HOST_IP = '0.0.0.0'  # Listen on all network interfaces (RECOMMENDED)
BASE_PORT = 5000
NUM_CARS = 2


def main():
    """Main entry point"""
    root = tk.Tk()
    app = QCarGUIController(root, num_cars=NUM_CARS, host_ip=HOST_IP, base_port=BASE_PORT)
    
    # Log startup
    app.log(f"QCar Fleet Controller started", 'INFO')
    app.log(f"Listening on ports {BASE_PORT}-{BASE_PORT + NUM_CARS - 1}", 'INFO')
    app.log(f"Waiting for {NUM_CARS} cars to connect...", 'INFO')
    
    root.mainloop()


if __name__ == '__main__':
    main()
