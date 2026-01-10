"""
Status and Log Panels for QCar Fleet Controller.

This module contains widgets for displaying system status and activity logs.
"""

import tkinter as tk
from tkinter import scrolledtext
from typing import Optional
from datetime import datetime
from dataclasses import dataclass

from ..theme import Theme, DEFAULT_THEME
from ..config import LogLevel
from .base import BaseWidget, ThemedButton, ThemedLabel, ThemedLabelFrame


@dataclass
class FleetStatus:
    """Data class for fleet status information."""
    total_cars: int = 0
    connected_cars: int = 0
    commands_sent: int = 0
    commands_failed: int = 0
    success_rate: float = 100.0
    uptime_seconds: float = 0.0
    avg_telemetry_rate: float = 0.0
    host_ip: str = "0.0.0.0"
    base_port: int = 5000


class StatusPanelWidget(BaseWidget):
    """Widget for displaying system and fleet status."""
    
    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self._stats_label: Optional[tk.Label] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the status panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="📡 System Status",
            theme=self.theme
        )
        
        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill='both', expand=True, padx=15, pady=10)
        
        # Single combined stats label
        self._stats_label = tk.Label(
            content,
            text="Fleet: 0/0\nCommands: 0 sent, 0 failed\nSuccess: 100%\nServer: 0.0.0.0:5000",
            bg=c.bg_medium,
            fg=c.fg_primary,
            font=self.theme.fonts.small(),
            justify='left',
            anchor='w'
        )
        self._stats_label.pack(fill='both', expand=True, anchor='w')
    
    def update(self, status: FleetStatus) -> None:
        """Update the status display."""
        c = self.theme.colors
        
        if self._stats_label:
            rate = status.success_rate
            rate_color = c.accent_green if rate > 90 else c.accent_orange if rate > 70 else c.accent_red
            
            # Format multi-line status text
            text = (
                f"Fleet: {status.connected_cars}/{status.total_cars} connected\n"
                f"Commands: {status.commands_sent} sent, {status.commands_failed} failed\n"
                f"Success: {rate:.1f}% | Telemetry: {status.avg_telemetry_rate:.1f} Hz\n"
                f"Server: {status.host_ip}:{status.base_port} | Uptime: {status.uptime_seconds:.0f}s"
            )
            
            self._stats_label.config(text=text)


class LogPanelWidget(BaseWidget):
    """Widget for displaying activity logs."""
    
    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self._log_text: Optional[scrolledtext.ScrolledText] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the log panel."""
        c = self.theme.colors
        
        self.frame = ThemedLabelFrame(
            self.parent,
            text="📝 Activity Log",
            theme=self.theme
        )
        
        # Controls row
        controls = tk.Frame(self.frame, bg=c.bg_medium)
        controls.pack(fill='x', padx=15, pady=10)
        
        ThemedButton(
            controls,
            text="Clear Log",
            button_type='secondary',
            command=self.clear,
            padx=15,
            pady=4
        ).pack(side='right')
        
        # Log text area
        self._log_text = scrolledtext.ScrolledText(
            self.frame,
            width=50,
            height=25,
            bg=c.bg_header,
            fg=c.fg_primary,
            font=self.theme.fonts.mono(),
            insertbackground=c.fg_primary,
            selectbackground=c.bg_light
        )
        self._log_text.pack(fill='both', expand=True, padx=15, pady=(0, 15))
        self._log_text.config(state='disabled')
        
        # Configure log level tags
        self._configure_tags()
    
    def _configure_tags(self) -> None:
        """Configure text tags for log levels."""
        for level in ['INFO', 'SUCCESS', 'WARNING', 'ERROR', 'DEBUG', 'CONFIG']:
            color = self.theme.get_log_color(level)
            self._log_text.tag_config(level, foreground=color)
    
    def log(self, message: str, level: str = 'INFO') -> None:
        """Add a log message with timestamp and color coding."""
        if not self._log_text:
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {message}\n"
        
        self._log_text.config(state='normal')
        self._log_text.insert(tk.END, formatted_msg)
        
        # Apply color tag to the last line
        line_start = self._log_text.index("end-2c linestart")
        line_end = self._log_text.index("end-1c")
        self._log_text.tag_add(level, line_start, line_end)
        
        self._log_text.config(state='disabled')
        self._log_text.see(tk.END)
    
    def clear(self) -> None:
        """Clear the log."""
        if not self._log_text:
            return
        
        self._log_text.config(state='normal')
        self._log_text.delete(1.0, tk.END)
        self._log_text.config(state='disabled')
        self.log("Log cleared", 'INFO')


class HeaderWidget(BaseWidget):
    """Widget for the application header with title and statistics."""
    
    def __init__(self, parent: tk.Widget, title: str = "QCar Fleet Controller",
                 theme: Theme = None):
        self.title = title
        self._title_label: Optional[tk.Label] = None
        self._stats_label: Optional[tk.Label] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the header widget."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_darkest, height=80)
        self.frame.pack_propagate(False)
        
        content = tk.Frame(self.frame, bg=c.bg_darkest)
        content.pack(fill='both', expand=True)
        
        # Title
        self._title_label = tk.Label(
            content,
            text=f"🚗 {self.title}",
            bg=c.bg_darkest,
            fg=c.fg_primary,
            font=self.theme.fonts.title()
        )
        self._title_label.pack(pady=10)
        
        # # Statistics bar
        # self._stats_label = tk.Label(
        #     content,
        #     text="Commands: 0 sent, 0 failed | Uptime: 0s | Telemetry: 0.0 Hz",
        #     bg=c.bg_darkest,
        #     fg=c.fg_muted,
        #     font=self.theme.fonts.small()
        # )
        # self._stats_label.pack(pady=(0, 5))
    
    def update_stats(self, commands_sent: int, commands_failed: int,
                     uptime: float, telemetry_rate: float) -> None:
        """Update the statistics display."""
        if self._stats_label:
            self._stats_label.config(
                text=f"Commands: {commands_sent} sent, {commands_failed} failed | "
                     f"Uptime: {uptime:.0f}s | Telemetry: {telemetry_rate:.1f} Hz"
            )
