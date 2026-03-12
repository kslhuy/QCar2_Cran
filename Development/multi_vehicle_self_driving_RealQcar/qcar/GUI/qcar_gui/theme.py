"""
Theme and styles module for QCar Fleet Controller.

This module contains all color schemes, fonts, and ttk style configurations.
Separating styles from logic allows for easy theming and consistent UI.
"""

from dataclasses import dataclass
from typing import Dict, Any
import tkinter as tk
from tkinter import ttk


@dataclass(frozen=True)
class ColorScheme:
    """Color scheme definition for the application."""
    # Background colors
    bg_darkest: str = '#0d0d0d'
    bg_dark: str = '#1e1e1e'
    bg_medium: str = '#2d2d2d'
    bg_light: str = '#3d3d3d'
    bg_panel: str = '#252525'
    bg_header: str = '#1a1a1a'
    
    # Foreground colors
    fg_primary: str = '#ffffff'
    fg_secondary: str = '#cccccc'
    fg_muted: str = '#888888'
    
    # Accent colors
    accent_green: str = '#4caf50'
    accent_red: str = '#f44336'
    accent_blue: str = '#2196f3'
    accent_orange: str = '#ff9800'
    accent_purple: str = '#9c27b0'
    accent_teal: str = '#009688'
    accent_brown: str = '#795548'
    accent_gray: str = '#607d8b'
    
    # Status colors
    status_connected: str = '#4caf50'
    status_disconnected: str = '#f44336'
    status_pending: str = '#ff9800'
    status_inactive: str = '#888888'
    
    # Platoon indicator colors
    platoon_leader: str = '#ffa726'
    platoon_follower: str = '#42a5f5'
    
    # Log level colors
    log_info: str = '#ffffff'
    log_success: str = '#4caf50'
    log_warning: str = '#ff9800'
    log_error: str = '#f44336'
    log_debug: str = '#888888'


@dataclass(frozen=True)
class FontScheme:
    """Font configuration for the application."""
    family: str = 'Segoe UI'
    family_mono: str = 'Consolas'
    
    # Size definitions
    size_title: int = 16
    size_subtitle: int = 12
    size_heading: int = 10
    size_body: int = 9
    size_small: int = 8
    size_tiny: int = 8
    size_log: int = 8
    
    def title(self) -> tuple:
        return (self.family, self.size_title, 'bold')
    
    def subtitle(self) -> tuple:
        return (self.family, self.size_subtitle, 'bold')
    
    def heading(self) -> tuple:
        return (self.family, self.size_heading, 'bold')
    
    def body(self) -> tuple:
        return (self.family, self.size_body)
    
    def body_bold(self) -> tuple:
        return (self.family, self.size_body, 'bold')
    
    def small(self) -> tuple:
        return (self.family, self.size_small)
    
    def small_bold(self) -> tuple:
        return (self.family, self.size_small, 'bold')
    
    def tiny(self) -> tuple:
        return (self.family, self.size_tiny)
    
    def mono(self) -> tuple:
        return (self.family_mono, self.size_log)


class Theme:
    """Theme manager for the application."""
    
    def __init__(self, colors: ColorScheme = None, fonts: FontScheme = None):
        self.colors = colors or ColorScheme()
        self.fonts = fonts or FontScheme()
    
    def apply_to_root(self, root: tk.Tk) -> None:
        """Apply theme to the root window."""
        root.configure(bg=self.colors.bg_dark)
    
    def configure_ttk_styles(self) -> None:
        """Configure all ttk styles for the application."""
        style = ttk.Style()
        style.theme_use('clam')
        
        c = self.colors
        f = self.fonts
        
        # Label styles
        style.configure('Title.TLabel',
                        background=c.bg_dark,
                        foreground=c.fg_primary,
                        font=f.title())
        
        style.configure('Subtitle.TLabel',
                        background=c.bg_medium,
                        foreground=c.fg_primary,
                        font=f.subtitle())
        
        style.configure('Info.TLabel',
                        background=c.bg_medium,
                        foreground=c.fg_secondary,
                        font=f.body())
        
        style.configure('Status.TLabel',
                        background=c.bg_light,
                        foreground=c.fg_primary,
                        font=f.small())
        
        # Frame styles
        style.configure('CarFrame.TFrame',
                        background=c.bg_medium,
                        relief='raised',
                        borderwidth=2)
        
        style.configure('Dark.TFrame',
                        background=c.bg_dark)
        
        style.configure('Panel.TFrame',
                        background=c.bg_medium)
        
        # Button styles
        self._configure_button_style(style, 'Start.TButton', c.accent_green, f.body_bold())
        self._configure_button_style(style, 'Stop.TButton', c.accent_red, f.body_bold())
        self._configure_button_style(style, 'Command.TButton', c.accent_blue, f.small())
        self._configure_button_style(style, 'Platoon.TButton', c.accent_purple, f.small())
        self._configure_button_style(style, 'Emergency.TButton', c.accent_orange, f.body_bold())
        self._configure_button_style(style, 'Secondary.TButton', c.accent_gray, f.small())
    
    def _configure_button_style(self, style: ttk.Style, name: str, 
                                 bg_color: str, font: tuple) -> None:
        """Helper to configure a button style."""
        style.configure(name,
                        background=bg_color,
                        foreground='white',
                        font=font)
    
    def get_log_color(self, level: str) -> str:
        """Get color for a log level."""
        level_colors = {
            'INFO': self.colors.log_info,
            'SUCCESS': self.colors.log_success,
            'WARNING': self.colors.log_warning,
            'ERROR': self.colors.log_error,
            'DEBUG': self.colors.log_debug,
            'CONFIG': self.colors.fg_secondary,
        }
        return level_colors.get(level.upper(), self.colors.log_info)
    
    def create_button_config(self, button_type: str) -> Dict[str, Any]:
        """Create button configuration dictionary."""
        configs = {
            'start': {
                'bg': self.colors.accent_green,
                'fg': 'white',
                'font': self.fonts.body_bold(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'stop': {
                'bg': self.colors.accent_red,
                'fg': 'white',
                'font': self.fonts.body_bold(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'command': {
                'bg': self.colors.accent_blue,
                'fg': 'white',
                'font': self.fonts.small_bold(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'platoon': {
                'bg': self.colors.accent_purple,
                'fg': 'white',
                'font': self.fonts.small_bold(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'secondary': {
                'bg': self.colors.accent_gray,
                'fg': 'white',
                'font': self.fonts.small(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'warning': {
                'bg': self.colors.accent_orange,
                'fg': 'white',
                'font': self.fonts.body_bold(),
                'relief': 'flat',
                'cursor': 'hand2',
            },
            'disabled': {
                'bg': '#4d4d4d',
                'fg': 'white',
                'font': self.fonts.small_bold(),
                'relief': 'flat',
            },
        }
        return configs.get(button_type, configs['command'])
    
    def create_entry_config(self) -> Dict[str, Any]:
        """Create entry widget configuration."""
        return {
            'bg': self.colors.bg_light,
            'fg': self.colors.fg_primary,
            'font': self.fonts.small(),
            'insertbackground': self.colors.fg_primary,
            'relief': 'flat',
        }
    
    def create_label_config(self, style: str = 'default') -> Dict[str, Any]:
        """Create label widget configuration."""
        configs = {
            'default': {
                'bg': self.colors.bg_medium,
                'fg': self.colors.fg_primary,
                'font': self.fonts.body(),
            },
            'title': {
                'bg': self.colors.bg_header,
                'fg': self.colors.fg_primary,
                'font': self.fonts.subtitle(),
            },
            'muted': {
                'bg': self.colors.bg_medium,
                'fg': self.colors.fg_muted,
                'font': self.fonts.small(),
            },
            'value': {
                'bg': self.colors.bg_medium,
                'fg': self.colors.fg_primary,
                'font': self.fonts.small_bold(),
            },
        }
        return configs.get(style, configs['default'])
    
    def create_labelframe_config(self) -> Dict[str, Any]:
        """Create LabelFrame configuration."""
        return {
            'bg': self.colors.bg_medium,
            'fg': self.colors.fg_primary,
            'font': self.fonts.heading(),
        }


# Default theme instance
DEFAULT_THEME = Theme()
