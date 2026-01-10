"""
Base widgets module for QCar Fleet Controller.

This module contains reusable base widget classes that provide
common functionality for the application's UI components.
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, Callable, Any, Dict
from abc import ABC, abstractmethod

from ..theme import Theme, DEFAULT_THEME


class BaseWidget(ABC):
    """Abstract base class for all custom widgets."""
    
    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self.parent = parent
        self.theme = theme or DEFAULT_THEME
        self.frame: Optional[tk.Frame] = None
        self._build()
    
    @abstractmethod
    def _build(self) -> None:
        """Build the widget. Must be implemented by subclasses."""
        pass
    
    def pack(self, **kwargs) -> 'BaseWidget':
        """Pack the widget's main frame."""
        if self.frame:
            self.frame.pack(**kwargs)
        return self
    
    def grid(self, **kwargs) -> 'BaseWidget':
        """Grid the widget's main frame."""
        if self.frame:
            self.frame.grid(**kwargs)
        return self
    
    def destroy(self) -> None:
        """Destroy the widget."""
        if self.frame:
            self.frame.destroy()


class ThemedButton(tk.Button):
    """A themed button with consistent styling."""
    
    def __init__(self, parent: tk.Widget, text: str, 
                 button_type: str = 'command',
                 command: Callable = None,
                 theme: Theme = None,
                 **kwargs):
        self.theme = theme or DEFAULT_THEME
        config = self.theme.create_button_config(button_type)
        config.update(kwargs)
        
        super().__init__(parent, text=text, command=command, **config)
    
    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the button with appropriate styling."""
        if enabled:
            self.config(state='normal')
        else:
            disabled_config = self.theme.create_button_config('disabled')
            self.config(state='disabled', **disabled_config)


class ThemedEntry(tk.Entry):
    """A themed entry widget with consistent styling."""
    
    def __init__(self, parent: tk.Widget, width: int = 10,
                 theme: Theme = None, **kwargs):
        self.theme = theme or DEFAULT_THEME
        config = self.theme.create_entry_config()
        config['width'] = width
        config.update(kwargs)
        
        super().__init__(parent, **config)
    
    def get_float(self, default: float = 0.0) -> float:
        """Get value as float, returning default on error."""
        try:
            return float(self.get())
        except ValueError:
            return default
    
    def get_int(self, default: int = 0) -> int:
        """Get value as int, returning default on error."""
        try:
            return int(self.get())
        except ValueError:
            return default
    
    def get_list(self, separator: str = ',', item_type: type = int) -> list:
        """Get value as a list of items."""
        try:
            text = self.get().replace(separator, ' ')
            return [item_type(x.strip()) for x in text.split() if x.strip()]
        except ValueError:
            return []


class ThemedLabel(tk.Label):
    """A themed label with consistent styling."""
    
    def __init__(self, parent: tk.Widget, text: str = '',
                 style: str = 'default',
                 theme: Theme = None, **kwargs):
        self.theme = theme or DEFAULT_THEME
        config = self.theme.create_label_config(style)
        config.update(kwargs)
        
        super().__init__(parent, text=text, **config)


class ThemedLabelFrame(tk.LabelFrame):
    """A themed label frame with consistent styling."""
    
    def __init__(self, parent: tk.Widget, text: str = '',
                 theme: Theme = None, **kwargs):
        self.theme = theme or DEFAULT_THEME
        config = self.theme.create_labelframe_config()
        config.update(kwargs)
        
        super().__init__(parent, text=text, **config)


class ExpandablePanel(BaseWidget):
    """An expandable/collapsible panel widget."""
    
    def __init__(self, parent: tk.Widget, title: str,
                 expanded: bool = True,
                 theme: Theme = None,
                 on_toggle: Callable[[bool], None] = None):
        self.title = title
        self._expanded = expanded
        self.on_toggle = on_toggle
        self._content_frame: Optional[tk.Frame] = None
        self._header_widgets: Dict[str, tk.Widget] = {}
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the expandable panel."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_medium, relief='raised', bd=2)
        
        # Header (increased height for better clickability)
        self._header = tk.Frame(self.frame, bg=c.bg_header, height=50)
        self._header.pack(fill='x')
        self._header.pack_propagate(False)
        
        # Expand button (larger click area)
        self._expand_btn = tk.Label(
            self._header,
            text="▼" if self._expanded else "▶",
            bg=c.bg_header,
            fg=c.accent_green,
            font=self.theme.fonts.subtitle(),
            cursor='hand2',
            padx=15,
            pady=15
        )
        self._expand_btn.pack(side='left', padx=(10, 5), pady=5)
        
        # Title (fill entire header for better clickability)
        self._title_label = tk.Label(
            self._header,
            text=self.title,
            bg=c.bg_header,
            fg=c.fg_primary,
            font=self.theme.fonts.subtitle(),
            cursor='hand2'
        )
        self._title_label.pack(side='left', fill='both', expand=True, padx=10, pady=15)
        
        # Content frame
        self._content_frame = tk.Frame(self.frame, bg=c.bg_medium)
        if self._expanded:
            self._content_frame.pack(fill='x', padx=10, pady=10)
        
        # Bind toggle events
        for widget in [self._expand_btn, self._header, self._title_label]:
            widget.bind('<Button-1>', self._toggle)
            self._setup_hover_effects(widget)
    
    def _setup_hover_effects(self, widget: tk.Widget) -> None:
        """Setup hover effects for a widget."""
        c = self.theme.colors
        if widget != self._expand_btn:
            widget.bind('<Enter>', lambda e: widget.config(bg=c.bg_light))
            widget.bind('<Leave>', lambda e: widget.config(bg=c.bg_header))
    
    def _toggle(self, event=None) -> None:
        """Toggle the panel expansion state."""
        self._expanded = not self._expanded
        self._expand_btn.config(text="▼" if self._expanded else "▶")
        
        if self._expanded:
            self._content_frame.pack(fill='x', padx=10, pady=10)
        else:
            self._content_frame.pack_forget()
        
        if self.on_toggle:
            self.on_toggle(self._expanded)
    
    @property
    def content(self) -> tk.Frame:
        """Get the content frame for adding child widgets."""
        return self._content_frame
    
    @property
    def header(self) -> tk.Frame:
        """Get the header frame for adding status indicators."""
        return self._header
    
    def add_header_widget(self, name: str, widget: tk.Widget) -> None:
        """Add a widget to the header."""
        self._header_widgets[name] = widget
    
    def get_header_widget(self, name: str) -> Optional[tk.Widget]:
        """Get a header widget by name."""
        return self._header_widgets.get(name)
    
    @property
    def is_expanded(self) -> bool:
        """Check if panel is expanded."""
        return self._expanded


class StatusIndicator(tk.Label):
    """A status indicator label with icon and color coding."""
    
    STATUS_CONFIGS = {
        'connected': ('🟢', 'Connected', '#4caf50'),
        'disconnected': ('🔴', 'Disconnected', '#f44336'),
        'pending': ('🟡', 'Pending', '#ff9800'),
        'inactive': ('⚪', 'Inactive', '#888888'),
        'error': ('❌', 'Error', '#f44336'),
        'active': ('✅', 'Active', '#4caf50'),
    }
    
    def __init__(self, parent: tk.Widget, status: str = 'inactive',
                 show_text: bool = True, theme: Theme = None, **kwargs):
        self.theme = theme or DEFAULT_THEME
        self._show_text = show_text
        
        config = {
            'bg': self.theme.colors.bg_header,
            'font': self.theme.fonts.heading(),
            'padx': 10,
        }
        config.update(kwargs)
        
        super().__init__(parent, **config)
        self.set_status(status)
    
    def set_status(self, status: str, custom_text: str = None) -> None:
        """Set the status with appropriate icon and color."""
        icon, default_text, color = self.STATUS_CONFIGS.get(
            status, ('⚪', 'Unknown', '#888888')
        )
        
        text = custom_text if custom_text else default_text
        display_text = f"{icon} {text}" if self._show_text else icon
        
        self.config(text=display_text, fg=color)


class ScrollableFrame(BaseWidget):
    """A scrollable frame widget with canvas and scrollbar."""
    
    def __init__(self, parent: tk.Widget, theme: Theme = None):
        self._canvas: Optional[tk.Canvas] = None
        self._scrollbar: Optional[tk.Scrollbar] = None
        self._inner_frame: Optional[tk.Frame] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the scrollable frame."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_dark)
        
        self._canvas = tk.Canvas(self.frame, bg=c.bg_dark, highlightthickness=0)
        self._scrollbar = tk.Scrollbar(
            self.frame, orient='vertical', command=self._canvas.yview
        )
        
        self._inner_frame = tk.Frame(self._canvas, bg=c.bg_dark)
        
        self._inner_frame.bind(
            '<Configure>',
            lambda e: self._canvas.configure(scrollregion=self._canvas.bbox('all'))
        )
        
        self._canvas.create_window((0, 0), window=self._inner_frame, anchor='nw')
        self._canvas.configure(yscrollcommand=self._scrollbar.set)
        
        self._canvas.pack(side='left', fill='both', expand=True)
        self._scrollbar.pack(side='right', fill='y')
        
        # Bind mouse wheel
        self._canvas.bind_all('<MouseWheel>', self._on_mousewheel)
    
    def _on_mousewheel(self, event) -> None:
        """Handle mouse wheel scrolling."""
        self._canvas.yview_scroll(int(-1 * (event.delta / 120)), 'units')
    
    @property
    def content(self) -> tk.Frame:
        """Get the inner scrollable frame for adding child widgets."""
        return self._inner_frame
    
    def scroll_to_top(self) -> None:
        """Scroll to the top of the content."""
        self._canvas.yview_moveto(0)
    
    def scroll_to_bottom(self) -> None:
        """Scroll to the bottom of the content."""
        self._canvas.yview_moveto(1)


class FormRow(BaseWidget):
    """A form row with label, entry, and optional button."""
    
    def __init__(self, parent: tk.Widget, label: str,
                 default_value: str = '',
                 entry_width: int = 10,
                 button_text: str = None,
                 button_command: Callable = None,
                 theme: Theme = None):
        self.label_text = label
        self.default_value = default_value
        self.entry_width = entry_width
        self.button_text = button_text
        self.button_command = button_command
        self._entry: Optional[ThemedEntry] = None
        super().__init__(parent, theme)
    
    def _build(self) -> None:
        """Build the form row."""
        c = self.theme.colors
        
        self.frame = tk.Frame(self.parent, bg=c.bg_medium)
        
        # Label
        ThemedLabel(
            self.frame,
            text=self.label_text,
            style='muted',
            theme=self.theme
        ).pack(side='left', padx=(0, 8))
        
        # Entry
        self._entry = ThemedEntry(
            self.frame,
            width=self.entry_width,
            theme=self.theme
        )
        self._entry.insert(0, self.default_value)
        self._entry.pack(side='left', padx=(0, 8))
        
        # Optional button
        if self.button_text and self.button_command:
            ThemedButton(
                self.frame,
                text=self.button_text,
                button_type='command',
                command=self.button_command,
                theme=self.theme,
                padx=12,
                pady=4
            ).pack(side='left')
    
    @property
    def entry(self) -> ThemedEntry:
        """Get the entry widget."""
        return self._entry
    
    def get_value(self) -> str:
        """Get the current entry value."""
        return self._entry.get() if self._entry else ''
    
    def set_value(self, value: str) -> None:
        """Set the entry value."""
        if self._entry:
            self._entry.delete(0, tk.END)
            self._entry.insert(0, value)
