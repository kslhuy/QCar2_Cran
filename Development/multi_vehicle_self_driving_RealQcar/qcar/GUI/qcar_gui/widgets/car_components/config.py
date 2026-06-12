import tkinter as tk
from typing import Callable, Optional

from ...theme import Theme
from ..base import BaseWidget, ThemedLabelFrame, ThemedLabel, ThemedButton, ThemedEntry
from .types import CarPanelCallbacks

class PerceptionControl(BaseWidget):
    """Widget for perception system (YOLO) control."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._perception_btn: Optional[tk.Button] = None
        self._probing_btn: Optional[tk.Button] = None
        self._perception_active = False
        self._probing_active = False
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(
            self.parent, text="👁️ Perception", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        btn_row = tk.Frame(content, bg=c.bg_medium)
        btn_row.pack(fill="x")

        self._perception_btn = ThemedButton(
            btn_row,
            text="👁️ Activate YOLO",
            button_type="command",
            command=self._toggle_perception,
            padx=10,
            pady=3,
        )
        self._perception_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))

        self._probing_btn = ThemedButton(
            btn_row,
            text="📺 Probing",
            button_type="command",
            command=self._toggle_probing,
            padx=10,
            pady=3,
        )
        self._probing_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

    def _toggle_perception(self) -> None:
        if self.callbacks.on_toggle_perception:
            self.callbacks.on_toggle_perception(self.car_id)

    def _toggle_probing(self) -> None:
        if self.callbacks.on_toggle_probing:
            self.callbacks.on_toggle_probing(self.car_id)

    def set_perception_active(self, active: bool) -> None:
        self._perception_active = active
        if self._perception_btn:
            if active:
                self._perception_btn.config(
                    text="👁️ YOLO: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._perception_btn.config(
                    text="👁️ Activate YOLO", bg=self.theme.colors.accent_blue
                )

    def set_probing_active(self, active: bool) -> None:
        self._probing_active = active
        if self._probing_btn:
            if active:
                self._probing_btn.config(
                    text="📺 Probing: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._probing_btn.config(
                    text="📺 Probing", bg=self.theme.colors.accent_blue
                )

    @property
    def is_active(self) -> bool:
        return self._perception_active

    @property
    def is_probing_active(self) -> bool:
        return self._probing_active


class ScopesControl(BaseWidget):
    """Widget for estimation scopes and remote plotting control."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._scopes_btn: Optional[tk.Button] = None
        self._remote_plot_btn: Optional[tk.Button] = None
        self._scopes_active = False
        self._remote_plot_active = False
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(self.parent, text="📊 Scopes", theme=self.theme)

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        row = tk.Frame(content, bg=c.bg_medium)
        row.pack(fill="x")

        self._remote_local_btn = ThemedButton(
            row,
            text="📡 Local Plot",
            button_type="command",
            command=self._toggle_remote_local,
            padx=10,
            pady=3,
        )
        self._remote_local_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))

        self._remote_fleet_btn = ThemedButton(
            row,
            text="📡 Fleet Plot",
            button_type="command",
            command=self._toggle_remote_fleet,
            padx=10,
            pady=3,
        )
        self._remote_fleet_btn.pack(side="left", expand=True, fill="x", padx=(2, 0))

        self._remote_fleet_btn.config(state="disabled")
        self._fleet_enabled = False

    def _toggle_scopes(self) -> None:
        if self.callbacks.on_toggle_scopes:
            self.callbacks.on_toggle_scopes(self.car_id)

    def _toggle_remote_local(self) -> None:
        if self.callbacks.on_toggle_remote_plot_local:
            self.callbacks.on_toggle_remote_plot_local(self.car_id)

    def _toggle_remote_fleet(self) -> None:
        if self.callbacks.on_toggle_remote_plot_fleet:
            self.callbacks.on_toggle_remote_plot_fleet(self.car_id)

    def set_scopes_active(self, active: bool) -> None:
        self._scopes_active = active
        if hasattr(self, "_scopes_btn") and self._scopes_btn:
            if active:
                self._scopes_btn.config(
                    text="📊 Local: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._scopes_btn.config(
                    text="📊 Local Plots", bg=self.theme.colors.accent_blue
                )

    def set_remote_local_active(self, active: bool) -> None:
        self._remote_local_active = active
        if self._remote_local_btn:
            if active:
                self._remote_local_btn.config(
                    text="📡 Local: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._remote_local_btn.config(
                    text="📡 Local Plot", bg=self.theme.colors.accent_blue
                )

    def set_remote_fleet_active(self, active: bool) -> None:
        self._remote_fleet_active = active
        if self._remote_fleet_btn:
            if active:
                self._remote_fleet_btn.config(
                    text="📡 Fleet: ON", bg=self.theme.colors.accent_green
                )
            else:
                self._remote_fleet_btn.config(
                    text="📡 Fleet Plot", bg=self.theme.colors.accent_blue
                )

    def set_fleet_button_enabled(self, enabled: bool) -> None:
        self._fleet_enabled = enabled
        if self._remote_fleet_btn:
            self._remote_fleet_btn.config(state="normal" if enabled else "disabled")

    @property
    def is_active(self) -> bool:
        return self._scopes_active

    @property
    def is_remote_local_active(self) -> bool:
        return getattr(self, "_remote_local_active", False)

    @property
    def is_remote_fleet_active(self) -> bool:
        return getattr(self, "_remote_fleet_active", False)


class PlatoonControl(BaseWidget):
    """Widget for platoon configuration."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self._position_var: Optional[tk.StringVar] = None
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        self.frame = ThemedLabelFrame(self.parent, text="🚗 Platoon", theme=self.theme)

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        position_frame = tk.Frame(content, bg=c.bg_medium)
        position_frame.pack(fill="x", pady=(0, 3))

        ThemedLabel(
            position_frame, text="Position:", style="muted", theme=self.theme
        ).pack(side="left", padx=(0, 5))

        self._position_var = tk.StringVar(value=str(self.car_id + 1))
        position_entry = ThemedEntry(position_frame, width=3, theme=self.theme)
        position_entry.insert(0, str(self.car_id + 1))
        position_entry.pack(side="left", padx=(0, 5))
        position_entry.bind("<KeyRelease>", self._on_position_change)
        self._position_entry = position_entry

        ThemedLabel(
            position_frame,
            text="(1=Leader, 2,3...=Followers)",
            style="muted",
            theme=self.theme,
        ).pack(side="left", padx=(5, 0))

    def _on_position_change(self, event=None) -> None:
        if self.callbacks.on_platoon_position_change:
            try:
                position = int(self._position_entry.get())
                self.callbacks.on_platoon_position_change(self.car_id, position)
            except ValueError:
                pass

    @property
    def position(self) -> int:
        try:
            return int(self._position_entry.get())
        except (ValueError, AttributeError):
            return self.car_id + 1


class ControllerTuningControl(BaseWidget):
    """Widget for tuning controller parameters."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors
        self.frame = ThemedLabelFrame(
            self.parent, text="🎛️ Controller Tuning", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        row = tk.Frame(content, bg=c.bg_medium)
        row.pack(fill="x", pady=2)

        self._context_var = tk.StringVar(value="path")
        ctx_menu = tk.OptionMenu(row, self._context_var, "path", "leader")
        ctx_menu.config(
            bg=c.bg_light,
            fg=c.fg_primary,
            highlightthickness=0,
            font=self.theme.fonts.tiny(),
        )
        ctx_menu["menu"].config(bg=c.bg_light, fg=c.fg_primary)
        ctx_menu.pack(side="left", padx=(0, 2))

        self._category_var = tk.StringVar(value="lateral")
        cat_menu = tk.OptionMenu(row, self._category_var, "longitudinal", "lateral")
        cat_menu.config(
            bg=c.bg_light,
            fg=c.fg_primary,
            highlightthickness=0,
            font=self.theme.fonts.tiny(),
        )
        cat_menu["menu"].config(bg=c.bg_light, fg=c.fg_primary)
        cat_menu.pack(side="left", padx=(0, 5))

        row2 = tk.Frame(content, bg=c.bg_medium)
        row2.pack(fill="x", pady=(2, 0))

        ThemedLabel(row2, text="Param:", style="muted", theme=self.theme).pack(
            side="left"
        )
        self._param_name = ThemedEntry(row2, width=10, theme=self.theme)
        self._param_name.pack(side="left", padx=(2, 5))

        ThemedLabel(row2, text="Value:", style="muted", theme=self.theme).pack(
            side="left"
        )
        self._param_val = ThemedEntry(row2, width=8, theme=self.theme)
        self._param_val.pack(side="left", padx=(2, 5))

        ThemedButton(
            row2,
            text="Update",
            button_type="command",
            command=self._on_update,
            padx=6,
            pady=1,
        ).pack(side="left")

    def _on_update(self):
        if self.callbacks.on_set_controller_params:
            try:
                val = float(self._param_val.get())
                param_name = self._param_name.get().strip()
                if param_name:
                    params_dict = {param_name: val}
                    self.callbacks.on_set_controller_params(
                        self.car_id,
                        self._category_var.get(),
                        params_dict,
                        self._context_var.get(),
                    )
            except ValueError:
                pass


class RuntimeSwitchingControl(BaseWidget):
    """Widget for runtime observer and controller switching."""

    def __init__(
        self,
        parent: tk.Widget,
        car_id: int,
        callbacks: CarPanelCallbacks,
        config: dict = None,
        theme: Theme = None,
    ):
        self.car_id = car_id
        self.callbacks = callbacks
        self.config = config or {}
        self._local_obs_var: Optional[tk.StringVar] = None
        self._fleet_obs_var: Optional[tk.StringVar] = None
        self._path_long_ctrl_var: Optional[tk.StringVar] = None
        self._path_lat_ctrl_var: Optional[tk.StringVar] = None
        self._leader_long_ctrl_var: Optional[tk.StringVar] = None
        self._leader_lat_ctrl_var: Optional[tk.StringVar] = None
        super().__init__(parent, theme)

    def _build(self) -> None:
        c = self.theme.colors

        local_obs = self.config.get(
            "local_observers",
            ["ekf", "luenberger", "neural_luenberger", "robust_kalman_net", "high_gain_observer"],
        )
        fleet_obs = self.config.get(
            "fleet_observers",
            ["consensus", "distributed_luenberger", "trust_consensus", "trust_kalman", "distributed_longitudinal_high_gain"],
        )
        path_long = self.config.get(
            "path_longitudinal_controllers", ["pid", "qcar2_speed", "cacc", "sa_acc", "lcc"]
        )
        path_lat = self.config.get(
            "path_lateral_controllers", ["pp_map", "path", "stanley", "mpc", "dummy"]
        )
        leader_long = self.config.get(
            "leader_longitudinal_controllers", ["cacc", "pid", "qcar2_speed", "sa_acc", "lcc"]
        )
        leader_lat = self.config.get(
            "leader_lateral_controllers",
            ["pure_pursuit", "stanley", "lookahead", "hybrid", "fusion", "mpc", "dummy"],
        )

        self.frame = ThemedLabelFrame(
            self.parent, text="⚙️ Runtime Config", theme=self.theme
        )

        content = tk.Frame(self.frame, bg=c.bg_medium)
        content.pack(fill="x", padx=6, pady=4)

        # Observer row (Local + Fleet side-by-side)
        self._build_section_label(content, "👀 Observers")
        obs_row = tk.Frame(content, bg=c.bg_medium)
        obs_row.pack(fill="x", pady=2)
        
        self._build_dropdown_component(
            obs_row,
            "Local:",
            local_obs,
            "_local_obs_var",
            self._apply_local_observer,
        )
        self._build_dropdown_component(
            obs_row,
            "Fleet:",
            fleet_obs,
            "_fleet_obs_var",
            self._apply_fleet_observer,
            padx=(5, 0)
        )

        # Following Path row (Long + Lat side-by-side)
        self._build_section_label(content, "🛤️ Following Path")
        path_row = tk.Frame(content, bg=c.bg_medium)
        path_row.pack(fill="x", pady=2)
        
        self._build_dropdown_component(
            path_row,
            "Long:",
            path_long,
            "_path_long_ctrl_var",
            self._apply_path_long_controller,
        )
        self._build_dropdown_component(
            path_row,
            "Lat:",
            path_lat,
            "_path_lat_ctrl_var",
            self._apply_path_lat_controller,
            padx=(5, 0)
        )

        # Following Leader row (Long + Lat side-by-side)
        self._build_section_label(content, "🚗 Following Leader")
        leader_row = tk.Frame(content, bg=c.bg_medium)
        leader_row.pack(fill="x", pady=2)
        
        self._build_dropdown_component(
            leader_row,
            "Long:",
            leader_long,
            "_leader_long_ctrl_var",
            self._apply_leader_long_controller,
        )
        self._build_dropdown_component(
            leader_row,
            "Lat:",
            leader_lat,
            "_leader_lat_ctrl_var",
            self._apply_leader_lat_controller,
            padx=(5, 0)
        )

    def _build_section_label(self, parent: tk.Frame, text: str) -> None:
        c = self.theme.colors
        label = tk.Label(
            parent,
            text=text,
            bg=c.bg_medium,
            fg=c.accent_blue,
            font=self.theme.fonts.tiny(),
            anchor="w",
        )
        label.pack(fill="x", pady=(4, 0))

    def _build_dropdown_component(
        self,
        parent: tk.Frame,
        label_text: str,
        options: list,
        var_attr: str,
        apply_callback: Callable,
        padx: tuple = (0, 0)
    ) -> None:
        c = self.theme.colors

        container = tk.Frame(parent, bg=c.bg_medium)
        container.pack(side="left", fill="x", expand=True, padx=padx)

        ThemedLabel(container, text=label_text, style="muted", theme=self.theme).pack(
            side="left", padx=(0, 2)
        )

        var = tk.StringVar(value=options[0] if options else "")
        setattr(self, var_attr, var)

        dropdown = tk.OptionMenu(container, var, *options)
        dropdown.config(
            bg=c.bg_light,
            fg=c.fg_primary,
            activebackground=c.bg_medium,
            activeforeground=c.fg_primary,
            highlightthickness=0,
            font=self.theme.fonts.tiny(),
            width=8,
        )
        dropdown["menu"].config(
            bg=c.bg_light,
            fg=c.fg_primary,
            activebackground=c.accent_blue,
            activeforeground=c.fg_primary,
        )
        dropdown.pack(side="left", padx=(0, 2))

        # Smaller apply button
        ThemedButton(
            container,
            text="Set",
            button_type="command",
            command=apply_callback,
            padx=4,
            pady=0,
            width=2,
        ).pack(side="left")

    def _apply_local_observer(self) -> None:
        if self.callbacks.on_set_local_observer and self._local_obs_var:
            observer_type = self._local_obs_var.get()
            self.callbacks.on_set_local_observer(self.car_id, observer_type)

    def _apply_fleet_observer(self) -> None:
        if self.callbacks.on_set_fleet_observer and self._fleet_obs_var:
            observer_type = self._fleet_obs_var.get()
            self.callbacks.on_set_fleet_observer(self.car_id, observer_type)

    def _apply_path_long_controller(self) -> None:
        if self.callbacks.on_set_controller and self._path_long_ctrl_var:
            controller_type = self._path_long_ctrl_var.get()
            self.callbacks.on_set_controller(
                self.car_id, "longitudinal", controller_type, "path"
            )

    def _apply_path_lat_controller(self) -> None:
        if self.callbacks.on_set_controller and self._path_lat_ctrl_var:
            controller_type = self._path_lat_ctrl_var.get()
            self.callbacks.on_set_controller(
                self.car_id, "lateral", controller_type, "path"
            )

    def _apply_leader_long_controller(self) -> None:
        if self.callbacks.on_set_controller and self._leader_long_ctrl_var:
            controller_type = self._leader_long_ctrl_var.get()
            self.callbacks.on_set_controller(
                self.car_id, "longitudinal", controller_type, "leader"
            )

    def _apply_leader_lat_controller(self) -> None:
        if self.callbacks.on_set_controller and self._leader_lat_ctrl_var:
            controller_type = self._leader_lat_ctrl_var.get()
            self.callbacks.on_set_controller(
                self.car_id, "lateral", controller_type, "leader"
            )

    def set_current_values(
        self,
        local_obs: str = None,
        fleet_obs: str = None,
        path_long_ctrl: str = None,
        path_lat_ctrl: str = None,
        leader_long_ctrl: str = None,
        leader_lat_ctrl: str = None,
    ) -> None:
        if local_obs and self._local_obs_var:
            self._local_obs_var.set(local_obs)
        if fleet_obs and self._fleet_obs_var:
            self._fleet_obs_var.set(fleet_obs)
        if path_long_ctrl and self._path_long_ctrl_var:
            self._path_long_ctrl_var.set(path_long_ctrl)
        if path_lat_ctrl and self._path_lat_ctrl_var:
            self._path_lat_ctrl_var.set(path_lat_ctrl)
        if leader_long_ctrl and self._leader_long_ctrl_var:
            self._leader_long_ctrl_var.set(leader_long_ctrl)
        if leader_lat_ctrl and self._leader_lat_ctrl_var:
            self._leader_lat_ctrl_var.set(leader_lat_ctrl)
