"""Kit extension entry point: checks the OpenBeam core and offers the demo scene."""

from __future__ import annotations

import carb
import omni.ext

from . import core

_EXT = "openbeam.isaacsim.softbody"
_DEMO_ACTION = "load_wrecking_ball_demo"


class OpenBeamSoftBodyExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._menu = None
        self._demo = None
        try:
            path = core.library_path()
            core._load()
            carb.log_warn(f"[{_EXT}] started: openbeam_core API 1 from {path}")
        except Exception as e:  # the extension stays loaded; SoftBody raises on use
            carb.log_error(f"[{_EXT}] openbeam_core unavailable: {e}")
        # Window > OpenBeam > Wrecking Ball Demo, where the app has menus.
        try:
            import omni.kit.actions.core
            from omni.kit.menu.utils import MenuItemDescription, add_menu_items
        except ImportError:  # headless apps have no menus
            return
        omni.kit.actions.core.get_action_registry().register_action(
            _EXT, _DEMO_ACTION, self.load_demo, display_name="OpenBeam Wrecking Ball Demo",
            description="Build the OpenBeam wrecking-ball demo in the open stage")
        self._menu = [MenuItemDescription(name="OpenBeam", sub_menu=[
            MenuItemDescription(name="Wrecking Ball Demo", onclick_action=(_EXT, _DEMO_ACTION))])]
        add_menu_items(self._menu, "Window")

    def load_demo(self) -> None:
        """Builds the wrecking-ball scene in the open stage; press Play to run it."""
        import omni.usd

        from .scene import CRATE_POSITION, DEFAULT_CRATE, build_wrecking_ball
        from .softbody import SoftBody

        if self._demo is not None:
            self._demo.destroy()
        stage = omni.usd.get_context().get_stage()
        paths = build_wrecking_ball(stage)
        self._demo = SoftBody(DEFAULT_CRATE, position=CRATE_POSITION, colliders=[paths["ball"]])
        carb.log_warn(f"[{_EXT}] demo scene ready: press Play")

    def on_shutdown(self) -> None:
        if self._demo is not None:
            self._demo.destroy()
            self._demo = None
        if self._menu:
            import omni.kit.actions.core
            from omni.kit.menu.utils import remove_menu_items

            remove_menu_items(self._menu, "Window")
            omni.kit.actions.core.get_action_registry().deregister_all_actions_for_extension(_EXT)
            self._menu = None
