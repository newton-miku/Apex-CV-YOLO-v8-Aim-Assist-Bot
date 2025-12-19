import time
import threading
from typing import Optional
from makcu import create_controller


class MakcuMouseController:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 1.0, handlike: bool = False):
        self.controller = create_controller(debug=False, auto_reconnect=True)
        self.recoil_active = False
        self.internal_hand_like = handlike
        self.internal_hand_like_sep_time = 2
        self.recoil_thread: Optional[threading.Thread] = None
        self.recoil_lock = threading.Lock()
        self.recoil_delay_ms = 4
        self.recoil_step1 = (-2, 6)
        self.recoil_step2 = (2, -3)

    def init_serial(self) -> bool:
        try:
            return self.controller is not None and self.controller.is_connected()
        except Exception:
            return False

    def send_mouse_move(self, dx: int, dy: int) -> bool:
        try:
            dx = int(dx)
            dy = int(dy)
            if self.internal_hand_like:
                self.controller.move_smooth(dx, dy, segments=self.internal_hand_like_sep_time)
            else:
                self.controller.move(dx, dy)
            return True
        except Exception:
            return False

    def _internal_mouse_move(self, dx, dy):
        try:
            dx = int(dx)
            dy = int(dy)
            self.controller.move(dx, dy)
            return True
        except Exception:
            return False

    def _internal_recoil_control(self):
        while True:
            with self.recoil_lock:
                if not self.recoil_active:
                    break
            self._internal_mouse_move(*self.recoil_step1)
            time.sleep(self.recoil_delay_ms / 1000)
            with self.recoil_lock:
                if not self.recoil_active:
                    break
            self._internal_mouse_move(*self.recoil_step2)
            time.sleep(self.recoil_delay_ms / 1000)

    def start_recoil(self):
        with self.recoil_lock:
            if self.recoil_active:
                return
            self.recoil_active = True
        self.recoil_thread = threading.Thread(target=self._internal_recoil_control, daemon=True)
        self.recoil_thread.start()

    def stop_recoil(self):
        with self.recoil_lock:
            if not self.recoil_active:
                return
            self.recoil_active = False
        if self.recoil_thread and self.recoil_thread.is_alive():
            self.recoil_thread.join(timeout=0.1)

    def close(self):
        self.stop_recoil()
        try:
            if self.controller is not None:
                self.controller.disconnect()
        except Exception:
            pass
