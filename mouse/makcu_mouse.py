import time
import threading
from typing import Optional
from makcu import create_controller
import queue


class MakcuMouseController:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 1.0, handlike: bool = False):
        self.controller = create_controller(debug=False, auto_reconnect=True)
        self.recoil_active = False
        self.internal_hand_like = handlike
        self.internal_hand_like_sep_time = 3
        self.recoil_thread: Optional[threading.Thread] = None
        self.recoil_lock = threading.Lock()
        self.recoil_event = threading.Event()  # 使用Event控制压枪线程启停
        self.recoil_delay_ms = 4
        self.recoil_step1 = (-2, 6)
        self.recoil_step2 = (2, -4)
        self.move_queue = queue.Queue(maxsize=256)
        self.running = True
        self.min_interval_ms = 4
        self.last_move_ts = 0.0

        # 优化：添加移动向量限制和合并优化参数
        self.max_move_magnitude = 300  # 最大移动向量幅度
        self.merge_max_count = 10  # 每次最多合并10个移动指令

        self.move_worker = threading.Thread(target=self._move_worker, daemon=True)
        self.move_worker.start()

        # 创建单常驻压枪线程
        self._recoil_worker_started = False

    def init_serial(self) -> bool:
        try:
            return self.controller is not None and self.controller.is_connected()
        except Exception:
            return False

    def send_mouse_move(self, dx: int, dy: int) -> bool:
        try:
            dx = int(dx)
            dy = int(dy)

            # 优化：限制移动向量幅度，防止过大移动
            magnitude = (dx ** 2 + dy ** 2) ** 0.5
            if magnitude > self.max_move_magnitude:
                # 按比例缩放到最大幅度
                scale = self.max_move_magnitude / magnitude if magnitude > 0 else 1.0
                dx = int(dx * scale)
                dy = int(dy * scale)

            try:
                self.move_queue.put_nowait((dx, dy))
            except queue.Full:
                # 队列满时，移除最旧的指令，添加新指令
                try:
                    _ = self.move_queue.get_nowait()
                except Exception:
                    pass
                try:
                    self.move_queue.put_nowait((dx, dy))
                except Exception:
                    return False
            return True
        except Exception:
            return False

    def _internal_mouse_move(self, dx, dy):
        return self.send_mouse_move(dx, dy)

    def _move_worker(self):
        """优化的移动处理线程：智能合并移动指令，减少串口通信次数"""
        while self.running:
            try:
                dx, dy = self.move_queue.get(timeout=0.01)
            except Exception:
                continue

            acc_dx, acc_dy = dx, dy
            merge_count = 1

            # 优化：限制合并数量，避免过多累积
            try:
                while merge_count < self.merge_max_count:
                    nx, ny = self.move_queue.get_nowait()
                    acc_dx += nx
                    acc_dy += ny
                    merge_count += 1
            except Exception:
                pass

            now = time.time()
            # 检查最小间隔
            elapsed_ms = (now - self.last_move_ts) * 1000.0
            if elapsed_ms < self.min_interval_ms:
                # 将累积的移动重新放回队列，等待下次处理
                try:
                    self.move_queue.put_nowait((acc_dx, acc_dy))
                except Exception:
                    pass
                continue

            self.last_move_ts = now
            try:
                if self.internal_hand_like:
                    self.controller.move_smooth(acc_dx, acc_dy, segments=self.internal_hand_like_sep_time)
                else:
                    self.controller.move(acc_dx, acc_dy)
            except Exception:
                pass

    def _internal_recoil_control(self):
        """压枪控制循环，使用time.sleep保持固定节奏，Event用于快速中断"""
        while self.running:
            # 等待压枪启用（阻塞直到set）
            self.recoil_event.wait()
            if not self.running:
                break

            # 执行第一步压枪移动
            with self.recoil_lock:
                if not self.recoil_active:
                    continue  # 回到wait状态
            self._internal_mouse_move(*self.recoil_step1)

            # 固定延迟，保持节奏
            time.sleep(self.recoil_delay_ms / 1000)

            # 检查是否需要停止
            with self.recoil_lock:
                if not self.recoil_active:
                    continue  # 回到wait状态

            # 执行第二步压枪移动
            self._internal_mouse_move(*self.recoil_step2)

            # 固定延迟
            time.sleep(self.recoil_delay_ms / 1000)

    def start_recoil(self):
        """启动压枪控制（事件驱动模式，无线程创建开销）"""
        with self.recoil_lock:
            if self.recoil_active:
                return
            self.recoil_active = True

        # 确保常驻线程已启动
        if not self._recoil_worker_started:
            with self.recoil_lock:
                if not self._recoil_worker_started:
                    self._recoil_worker_started = True
                    self.recoil_thread = threading.Thread(target=self._internal_recoil_control, daemon=True)
                    self.recoil_thread.start()

        # 设置Event启用压枪（无阻塞）
        self.recoil_event.set()

    def stop_recoil(self):
        """停止压枪控制（无join，无阻塞）"""
        with self.recoil_lock:
            if not self.recoil_active:
                return
            self.recoil_active = False

        # 清除Event立即停止（无阻塞）
        self.recoil_event.clear()

    def close(self):
        self.stop_recoil()
        try:
            self.running = False
            if self.move_worker and self.move_worker.is_alive():
                self.move_worker.join(timeout=0.2)
            if self.controller is not None:
                self.controller.disconnect()
        except Exception:
            pass
