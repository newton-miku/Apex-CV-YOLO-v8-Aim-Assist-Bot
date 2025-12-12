import time
import threading
from typing import Optional
# 假设SerialManager在mouse.com_util中，根据实际路径调整
from mouse.com_util import SerialManager
import args_

class MakcuMouseController:
    """
    makcu鼠标控制器，用于通过串口发送移动指令和实现压枪控制功能
    """
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 1.0, handlike: bool = False):
        """
        初始化makcu鼠标控制器

        Args:
            port: 串口名称（如COM8），为None时自动检测
            baudrate: 串口波特率，默认115200
            timeout: 串口超时时间，默认1秒
        """
        # 初始化串口管理器
        self.serial_manager = SerialManager(port=port, baudrate=baudrate, timeout=timeout)
        # 压枪相关状态与配置
        self.recoil_active = False  # 压枪激活状态
        self.internal_hand_like = False # 是否启用硬件仿人手
        self.internal_hand_like_sep_time = 2 # 默认仿人手分段
        self.recoil_thread: Optional[threading.Thread] = None  # 压枪线程对象
        self.recoil_lock = threading.Lock()  # 线程锁，保证状态线程安全
        # 压枪参数配置（抽离为属性，便于调整）
        self.recoil_delay_ms = 4  # 步骤间延迟（毫秒）
        self.recoil_step1 = (-2, 6)  # 第一步：dx, dy
        self.recoil_step2 = (2, -4)  # 第二步：dx, dy

    def init_serial(self) -> bool:
        """
        初始化串口连接

        Returns:
            连接成功返回True，失败返回False
        """
        return self.serial_manager.init_serial()

    def send_mouse_move(self, dx: int, dy: int) -> bool:
        """
        向makcu鼠标发送移动指令，格式：km.move(dx,dy)\r\n
        当启用硬件仿人手时会自动分段移动

        Args:
            dx: 水平移动量
            dy: 垂直移动量

        Returns:
            发送成功返回True，失败返回False
        """
        # 构造指令字符串
        command = f"km.move({int(dx)},{int(dy)})\r\n"
        if not self.internal_hand_like:
            sep_time = self.internal_hand_like_sep_time
            command = f"km.move({int(dx)},{int(dy)},{sep_time})\r\n"
        # 使用SerialManager发送数据，内部已处理连接检查和重连
        return self.serial_manager.send_to_com(command)
    
    def _internal_mouse_move(self, dx, dy):
        """
        向makcu鼠标发送移动指令，格式：km.move(dx,dy)\r\n

        Args:
            dx: 水平移动量
            dy: 垂直移动量

        Returns:
            发送成功返回True，失败返回False
        """
        # 构造指令字符串
        command = f"km.move({int(dx)},{int(dy)})\r\n"
        # 使用SerialManager发送数据，内部已处理连接检查和重连
        return self.serial_manager.send_to_com(command)


    def _internal_recoil_control(self):
        """
        内部压枪控制函数，运行在独立线程中
        注意：外部不要直接调用，通过start_recoil/stop_recoil控制
        """
        while True:
            # 加锁检查压枪状态，确保线程安全
            with self.recoil_lock:
                if not self.recoil_active:
                    break

            # 第一步：发送第一个移动指令
            self._internal_mouse_move(*self.recoil_step1)
            # 等待指定毫秒（转换为秒）
            time.sleep(self.recoil_delay_ms / 1000)

            # 再次检查状态（避免等待期间被停止）
            with self.recoil_lock:
                if not self.recoil_active:
                    break

            # 第二步：发送第二个移动指令
            self._internal_mouse_move(*self.recoil_step2)
            # 等待指定毫秒，完成一个周期
            time.sleep(self.recoil_delay_ms / 1000)

    def start_recoil(self):
        """
        启动压枪控制（创建新线程运行压枪逻辑）
        注意：避免重复启动线程
        """
        with self.recoil_lock:
            if self.recoil_active:
                print("压枪功能已处于激活状态，无需重复启动")
                return
            self.recoil_active = True

        # 创建并启动线程（守护线程，主程序退出时自动结束）
        self.recoil_thread = threading.Thread(target=self._internal_recoil_control, daemon=True)
        self.recoil_thread.start()
        print("压枪功能已启动")

    def stop_recoil(self):
        """
        停止压枪控制
        """
        with self.recoil_lock:
            if not self.recoil_active:
                print("压枪功能已处于停止状态，无需重复停止")
                return
            self.recoil_active = False

        # 等待线程结束（可选，因线程是守护线程，也可忽略）
        if self.recoil_thread and self.recoil_thread.is_alive():
            self.recoil_thread.join(timeout=0.1)  # 超时100ms，避免阻塞
        print("压枪功能已停止")

    def close(self):
        """
        关闭串口连接，释放资源
        """
        # 先停止压枪
        self.stop_recoil()
        # 关闭串口
        self.serial_manager.close_serial()
        print("makcu鼠标控制器已关闭，资源已释放")

# 示例使用
if __name__ == "__main__":
    # 初始化控制器（指定串口，或设为None自动检测）
    controller = MakcuMouseController(baudrate=115200)

    # 初始化串口
    if not controller.init_serial():
        print("串口初始化失败，程序退出")
    else:
        try:
            # 测试单独发送移动指令
            controller.send_mouse_move(0, 0)  # 发送空移动指令

            # 启动压枪
            controller.start_recoil()

            # 运行5秒后停止压枪（模拟实际使用场景）
            time.sleep(5)
            controller.stop_recoil()

            # 可再次启动压枪（测试重复启动逻辑）
            time.sleep(2)
            controller.start_recoil()
            time.sleep(3)
            controller.stop_recoil()

        except KeyboardInterrupt:
            # 捕获Ctrl+C，优雅退出
            print("\n用户终止程序，正在清理资源...")
        finally:
            # 关闭控制器，释放资源
            controller.close()