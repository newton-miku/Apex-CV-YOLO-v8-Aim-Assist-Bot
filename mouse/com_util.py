import time
import serial
import serial.tools.list_ports
from typing import Optional, Union


class SerialManager:
    """
    串口管理类，用于处理串口的连接、数据发送、重连等操作
    """

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 1.0):
        """
        初始化串口管理器

        Args:
            port: 串口名称，如'COM8'、'/dev/ttyUSB0'，为None时自动检测
            baudrate: 波特率，默认115200
            timeout: 串口超时时间，默认1秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None  # 串口对象

    def find_serial_port(self) -> Optional[str]:
        """
        自动查找可用的串口设备，返回第一个找到的串口名称

        Returns:
            可用串口名称，无可用串口时返回None
        """
        try:
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
            return None
        except Exception as e:
            print(f"扫描串口设备时出错: {e}")
            return None

    def init_serial(self) -> bool:
        """
        初始化串口连接（支持自动检测端口）

        Returns:
            连接成功返回True，失败返回False
        """
        # 如果未指定端口，自动查找
        if self.port is None:
            print("正在自动检测串口设备...")
            self.port = self.find_serial_port()
            if self.port is None:
                print("未找到可用的串口设备")
                return False
            print(f"找到串口设备: {self.port}")

        # 关闭已存在的连接
        self.close_serial()

        # 新建串口连接
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                write_timeout=self.timeout
            )
            print(f"串口 {self.port} 连接成功（波特率：{self.baudrate}）")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败：串口访问异常 - {e}")
        except ValueError as e:
            print(f"串口连接失败：参数无效 - {e}")
        except Exception as e:
            print(f"串口连接失败：未知错误 - {e}")
        return False

    def send_to_com(self, command: Union[str, bytes], encoding: str = 'utf-8') -> bool:
        """
        向串口发送数据

        Args:
            command: 要发送的数据，字符串或字节流
            encoding: 字符串编码格式，默认utf-8（仅当command为字符串时生效）

        Returns:
            发送成功返回True，失败返回False
        """
        # 先检查并修复连接
        if not self.check_serial_connection():
            print("串口连接异常，无法发送数据")
            return False

        try:
            # 处理数据类型，统一转为字节流
            if isinstance(command, str):
                data = command.encode(encoding)
            else:
                data = command

            self.ser.write(data)  # type: ignore
            return True
        except serial.SerialException as e:
            print(f"串口数据发送失败：串口通信异常 - {e}")
            # 修改了判断方式，正确处理异常对象
            if "Write timeout" not in str(e):
                self.reconnect_serial()
        except UnicodeEncodeError as e:
            print(f"串口数据发送失败：字符串编码错误 - {e}")
        except Exception as e:
            print(f"串口数据发送失败：未知错误 - {e}")
        return False

    def reconnect_serial(self) -> bool:
        """
        重新连接串口（使用当前配置的端口和波特率）

        Returns:
            重连成功返回True，失败返回False
        """
        print(f"正在重新连接串口 {self.port}...")
        return self.init_serial()

    def check_serial_connection(self) -> bool:
        """
        检查串口连接状态，若断开则尝试重连

        Returns:
            连接正常（或重连成功）返回True，否则返回False
        """ 
        try:
            if self.ser is None or not self.ser.is_open:
                print("检测到串口断开连接")
                return self.reconnect_serial()
            return True
        except Exception as e:
            print(f"检查串口连接时出错：{e}")
            return self.reconnect_serial()

    def close_serial(self) -> None:
        """
        关闭串口连接
        """
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                print(f"串口 {self.port} 已关闭")
            except Exception as e:
                print(f"关闭串口时出错：{e}")
        self.ser = None


# 示例使用
if __name__ == "__main__":
    # 方式1：自动检测串口
    serial_manager = SerialManager(baudrate=115200)
    # 方式2：指定串口
    # serial_manager = SerialManager(port='COM8', baudrate=115200)

    # 初始化串口
    if serial_manager.init_serial():
        # 发送数据
        serial_manager.send_to_com("km.version()\r\n")
        # serial_manager.send_to_com("km.move(50,50,10)\r\n")
        # time.sleep(3)
        # # 键盘命令仅3.9以上版本可用
        # serial_manager.send_to_com("km.string(\"Hello\")\r\n")
        # time.sleep(2)
        # 关闭串口（可选，程序结束时建议关闭）
        serial_manager.close_serial()
    else:
        print("串口初始化失败") 