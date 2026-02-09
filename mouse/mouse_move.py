import serial
import serial.tools.list_ports

from mouse.com_util import init_serial,ser

# 在文件开头添加全局变量
def send_to_hardware(dx, dy):
    """
    通过串口发送移动指令到外部硬件
    格式: "MOVE dx dy\n"
    """
    global ser
    if ser and ser.is_open:
        command = f"km.move({int(dx)},{int(dy)})\n"
        try:
            ser.write(command.encode())
            return True
        except Exception as e:
            print(f"串口发送失败: {e}")
            return False
    return False
# def send_to_hardware(dx, dy):
#     """
#     通过串口发送移动指令到外部硬件
#     格式: "MOVE dx dy\n"
#     """
#     global ser
#     if ser and ser.is_open:
#         command = f"m{int(dx)}:{int(dy)}\n"
#         try:
#             ser.write(command.encode())
#             return True
#         except Exception as e:
#             print(f"串口发送失败: {e}")
#             return False
#     return False

def move_mouse(args):
    global detecting, screen_center, destination, last, width, scale, pre_error, integral, pos
    global auto_fire, time_fire, shift_pressed, right_lock, mouse2_pressed, mouse1_pressed
    if detecting:
        if destination[0] == -1:
            if last[0] == -1:
                pre_error = integral = np.array([0., 0.])
                return
            else:
                mouse_vector = np.array([0, 0])
        else:
            mouse_vector = (destination - pos) / scale
        norm = np.linalg.norm(mouse_vector)
        # if destination not in region
        if norm <= args.aim_deadzone or (destination[0] == screen_center[0] and destination[1] == screen_center[1]): 
            return
        if norm > width*args.aim_fov: 
            return
            
        if args.pid:
            move = PID(args, mouse_vector)
            # 替换 win32api.mouse_event 为串口发送
            send_to_hardware(move[0], move[1] / 2)
            
            last_mv = last - destination + mouse_vector
            if not auto_fire or time.time()-time_fire <= 0.125: 
                return  # 125ms
            # norm <= width/2  # higher divisor increases precision but limits fire rate
            # move[0]*last_mv[0] >= 0  # ensures tracking
            if ( shift_pressed and not right_lock and mouse2_pressed and not mouse1_pressed  # scope fire
            and norm <= width*2/3 and move[0]*last_mv[0] >= 0 ):
                # 这里可以发送点击命令到硬件
                # send_click_command()
                time_fire = time.time()
            elif ( ((shift_pressed and not mouse2_pressed) or (right_lock and mouse2_pressed and not mouse1_pressed))  # hip fire
            and norm <= width*3/4 and move[0]*last_mv[0] >= 0 ):
                # 这里可以发送点击命令到硬件
                # send_click_command()
                time_fire = time.time()
            return
        # 非PID模式也替换为串口控制
        if norm <= width*4/3:
            send_to_hardware(mouse_vector[0] / 3, mouse_vector[1] / 3)
            return
    else:
        pre_error = integral = np.array([0., 0.])

if __name__ == "__main__":
    import time
    import argparse
    import numpy as np
    
    # 模拟必要的全局变量
    global detecting, screen_center, destination, last, width, scale, pre_error, integral, pos
    global auto_fire, time_fire, shift_pressed, right_lock, mouse2_pressed, mouse1_pressed
    
    # 初始化默认值
    detecting = True
    screen_center = np.array([960, 540])  # 假设1920x1080屏幕
    destination = np.array([1000, 500])
    last = np.array([960, 540])
    width = 200
    scale = 1.0
    pre_error = np.array([0.0, 0.0])
    integral = np.array([0.0, 0.0])
    pos = np.array([960, 540])
    auto_fire = False
    time_fire = time.time()
    shift_pressed = False
    right_lock = False
    mouse2_pressed = False
    mouse1_pressed = False
    
    # 创建一个简单的args模拟对象
    class Args:
        def __init__(self):
            self.aim_deadzone = 5
            self.aim_fov = 0.5
            self.pid = True
            self.Kp = 0.5
            self.Ki = 0.1
            self.Kd = 0.05
            self.crop_size = 0.5
    
    def PID(args, error):
        global integral, pre_error
        integral += error
        derivative = error - pre_error
        pre_error = error
        output = args.Kp*error + args.Ki*integral + args.Kd*derivative
        return output.astype(int)
    
    # 将PID函数添加到全局命名空间中
    import sys
    sys.modules[__name__].PID = PID
    
    # 获取串口端口列表
    ports = serial.tools.list_ports.comports()
    print("可用的串口:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    
    if not ports:
        print("未找到可用的串口")
        exit(1)
    
    # 选择第一个串口进行测试
    selected_port = ports[0].device
    print(f"\n使用串口: {selected_port}")
    
    # 初始化串口连接
    if init_serial(selected_port, 115200):
        print("串口连接成功，开始测试...")
        
        # 测试发送移动命令
        print("测试1: 发送移动命令")
        for i in range(5):
            result = send_to_hardware(10 + i*5, -5 - i*3)
            print(f"发送移动命令 {i+1}: dx={10 + i*5}, dy={-5 - i*3}, 结果: {'成功' if result else '失败'}")
            time.sleep(0.5)
        
        print("\n测试完成")
    else:
        print("串口连接失败，无法进行测试")