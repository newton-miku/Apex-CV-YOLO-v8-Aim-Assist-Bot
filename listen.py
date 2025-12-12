import numpy as np
import win32api
import win32gui
import win32print
import win32con
from pynput import keyboard, mouse
import winsound
import time
import serial
import serial.tools.list_ports
from threading import Thread, Event

# from mouse.makcu_mouse import internal_recoil_control, makcu_mouse_move,recoil_active

detecting = False
listening = True
shift_pressed = False
mouse1_pressed = False
mouse2_pressed = False
left_lock = False  # lock on target when the left mouse button is pressed  # 左键锁, Left, 按鼠标左键时锁
right_lock = False  # lock when pressing the right mouse button (scoping)  # 右键锁, Right, 按鼠标右键(开镜)时锁
auto_fire = False
time_fire = time.time()
backforce = 0
screen_size = np.array([win32api.GetSystemMetrics(0), win32api.GetSystemMetrics(1)])
screen_center = screen_size / 2
destination = screen_center
last = destination
width = 0
hDC = win32gui.GetDC(0)
scale = win32print.GetDeviceCaps(hDC, win32con.LOGPIXELSX) / 96
pre_error = integral = np.array([0., 0.])

# 新增：用于内部压枪控制的变量
enable_recoil = True
recoil_status = False
recoil_thread = None
recoil_event = None

ser = None
# 串口配置
SERIAL_PORT = 'COM8'
SERIAL_BAUDRATE = 115200

def find_serial_port():
    """
    自动查找可用的串口设备
    """
    ports = serial.tools.list_ports.comports()
    if ports:
        # 返回第一个找到的串口设备
        return ports[0].device
    return None

def init_serial(port=None, baudrate=115200):
    """
    初始化串口连接
    如果未指定端口，则自动检测可用端口
    """
    global ser, SERIAL_PORT, SERIAL_BAUDRATE
    
    # 如果没有指定端口，则自动查找
    if port is None:
        print("正在自动检测串口设备...")
        port = find_serial_port()
        if port is None:
            print("未找到可用的串口设备")
            return False
        print(f"找到串口设备: {port}")
    
    SERIAL_PORT = port
    SERIAL_BAUDRATE = baudrate
    
    try:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"串口 {port} 连接成功")
        return True
    except Exception as e:
        print(f"串口连接失败: {e}")
        return False
def reconnect_serial():
    """
    重新连接串口
    """
    global ser, SERIAL_PORT, SERIAL_BAUDRATE
    try:
        if ser and ser.is_open:
            ser.close()
    except:
        pass
    return init_serial(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)

def check_serial_connection():
    """
    检查串口连接状态，如果断开则尝试重连
    """
    global ser
    try:
        if ser is None or not ser.is_open:
            print("检测到串口断开，正在尝试重连...")
            return reconnect_serial()
    except Exception as e:
        print(f"串口连接异常: {e}")
        return reconnect_serial()
    return True

recoil_active = False
def makcu_mouse_move(dx, dy):
    """
    通过串口发送移动指令到外部硬件
    格式: "km.move(dx,dy)\r\n"
    """
    global ser  
    # 简化错误处理，避免复杂的重试机制
    if ser is None or not ser.is_open:
        # 不立即尝试重连，而是直接返回失败
        return False
        
    command = f"km.move({int(dx)},{int(dy)})\r\n"
    try:
        ser.write(command.encode())
        return True
    except Exception as e:
        check_serial_connection()
        return False

# 新增：内部压枪函数
def internal_recoil_control():
    """
    内部压枪控制函数，在独立线程中运行
    按照指定模式：先发送km.move(6,-8)，等待约4ms，然后发送km.move(-6,6)
    """
    global recoil_active, ser
    wait_time = 4
    while recoil_active:
        # 检查串口连接
        if not check_serial_connection():
            time.sleep(0.004)  # 4ms延迟
            continue
            
        if ser and ser.is_open:
            # 第一步：发送 km.move(6,-8)
            dx1=-2
            dy1=6
            command1 = f"km.move({dx1},{dy1})\r\n"
            makcu_mouse_move(dx1,dy1)
            # try:
            #     ser.write(command1.encode())
            # except Exception as e:
            #     print(f"内部压枪发送失败: {e}")
            #     # 尝试重新连接
            #     check_serial_connection()
            #     continue
                
            # 等待约4ms
            time.sleep(wait_time/1000)
            
            # 检查是否还需要继续（可能在等待期间已停用）
            if not recoil_active:
                break
                
            # 第二步：发送 km.move(-6,6)
            dx2=2
            dy2=-4
            command2 = f"km.move({dx2},{dy2})\r\n"
            makcu_mouse_move(dx2,dy2)
            # try:
            #     ser.write(command2.encode())
            # except Exception as e:
            #     print(f"内部压枪发送失败: {e}")
            #     # 尝试重新连接
            #     check_serial_connection()
                
        # 总周期约为8ms（两个步骤各4ms）
        time.sleep(wait_time/1000)

def start_internal_recoil():
    """
    启动内部压枪功能
    """
    global recoil_thread, recoil_event, recoil_active
    if not recoil_active and enable_recoil:
        recoil_active = True
        recoil_thread = Thread(target=internal_recoil_control, daemon=True)
        recoil_thread.start()
        print("内部压枪已启动")

def stop_internal_recoil():
    """
    停止内部压枪功能
    """
    global recoil_active
    if recoil_active:
        recoil_active = False
        print("内部压枪已停止")

# def send_to_hardware(dx, dy):
#     """
#     通过串口发送移动指令到外部硬件
#     格式: "m dx:dy\n"
#     """
#     global ser
#     # 检查串口连接
#     if not check_serial_connection():
#         return False
        
#     if ser and ser.is_open:
#         command = f"m{int(dx)}:{int(dy)}  \n"
#         try:
#             ser.write(command.encode())
#             # print(f"m{int(dx)}:{int(dy)}")
#             return True
#         except Exception as e:
#             print(f"串口发送失败: {e}")
#             # 尝试重新连接
#             if not check_serial_connection():
#                 return False
#             # 重新发送
#             try:
#                 ser.write(command.encode())
#                 return True
#             except Exception as e2:
#                 print(f"重新发送失败: {e2}")
#                 return False
#     return False

def send_click_command():
    """
    发送点击命令到外部硬件
    格式: "m10:50x\n"
    """
    pass
    # global ser
    # # 检查串口连接
    # if not check_serial_connection():
    #     return False
        
    # if ser and ser.is_open:
    #     try:
    #         ser.write(b"m10:50x\n")
    #         return True
    #     except Exception as e:
    #         print(f"发送点击命令失败: {e}")
    #         # 尝试重新连接
    #         if not check_serial_connection():
    #             return False
    #         # 重新发送
    #         try:
    #             ser.write(b"m10:50x\n")
    #             return True
    #         except Exception as e2:
    #             print(f"重新发送失败: {e2}")
    #             return False

# 新增函数：发送字符到串口
def send_char_to_serial(char):
    """
    发送单个字符到串口
    """
    global ser
    return
    # 检查串口连接
    if not check_serial_connection():
        return False
        
    if ser and ser.is_open:
        try:
            ser.write(char.encode())
            return True
        except Exception as e:
            print(f"发送字符 {char} 失败: {e}")
            # 尝试重新连接
            if not check_serial_connection():
                return False
            # 重新发送
            try:
                ser.write(char.encode())
                return True
            except Exception as e2:
                print(f"重新发送失败: {e2}")
                return False
    return False
def listen_init(args):
    global caps_lock
    caps_lock = args.caps_lock


def get_D_L():
    global detecting, listening, left_lock, caps_lock
    if caps_lock:
        if win32api.GetKeyState(0x14):
            if not left_lock:
                detecting = False
                left_lock = True
                winsound.Beep(800, 100)
        else:
            if left_lock:
                detecting = False
                left_lock = False
                winsound.Beep(400, 100)
    return detecting, listening


def listen_k_press(key):
    global detecting, listening, shift_pressed, left_lock, right_lock, auto_fire, caps_lock, enable_recoil
    if key == keyboard.Key.home:
        detecting = False
        listening = False
        print("listeners stop")
        winsound.Beep(700, 100)
        winsound.Beep(600, 100)
        return False
    # if key == keyboard.Key.shift:
    #     shift_pressed = True
    #     if not detecting:
    #         detecting = True
    #         print("Start detection: ", detecting)
    # if key == keyboard.Key.left:
    #     detecting = False
    #     left_lock = not left_lock
    #     winsound.Beep(800 if left_lock else 400, 200)
    if key == keyboard.Key.right:
        # detecting = False
        # right_lock = not right_lock
        # winsound.Beep(900 if right_lock else 500, 200)
        enable_recoil = not enable_recoil
        winsound.Beep(900 if enable_recoil else 500, 200)
    # if key == keyboard.Key.up:  # AUTO_FIRE is detected by EAC
    #     detecting = False
    #     auto_fire = not auto_fire
    #     winsound.Beep(850 if auto_fire else 450, 200)
    if not caps_lock:
        if key == keyboard.KeyCode.from_char('1') or key == keyboard.KeyCode.from_char('2'):
            if not left_lock:
                detecting = False
                left_lock = True
                winsound.Beep(800, 100)
        if key == keyboard.KeyCode.from_char('g'):
            if left_lock:
                detecting = False
                left_lock = False
                winsound.Beep(400, 100)


def listen_k_release(key):
    global detecting, shift_pressed, mouse1_pressed, mouse2_pressed, left_lock, right_lock
    if key == keyboard.Key.shift:
        shift_pressed = False
        if not (left_lock and mouse1_pressed) and not (right_lock and mouse2_pressed):
            detecting = False
            print("Start detection: ", detecting)


def listen_m_click(x, y, button, pressed):
    global detecting, shift_pressed, mouse1_pressed, mouse2_pressed, left_lock, right_lock, backforce,recoil_status
    # if button == mouse.Button.left and left_lock:
    #     if pressed:
    #         backforce = 3
    #         mouse1_pressed = True
    #         if not detecting:
    #         detecting = True
    #         print("Start detection: ", detecting)
    #     else:
    #         backforce = 0
    #         mouse1_pressed = False
    #         if not shift_pressed and (not right_lock or (right_lock and not mouse2_pressed)):
    #             detecting = False
    #             print("Start detection: ", detecting)
    if button == mouse.Button.left or button == mouse.Button.right:
        # 检查是否同时按下左右键且Caps Lock启用
        is_caps_lock_on = win32api.GetKeyState(0x14) & 1
        if pressed:
            if button == mouse.Button.left:
                mouse1_pressed = True
            elif button == mouse.Button.right:
                mouse2_pressed = True
                
            # 检查是否左右键都按下且Caps Lock启用
            if mouse1_pressed and mouse2_pressed and is_caps_lock_on:
                recoil_status =True
                # 替换发送's'为启动内部压枪
                start_internal_recoil()
            elif recoil_status == True:
                recoil_status = False
                # 替换发送'x'为停止内部压枪
                stop_internal_recoil()
                
            if left_lock:
                backforce = 6
                if not detecting:
                    detecting = True
                    print("Start detection: ", detecting)
        else:
            if button == mouse.Button.left:
                mouse1_pressed = False
            elif button == mouse.Button.right:
                mouse2_pressed = False
                
            # 当释放按键时，如果Caps Lock启用，则停止内部压枪
            # if (mouse1_pressed and mouse2_pressed):
            if (mouse1_pressed or mouse2_pressed) and recoil_status == True:
                recoil_status = False
                # 替换发送'x'为停止内部压枪
                stop_internal_recoil()
                
            if left_lock:
                backforce = 0
                if not shift_pressed and (not right_lock or (right_lock and not mouse2_pressed)):
                    detecting = False
                    print("Start detection: ", detecting)
    # if button == mouse.Button.right and right_lock:
    #     if pressed:
    #         mouse2_pressed = True
    #         if not detecting:
    #             detecting = True
    #             print("Start detection: ", detecting)
    #     else:
    #         mouse2_pressed = False
    #         if not shift_pressed and (not left_lock or (left_lock and not mouse1_pressed)):
    #             detecting = False
    #             print("Start detection: ", detecting)
    if button == mouse.Button.right:
        if pressed:
            mouse2_pressed = True
            if right_lock:
                if not detecting:
                    detecting = True
                    print("Start detection: ", detecting)
        else:
            mouse2_pressed = False
            if right_lock:
                if not shift_pressed and (not left_lock or (left_lock and not mouse1_pressed)):
                    detecting = False
                    print("Start detection: ", detecting)



def PID(args, error):
    global integral, pre_error, backforce
    integral += error
    derivative = error - pre_error
    pre_error = error
    output = args.Kp*error + args.Ki*integral + args.Kd*derivative
    output[1] += backforce
    return output.astype(int)


def move_mouse(args):
    global detecting, screen_center, destination, last, width, scale, pre_error, integral, pos
    global auto_fire, time_fire, shift_pressed, right_lock, mouse2_pressed, mouse1_pressed, ser
    if 1:
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
        if norm <= args.aim_deadzone or (destination[0] == screen_center[0] and destination[1] == screen_center[1]): return
        if norm > width*args.aim_fov: return
        if args.pid:
            move = PID(args, mouse_vector)
            # 使用串口发送移动指令替代win32api.mouse_event
            makcu_mouse_move(move[0], move[1] / 2)
            last_mv = last - destination + mouse_vector
            if not auto_fire or time.time()-time_fire <= 0.0625: return  # 125ms
            # norm <= width/2  # higher divisor increases precision but limits fire rate
            # move[0]*last_mv[0] >= 0  # ensures tracking
            if ( shift_pressed and not right_lock and mouse2_pressed and not mouse1_pressed  # scope fire
            and norm <= width*2/3 and move[0]*last_mv[0] >= 0 ):
                # 发送点击命令到外部硬件
                # send_click_command()
                time_fire = time.time()
            elif ( ((shift_pressed and not mouse2_pressed) or (right_lock and mouse2_pressed and not mouse1_pressed))  # hip fire
            and norm <= width*3/4 and move[0]*last_mv[0] >= 0 ):
                # 发送点击命令到外部硬件
                # send_click_command()
                time_fire = time.time()
            return
        # 非PID模式也使用串口控制
        if norm <= width*4/3:
            makcu_mouse_move(mouse_vector[0]*0.5, mouse_vector[1]*0.6)
            return
    else:
        pre_error = integral = np.array([0., 0.])


# redirect the mouse closer to the nearest box center
def mouse_redirection(args, boxes):
    global screen_size, screen_center, destination, last, width, pos
    if boxes.shape[0] == 0:
        width = -1
        last = destination
        destination = np.array([-1, -1])
        return
    # pos = np.array(win32api.GetCursorPos())  # GetCursorPos is monitored by BattlEye
    pos = np.array(mouse.Controller().position)

    # get the center of the boxes
    boxes_center = (
        (boxes[:, :2] + boxes[:, 2:]) / 2
    )
    boxes_center[:, 1] = (
        # boxes[:, 1] * 0.6 + boxes[:, 3] * 0.4  # torso
        boxes[:, 1] * 0.7 + boxes[:, 3] * 0.3  # chest
        # boxes[:, 1] * 0.85 + boxes[:, 3] * 0.2  # head
    )

    # map the box from the image coordinate to the screen coordinate
    start_point = screen_center - screen_size[1] * args.crop_size / 2
    start_point = list(map(int, start_point))
    boxes_center[:, 0] = boxes_center[:, 0] + start_point[0]
    boxes_center[:, 1] = boxes_center[:, 1] + start_point[1]

    # find the nearest box center
    dis = np.linalg.norm(boxes_center - pos, axis=-1)
    min_index = np.argmin(dis)
    width = boxes[min_index, 2] - boxes[min_index, 0]
    last = destination
    destination = boxes_center[np.argmin(dis)].astype(int)
    # print(destination)