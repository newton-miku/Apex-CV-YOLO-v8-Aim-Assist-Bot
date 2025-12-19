import argparse
import sys
import time
from threading import Thread

from pynput import keyboard, mouse

import capture
import listen
from args_ import *
from capture import *
from draw import show_target
from listen import listen_k_press, listen_k_release, listen_m_click, listen_init, get_D_L, mouse_redirection, move_mouse
import tempfile

##import cv2
##from datetime import datetime

# 导入MakcuMouseController
from mouse.makcu_mouse import MakcuMouseController

global detecting, listening

# 添加单实例检查
def is_already_running():
    lock_file_path = os.path.join(tempfile.gettempdir(), 'apex_aim_assist.lock')
    
    try:
        # 尝试以独占模式打开锁文件
        if sys.platform == 'win32':
            # Windows 系统
            import msvcrt
            lock_file = open(lock_file_path, 'w')
            msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            # Unix/Linux 系统
            import fcntl
            lock_file = open(lock_file_path, 'w')
            fcntl.lockf(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        
        # 保存文件对象引用，防止被垃圾回收
        is_already_running.lock_file = lock_file
        return False
    except IOError:
        # 文件已被锁定，说明程序已在运行
        return True
def listeners():
    keyboard_listener = keyboard.Listener(on_press=listen_k_press, on_release=listen_k_release)
    keyboard_listener.start()

    mouse_listener = mouse.Listener(on_click=listen_m_click)
    mouse_listener.start()

    keyboard_listener.join()


if __name__ == "__main__":
    # 检查是否已经运行
    if is_already_running():
        print("程序已经在运行中！")
        # os.system("pause")
        sys.exit(1)
    
    os.system("")
    print("\033[01;04;31m" + "A" + "\033[32m" + "N" + "\033[33m" + "S" + "\033[34m" + "I" + "\033[00m" + " enabled")
    # create an arg set
    listening = True
    print("listeners start")

    args = argparse.ArgumentParser()
    args = arg_init(args)
    listen_init(args)

    thread_1 = Thread(target=listeners,daemon=True)
    thread_1.start()
    print(thread_1)

    capture_init(args)
    if args.model[-3:] == ".pt" or args.model[-7:] == ".engine":
        from predict import *

        predict_init(args)
    else:
        from trt import trt_init, trt

        trt_init(args)
    print("main start")
    time_start = time.time()

    # 初始化MakcuMouseController并将其传递给listen模块
    handlike = False
    if not args.pid:    
        handlike = True
    mouse_controller = MakcuMouseController(baudrate=115200,handlike=handlike)
    if not mouse_controller.init_serial():
        print("串口初始化失败，程序退出")
        sys.exit(1)
    
    # 将mouse_controller设置到listen模块中供其使用
    listen.set_mouse_controller(mouse_controller)

    count = 0
    time_capture_total = 0
    avg_predict_time = 0
    try:
        while listening:

            detecting, listening = get_D_L()
            # take a screenshot
            time_shot = time.time()
            img = take_shot(args)
            time_capture = time.time()
            time_capture_total += time_capture - time_shot
            # print(f"shot time: {(time.time() - time_shot)*1000:.2f}ms")
            # predict the image
            time.sleep(args.wait)
            # 提前缓存后缀判断结果，避免每次循环都做字符串切片与比较
            if args.model.endswith(('.pt', '.engine')):
                predict_output = predict(args, img)
                # 直接一次性在GPU上完成过滤与格式转换，减少CPU-GPU往返
                tgt = predict_output.boxes.data[predict_output.boxes.cls == args.target_index, :4]
                boxes = tgt.cpu().numpy()        # 已是对应的xyxy格式，无需再调.xyxy
            else:
                boxes, scores, cls_inds = trt(args, img)
                # print(scores, cls_inds)
            time_predict = time.time()
            avg_predict_time += (time_predict - time_capture)
            # print("predict time: ", time.time() - time_capture)
            if detecting:
                if boxes.size != 0:
                    if args.draw_boxes:
                        for i in range(0, int(boxes.size/4)):
                            show_target([int(boxes[i,0])+capture.x, int(boxes[i,1])+capture.y, int(boxes[i,2])+capture.x, int(boxes[i,3])+capture.y])
                #   elif args.save_img and listen.shift_pressed:
                #       cv2.imwrite('screenshots/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f') + '.png', img)
                # print(boxes)
                mouse_redirection(args, boxes)
                move_mouse(args)
            # print(f"post-process time: {(time.time() - time_predict)*1000:.2f}")
            # print(f"total time: {(time.time() - time_shot)*1000:.3f}ms")
            count += 1

            if (count % 300 == 0):
                time_per_100frame = time.time() - time_start
                time_start = time.time()
                print(f"Screenshot fps: {count / time_per_100frame:.2f}", )
                print(f"fps: {count / time_per_100frame:.2f}")
                interval = time_per_100frame / count
                print(f"Avg predict time: {(avg_predict_time/count)*1000:.2f}ms", )
                print(f"interval: {interval*1000:.2f}ms", )
                print("[LEFT_LOCK]" if listen.left_lock else "[         ]", "[RIGHT_LOCK]" if listen.right_lock else "[          ]", "[\033[30;41mAUTO_FIRE\033[00m]" if listen.auto_fire else "[          ]","[\033[30;42mRecoil_CTL\033[00m]" if listen.enable_recoil else "[          ]")
                count = 0
                time_capture_total = 0
                avg_predict_time = 0
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        # 清理锁文件
        if hasattr(is_already_running, 'lock_file'):
            is_already_running.lock_file.close()
            lock_file_path = os.path.join(tempfile.gettempdir(), 'apex_aim_assist.lock')
            try:
                os.remove(lock_file_path)
            except:
                pass

    print("main stop")
