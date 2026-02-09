import argparse
import sys
import time
import os
import psutil
import numpy as np
from threading import Thread, Lock
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import cv2
import queue

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

web_state = {"jpeg": b"", "fps": 0.0, "last_update": 0.0}
CLASS_NAMES = {0: "All", 1: "Enemy", 2: "Tag"}

# JPEG编码线程相关
encode_queue = queue.Queue(maxsize=2)  # 最多缓存2帧
encode_lock = Lock()
encode_worker_running = True

def _encode_worker():
    """独立JPEG编码线程"""
    while encode_worker_running:
        try:
            # 获取帧，超时退出
            frame_data = encode_queue.get(timeout=0.1)
            if frame_data is None:
                break
            frame, boxes, labels_cls, labels_conf = frame_data

            # 颜色转换
            display_frame = frame
            if display_frame is not None:
                if len(display_frame.shape) == 3:
                    if display_frame.shape[2] == 4:
                        display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGRA2BGR)
                    elif display_frame.shape[2] == 3:
                        display_frame = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)

                # 绘制检测框
                if boxes is not None and boxes.size != 0:
                    for i in range(0, int(boxes.size/4)):
                        x1 = int(boxes[i,0]); y1 = int(boxes[i,1]); x2 = int(boxes[i,2]); y2 = int(boxes[i,3])
                        # Draw aim FOV circle around target center
                        if args.aim_fov > 0:
                            target_width = x2 - x1
                            center_x = x1 + target_width // 2
                            center_y = y1 + (y2 - y1) // 2
                            radius = int(target_width * args.aim_fov)
                            cv2.circle(display_frame, (center_x, center_y), radius, (0, 255, 255), 1)

                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0,255,0), 2)
                        if labels_cls is not None and labels_conf is not None:
                            try:
                                cls_id = int(labels_cls[i])
                            except Exception:
                                cls_id = -1
                            conf_v = float(labels_conf[i]) if i < len(labels_conf) else 0.0
                            name = CLASS_NAMES.get(cls_id, str(cls_id))
                            text = f"{name}:{conf_v*100:.1f}%"
                            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                            bx1, by1 = x1, max(0, y1 - th - 6)
                            bx2, by2 = x1 + tw + 8, y1
                            cv2.rectangle(display_frame, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
                            cv2.putText(display_frame, text, (x1 + 4, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                # 编码JPEG
                ok, buf = cv2.imencode(".jpg", display_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    with encode_lock:
                        web_state["jpeg"] = buf.tobytes()
                        web_state["last_update"] = time.time()
        except queue.Empty:
            continue
        except Exception:
            pass

class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/frame.jpg"):
            data = web_state["jpeg"]
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(data)
        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last = 0.0
            try:
                while True:
                    if web_state["last_update"] <= last or not web_state["jpeg"]:
                        time.sleep(0.05)
                        continue
                    last = web_state["last_update"]
                    buf = web_state["jpeg"]
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(buf)).encode("ascii") + b"\r\n\r\n")
                    self.wfile.write(buf)
                    self.wfile.write(b"\r\n")
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
            except Exception:
                pass
        elif self.path.startswith("/stats"):
            body = json.dumps({"fps": web_state["fps"]}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        else:
            html = (
                b"<html><head><meta charset='utf-8'><title>Apex CV Stream</title>"
                b"<style>"
                b"body{margin:0;background:#000;}"
                b"#hud{position:fixed;top:12px;left:12px;z-index:10;background:rgba(0,0,0,.45);padding:8px 12px;border-radius:8px;"
                b"color:#00ff66;font:700 20px/1.2 Consolas,monospace;text-shadow:0 0 8px #000,0 0 2px #000;}"
                b"#img{width:100%;height:auto;display:block;}"
                b"</style></head>"
                b"<body>"
                b"<div id='hud'>FPS: <span id='fps'>0</span></div>"
                b"<img id='img' src='/stream'/>"
                b"<script>"
                b"setInterval(()=>{fetch('/stats').then(r=>r.json()).then(d=>{document.getElementById('fps').innerText=d.fps.toFixed(2);});},500);"
                b"</script></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html)

def start_web_server():
    server = ThreadingHTTPServer(("", 8000), WebHandler)
    server.serve_forever()

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

def set_process_priority(priority="high"):
    """
    设置当前进程的优先级
    priority: "low", "normal", "high", "realtime"
    """
    try:
        # 获取当前进程ID
        pid = os.getpid()
        p = psutil.Process(pid)
        
        # Windows系统优先级映射
        priority_map = {
            "low": psutil.BELOW_NORMAL_PRIORITY_CLASS,
            "normal": psutil.NORMAL_PRIORITY_CLASS,
            "high": psutil.HIGH_PRIORITY_CLASS,
            "realtime": psutil.REALTIME_PRIORITY_CLASS
        }
        
        p.nice(priority_map[priority])
        print(f"进程优先级已设置为: {priority}")
    except Exception as e:
        print(f"设置优先级失败: {e}")

if __name__ == "__main__":
    # 检查是否已经运行
    if is_already_running():
        print("程序已经在运行中！")
        # os.system("pause")
        sys.exit(1)
    
    os.system("")
        # 优先设置为"high"，而非"realtime"
    set_process_priority("high")
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
    web_thread = Thread(target=start_web_server, daemon=True)
    web_thread.start()

    # 启动JPEG编码线程
    encode_thread = Thread(target=_encode_worker, daemon=True)
    encode_thread.start()

    count = 0
    time_capture_total = 0
    avg_predict_time = 0
    web_last_encode_time = 0.0
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
            labels_cls = None
            labels_conf = None
            if args.model.endswith(('.pt', '.engine')):
                predict_output = predict(args, img)

                # 优化：在GPU上完成过滤，减少CPU-GPU数据传输
                # 使用torch.where进行一次性布尔索引
                if predict_output is not None and hasattr(predict_output, 'boxes'):
                    boxes_tensor = predict_output.boxes
                    # 创建目标类别掩码
                    target_mask = boxes_tensor.cls == args.target_index

                    # 使用掩码一次性提取所有需要的数据
                    if target_mask.any():
                        boxes = boxes_tensor.xyxy[target_mask].cpu().numpy()
                        labels_cls = boxes_tensor.cls[target_mask].cpu().numpy()
                        labels_conf = boxes_tensor.conf[target_mask].cpu().numpy()
                    else:
                        # 没有检测到目标时返回空数组
                        boxes = np.empty((0, 4), dtype=np.float32)
                        labels_cls = np.array([], dtype=np.float32)
                        labels_conf = np.array([], dtype=np.float32)
                else:
                    boxes = np.empty((0, 4), dtype=np.float32)
            else:
                boxes, scores, cls_inds = trt(args, img)
                labels_cls = cls_inds
                labels_conf = scores
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
            loop_fps = 1.0 / max(1e-6, (time.time() - time_shot))
            web_state["fps"] = loop_fps

            # 将帧推送到编码队列（异步编码，不阻塞主循环）
            if (time.time() - web_last_encode_time) >= 0.1:
                try:
                    encode_queue.put_nowait((img, boxes, labels_cls, labels_conf))
                except queue.Full:
                    pass  # 队列满时丢弃旧帧
                web_last_encode_time = time.time()
            # print(f"post-process time: {(time.time() - time_predict)*1000:.2f}")
            # print(f"total time: {(time.time() - time_shot)*1000:.3f}ms")
            count += 1

            if (count % 300 == 0):
                time_per_100frame = time.time() - time_start
                time_start = time.time()
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
        # 停止编码线程
        encode_worker_running = False
        try:
            encode_queue.put_nowait(None)  # 发送停止信号
        except Exception:
            pass

        # 清理锁文件
        if hasattr(is_already_running, 'lock_file'):
            is_already_running.lock_file.close()
            lock_file_path = os.path.join(tempfile.gettempdir(), 'apex_aim_assist.lock')
            try:
                os.remove(lock_file_path)
            except:
                pass

    print("main stop")
