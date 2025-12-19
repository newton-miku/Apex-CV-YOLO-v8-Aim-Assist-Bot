import time

import win32api
import bettercam
# import dxcam

region = None
camera = None


def capture_init(args):
    global x, y, region, camera
    screen_width = win32api.GetSystemMetrics(0)
    screen_height = win32api.GetSystemMetrics(1)
    crop_height = int(screen_height * args.crop_size)
    crop_width = int(crop_height * (screen_width / screen_height))
    x = (screen_width - crop_height) // 2
    y = (screen_height - crop_height) // 2
    region = (x, y, x + crop_height, y + crop_height)
    camera = bettercam.create(region=region)
    camera.start(region=region,video_mode=True,target_fps=args.fps)


def take_shot(args):
    global region, camera
    return camera.get_latest_frame()
    img = None
    while img is None:
        img = camera.grab(region=camera.region)
    return img

def capture_benchmark():
    screen_width = win32api.GetSystemMetrics(0)
    screen_height = win32api.GetSystemMetrics(1)
    crop_height = int(640)
    x = (screen_width - crop_height) // 2
    y = (screen_height - crop_height) // 2
    region = (x, y, x + crop_height, y + crop_height)
    start_time, fps = time.perf_counter(), 0
    camera = bettercam.create(region=region,max_buffer_len=512,output_idx=0)
    while fps < 1000:
        frame = camera.grab()
        if frame is not None:  # New frame
            fps += 1
    end_time = time.perf_counter() - start_time
    print(f"帧率: {fps / end_time}，{end_time:.2f}s")

if __name__ == "__main__":
    capture_benchmark()
