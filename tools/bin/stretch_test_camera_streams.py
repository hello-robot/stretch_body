import cv2
import pyrealsense2 as rs
import numpy as np
from threading import Thread
import time
import sys
from stretch_body import gamepad_teleop
import os

D405_COLOR_SIZE = [640, 480]
D405_DEPTH_SIZE = [640, 480]
D405_FPS = 15

D435I_COLOR_SIZE = [640, 480]
D435I_DEPTH_SIZE = [640, 480]
D435I_FPS = 30

UVC_COLOR_SIZE = [1280, 720] # [3840,2880] [1920, 1080] [1280, 720] [640, 480]
UVC_FPS = 30

UVC_VIDEO_INDEX = 6
UVC_VIDEO_FORMAT = 'MJPG' # MJPG YUYV

# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# 
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/ 
# 
# Setting Video formates using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2


# Set video format
cmd = f"v4l2-ctl --device /dev/video{UVC_VIDEO_INDEX} --set-fmt-video=pixelformat={UVC_VIDEO_FORMAT},width={UVC_COLOR_SIZE[0]},height={UVC_COLOR_SIZE[1]}"
os.system(cmd)


realsense_ctx = rs.context() 
connected_devices = {}

for i in range(len(realsense_ctx.devices)):
    camera_name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
    camera_serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    connected_devices[camera_name] = camera_serial


d405_serial = connected_devices['Intel RealSense D405']
d435i_serial = connected_devices['Intel RealSense D435I']




stop_stream = False

color_image_d405 = None
depth_image_d405=None
color_image_d435i=None
depth_image_d435i=None
image_uvc = None

def setup_realsense_camera(serial_number, color_size, depth_size, fps):
    pipeline = rs.pipeline()
    config = rs.config()

    if serial_number:
        config.enable_device(serial_number)

    config.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)

    profile = pipeline.start(config)
    return pipeline

def setup_uvc_camera(device_index, size, fps):
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def d405_stream():
    global stop_stream, color_image_d405, depth_image_d405
    pipeline_d405 = setup_realsense_camera(serial_number=d405_serial,
                                           color_size=D405_COLOR_SIZE,
                                            depth_size=D405_DEPTH_SIZE,
                                            fps=D405_FPS)
    while not stop_stream:
        try:
            frames_d405 = pipeline_d405.wait_for_frames()
            color_frame_d405 = frames_d405.get_color_frame()
            color_image_d405 = np.asanyarray(color_frame_d405.get_data())
            depth_frame_d405 = frames_d405.get_depth_frame()
            depth_image_d405 = np.asanyarray(depth_frame_d405.get_data())
        except Exception as e:
            print(f"Error D405: {e}")
    pipeline_d405.stop()

def d435i_stream():
    global stop_stream, color_image_d435i, depth_image_d435i
    pipeline_d435i = setup_realsense_camera(serial_number=d435i_serial,
                                            color_size=D435I_COLOR_SIZE,
                                            depth_size=D435I_DEPTH_SIZE,
                                            fps=D435I_FPS)
    while not stop_stream:
        try:
            frames_d435i = pipeline_d435i.wait_for_frames()
            color_frame_d435i = frames_d435i.get_color_frame()
            color_image_d435i = np.asanyarray(color_frame_d435i.get_data())
            depth_frame_d435i = frames_d435i.get_depth_frame()
            depth_image_d435i = np.asanyarray(depth_frame_d435i.get_data())
        except Exception as e:
            print(f"Error D435i: {e}")
    pipeline_d435i.stop()

def uvc_cam_stream():
    global stop_stream, image_uvc
    uvc_camera = setup_uvc_camera(UVC_VIDEO_INDEX, UVC_COLOR_SIZE, UVC_FPS)
    
    while not stop_stream:
        try:
            ret, image_uvc = uvc_camera.read()
        except Exception as e:
            print(f"Error UVC Cam: {e}")
    uvc_camera.release()

def stream_cameras_thread(config):
    global stop_stream, color_image_d405, depth_image_d405, color_image_d435i, depth_image_d435i, image_uvc

    teleop = gamepad_teleop.GamePadTeleop()
    teleop.startup()
    last_rb_push_ts = time.time()

    d405=config['d405']
    d435i=config['d435i']
    uvc=config['uvc']
    if d405:
        d405_thread = Thread(target=d405_stream)
        d405_thread.start()

    if d435i:
        d435i_thread = Thread(target=d435i_stream)
        d435i_thread.start()
    
    if uvc:
        uvc_thread = Thread(target=uvc_cam_stream)
        uvc_thread.start()

    time.sleep(3)

    while True:
        if color_image_d405 is not None:
            cv2.imshow('D405 Color', color_image_d405)
        if depth_image_d405 is not None:
            cv2.imshow('D405 Depth', depth_image_d405)
        if color_image_d435i is not None:
            cv2.imshow('D435i Color', color_image_d435i)
        if depth_image_d435i is not None:
            cv2.imshow('D435i Depth', depth_image_d435i)
        if image_uvc is not None:
            cv2.imshow('UVC Head Camera', image_uvc)

        # run gamepad teleop at 1/30 sleep cycle
        if time.time() - last_rb_push_ts >=1/30:
            teleop.do_motion()
            teleop.robot.push_command()
            last_rb_push_ts = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Release resources
            stop_stream = True
            time.sleep(0.2)
            d405_thread.join()
            d435i_thread.join()
            uvc_thread.join()
            cv2.destroyAllWindows()
            teleop.gamepad_controller.stop()
            teleop.robot.stop()
            break

if __name__ == "__main__":
    config = {'d405' : False,
              'd435i' : False,
              'uvc' : False}
    if len(sys.argv) >= 2:
        for i in range(1,len(sys.argv)):
            user_input = sys.argv[i]
            config[user_input] = True
        stream_cameras_thread(config)
    else:
        config = {'d405' : True,
                'd435i' : True,
                'uvc' : True}
        stream_cameras_thread(config)
        

    
