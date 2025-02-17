#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
import numpy as np
from threading import Thread, Lock
import time
import sys
import os
import stretch_body.hello_utils as hu
import argparse

D405_COLOR_SIZE = [640, 480]
D405_DEPTH_SIZE = [640, 480]
D405_FPS = 15

D435I_COLOR_SIZE = [640, 480]
D435I_DEPTH_SIZE = [640, 480]
D435I_FPS = 30

UVC_COLOR_SIZE = [1280, 720] # [3840,2880] [1920, 1080] [1280, 720] [640, 480]
UVC_FPS = 30

UVC_VIDEO_INDEX = '/dev/hello-nav-head-camera'
UVC_VIDEO_FORMAT = 'MJPG' # YUYV

# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# 
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/ 
# 
# Setting Video formates using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2


realsense_ctx = rs.context() 
connected_devices = {}

for i in range(len(realsense_ctx.devices)):
    camera_name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
    camera_serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    connected_devices[camera_name] = camera_serial


stop_stream = False
color_image_d405 = None
depth_image_d405=None
color_image_d435i=None
depth_image_d435i=None
image_uvc = None


def d405_stream():
    global stop_stream, color_image_d405, depth_image_d405, lock
    try:
        d405_serial = connected_devices['Intel RealSense D405']
    except KeyError:
        print("Unable to find Realsense D405...")
        return None
    
    print(f"\nD405 Stream Settings:\n D405_COLOR_SIZE={D405_COLOR_SIZE}\n D405_DEPTH_SIZE={D405_DEPTH_SIZE}\n FPS={D405_FPS}")
    pipeline_d405 = hu.setup_realsense_camera(serial_number=d405_serial,
                                           color_size=D405_COLOR_SIZE,
                                            depth_size=D405_DEPTH_SIZE,
                                            fps=D405_FPS)
    while not stop_stream:
        try:
            frames_d405 = pipeline_d405.wait_for_frames()
            color_frame_d405 = frames_d405.get_color_frame()
            depth_frame_d405 = frames_d405.get_depth_frame()
            color_image_d405 = np.asanyarray(color_frame_d405.get_data())
            depth_image_d405 = np.asanyarray(depth_frame_d405.get_data())

        except Exception as e:
            print(f"Error D405: {e}")
    pipeline_d405.stop()

def d435i_stream():
    global stop_stream, color_image_d435i, depth_image_d435i, lock
    try:
        d435i_serial = connected_devices['Intel RealSense D435I']
    except KeyError:
        print("Unable to find Realsense D435i...")
        return None
    print(f"D435i Stream Settings:\n D435I_COLOR_SIZE={D435I_COLOR_SIZE}\n D435I_DEPTH_SIZE={D435I_DEPTH_SIZE}\n FPS={D435I_FPS}")
    pipeline_d435i = hu.setup_realsense_camera(serial_number=d435i_serial,
                                            color_size=D435I_COLOR_SIZE,
                                            depth_size=D435I_DEPTH_SIZE,
                                            fps=D435I_FPS)
    while not stop_stream:
        try:
            frames_d435i = pipeline_d435i.wait_for_frames()
            color_frame_d435i = frames_d435i.get_color_frame()
            depth_frame_d435i = frames_d435i.get_depth_frame()
            color_image_d435i = np.asanyarray(color_frame_d435i.get_data())
            depth_image_d435i = np.asanyarray(depth_frame_d435i.get_data())
        except Exception as e:
            print(f"Error D435i: {e}")
    pipeline_d435i.stop()

def uvc_cam_stream(video_path=None):
    global stop_stream, image_uvc
    if video_path is None:
        if not os.path.exists('/dev/hello-nav-head-camera'):
            print("Unable to Find device: /dev/hello-nav-head-camera")
            return None
        print(f"Navigation Camera Stream Settings:\n UVC_COLOR_SIZE={UVC_COLOR_SIZE}\n FPS={UVC_FPS}")
        uvc_camera = hu.setup_uvc_camera(UVC_VIDEO_INDEX, UVC_COLOR_SIZE, UVC_FPS, UVC_VIDEO_FORMAT)
    else:
        if not os.path.exists(video_path):
            print(f"Unable to Find device: {video_path}")
            return None
        print(f"USB Camera Stream ({video_path}) Settings:\n UVC_COLOR_SIZE=Unset\n FPS=Unset")
        uvc_camera = hu.setup_uvc_camera(video_path)
    
    while not stop_stream:
        try:
            ret, image_uvc = uvc_camera.read()
        except Exception as e:
            print(f"Error UVC Cam: {e}")
    uvc_camera.release()

def print_video_devices_list():
    print("Found the following Video Devices:")
    video_devices = hu.get_video_devices()
    for devices in video_devices:
        print(f"{devices}")
        for d in video_devices[devices]:
            print(f"\t\t{d}")


def main(config):
    global stop_stream, color_image_d405, depth_image_d405, color_image_d435i, depth_image_d435i, image_uvc

    print('cv2.__version__ =', cv2.__version__)
    print('sys.version =', sys.version)

    d405=config['d405']
    d435i=config['d435i']
    uvc=config['navigation']
    video_path = config['usb_cam']

    if d405:
        d405_thread = Thread(target=d405_stream)
        d405_thread.daemon =True
        d405_thread.start()

    if d435i:
        d435i_thread = Thread(target=d435i_stream)
        d435i_thread.daemon =True
        d435i_thread.start()
    
    if uvc or video_path:
        uvc_thread = Thread(target=uvc_cam_stream,args=(video_path,))
        uvc_thread.daemon =True
        uvc_thread.start()

    time.sleep(3)

    while not stop_stream:
        try:
            if color_image_d405 is not None:
                cv2.imshow('D405 Color', color_image_d405)
            if depth_image_d405 is not None:
                cv2.imshow('D405 Depth', depth_image_d405)
            if color_image_d435i is not None:
                cv2.imshow('D435i Color', cv2.rotate(color_image_d435i, cv2.ROTATE_90_CLOCKWISE))
            if depth_image_d435i is not None:
                cv2.imshow('D435i Depth', cv2.rotate(depth_image_d435i, cv2.ROTATE_90_CLOCKWISE))
            if image_uvc is not None:
                # Change navigation camera orientation
                if uvc:
                    cv2.imshow('Navigation Head Camera', cv2.rotate(image_uvc, cv2.ROTATE_90_COUNTERCLOCKWISE))
                if video_path is not None:
                    cv2.imshow('USB Camera',image_uvc)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                # Release resources
                stop_stream = True
                cv2.destroyAllWindows()
        except (KeyboardInterrupt,SystemExit):
            stop_stream = True
    time.sleep(0.2)
    if d405:
        d405_thread.join()
    if d435i:
        d435i_thread.join()
    if uvc or video_path:
        uvc_thread.join()
    sys.exit()



hu.print_stretch_re_use()




parser = argparse.ArgumentParser(description="This tool will start streaming videos feeds"
                                             " from D435i, D405 and Navigation Camera")
parser.add_argument('--d435i', help='Streams D435i streams only',action="store_true")
parser.add_argument('--d405', help='Streams d405 streams only',action="store_true")
parser.add_argument('--navigation', help='Streams navigation camera stream only',action="store_true")
parser.add_argument('--usb_cam_port', type=str,help='Stream from usb video device in given port. E.g. --usb_cam_port /dev/video4')
parser.add_argument('--usb_cam_name', type=str,help='Stream from usb video with a camera name. E.g. --usb_cam_name Arducam')
parser.add_argument('--list', help='List all the enumerated Video devices',action="store_true")
args = vars(parser.parse_args())

config = {'d405' : False,
            'd435i' : False,
            'navigation' : False,
            'usb_cam': None}

if args['list']:
    print_video_devices_list()
    sys.exit()

if args['d405']:
    config['d405'] = True
    main(config)

if args['d435i']:
    config['d435i'] = True
    main(config)

if args['navigation']:
    config['navigation'] = True
    main(config)

if args['usb_cam_port']:
    config['usb_cam'] = args['usb_cam_port']
    main(config)

if args['usb_cam_name']:
    config['usb_cam'] = hu.get_video_device_port(args['usb_cam_name'])
    main(config)

config = {'d405' : True,
        'd435i' : True,
        'navigation' : True,
        'usb_cam': None}

main(config)



        

    
