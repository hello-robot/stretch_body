#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2

import os
import pathlib
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Tool to test the Realsense D435i Camera.')
parser.add_argument("--no_gui", action="store_true",
                    help="Show no GUI while reading images.")
parser.add_argument("--colormap", type=int, default=cv2.COLORMAP_OCEAN,
                    help="Valid OpenCV colormaps at 'https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html'.")
parser.add_argument("--save", nargs="?", type=str, const="",
                    help="Save as .avi video to given filepath at end of script.")
parser.add_argument("--save_limit", type=int, default=1,
                    help="The number of minutes of data to save.")
args, _ = parser.parse_known_args()

# Check if no display exists
if hu.get_display() is None:
    if args.no_gui is False:
        print('No display found. Setting no_gui=true')
    args.no_gui = True

# Configure depth and color streams
fps = 30
width = 640
height = 480
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

# Start streaming
stream = []
stream_limit = fps * 60 * args.save_limit
pipeline.start(config)
print("Press any key or Ctrl-C to exit...")

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=-0.04, beta=255.0), args.colormap)

        # Rotate images
        color_image = np.moveaxis(color_image, 0, 1)
        depth_colormap = np.moveaxis(depth_colormap, 0, 1)

        # Stack both images horizontally
        both_image = np.hstack((color_image, depth_colormap))

        # show stream if no_gui disabled
        if not args.no_gui:
            cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Realsense', both_image)
            if cv2.waitKey(1) & 0xFF != 255:
                raise KeyboardInterrupt()

        # maintain stream cache for saving video
        if args.save is not None:
            stream.append(both_image)
            if len(stream) > stream_limit:
                stream.pop(0)
except KeyboardInterrupt:
    pass
finally:
    pipeline.stop()
    if (args.save is not None) and (os.access(str(pathlib.Path(args.save).parent), os.W_OK)) and (str(args.save).lower().endswith('.avi')):
        print("Saving {0} frames to '{1}'".format(len(stream), pathlib.Path(args.save)))
        writer = cv2.VideoWriter(str(pathlib.Path(args.save)), cv2.VideoWriter_fourcc(*'MJPG'), fps, (2 * height, width))
        for image in stream:
            writer.write(image)
        writer.release()
        print("Done saving stream.")
