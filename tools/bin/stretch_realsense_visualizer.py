#!/usr/bin/env python3

# Sample command line usage:
#   python stretch_realsense_jog.py --save stretch_video_max60minutes.avi
#   python stretch_realsense_jog.py --save stretch_video_max90minutes.avi --save_limit 90
# See the parser arguments below for more command-line options.
# See the below code for more settings such as resolutions and whether to use color and/or depth images.
#
# Note that the actual frame rate may be lower than the nominal frame rate, so playback of saved videos
#  may be at the wrong speed.  This is especially relevant when using higher image resolutions.
#  The real-time duration and actual average frame rate will be added to the saved filenames,
#  so post-processing can try to speed up the videos. However, the actual frame rate is often variable.
#
# Originally posted: 2021-05-17 by Binit Shah (see https://forum.hello-robot.com/t/170)
# Revised:           2021-05-30 by Joseph DelPreto

import pyrealsense2 as rs
import numpy as np
import cv2

import time, datetime
import os
import pathlib
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

######################################
# Settings via command line arguments
######################################

parser = argparse.ArgumentParser(description='Tool to test the Realsense D435i Camera.')
parser.add_argument("--no_gui", action="store_true",
                    help="Show no GUI while reading images.")
parser.add_argument("--colormap", type=int, default=cv2.COLORMAP_OCEAN,
                    help="Valid OpenCV colormaps at 'https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html'.")
parser.add_argument("--save", nargs="?", type=str, const="",
                    help="Save as .avi video to given filepath at end of script.")
parser.add_argument("--save_limit", type=int, default=60,
                    help="The number of minutes of data to save.")
parser.add_argument("--d405", action="store_true",
                    help="By default, this tool shows the D435if head camera imagery. Setting this flag causes the tool to instead show the D405 wrist imagery.")
parser.add_argument('--exposure', action='store', type=str, default='auto',
                    help="Set the D405 exposure to ['low', 'medium', 'auto'] or an integer in the range [0, 165000]")
args, _ = parser.parse_known_args()
output_filepath = args.save

####################################
# Additional settings
####################################

# Whether to save a video of the color images and/or depth images.
save_video_color = True
save_video_depth = True
save_video_colorAndDepth = True # color and depth concatenated side-by-side
# Video streaming configuration.
# Note: the actual fps will probably be lower than the target, especially if larger resolutions are used or multiple videos are saved.
fps_color = 30 # FPS for color-only videos and for color+depth videos
fps_depth_downsample_factor = 3 # The depth frame rate will be fps_color/fps_depth_downsample_factor.
                                # Only used for the depth-only video stream (combined color+depth videos will be at the color fps).
resolution_color = [640, 480]   # [1920, 1080], [1280, 720], [640, 480]
resolution_depth = [640, 480]   # [1280, 720], [640, 480]
frameBuffer_limit_s = 10 # How many seconds of frames to keep in the buffer - probably something short to avoid memory usage issues
                         # (different from save_limit, which limits the overall video capture length and is set by the command line argument above).
# How to adjust the filenames (to add more info and to avoid accidentally overwriting videos).
output_filepath_addTimestamp = True
output_filepath_addFps = True
output_filepath_addDuration = True
# Some image processing options.
apply_local_histogram_equalization = False
apply_global_histogram_equalization = False

####################################
# Initialize
####################################

serial_no = None
rs_cameras = [{'name': device.get_info(rs.camera_info.name), 'serial_number': device.get_info(rs.camera_info.serial_number)}
    for device in rs.context().devices]
if args.d405:
    for info in rs_cameras:
        if 'D405' in info['name']:
            serial_no = info['serial_number']
    if serial_no is None:
        print("Wrist D405 camera not found. Exiting.")
        sys.exit(1)
else:
    for info in rs_cameras:
        if 'D435' in info['name']:
            serial_no = info['serial_number']
    if serial_no is None:
        print("Head D435 camera not found. Exiting.")
        sys.exit(1)
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(serial_no)
# Configure depth and color streams.
# Note: width/heights are swapped since the images will be rotated 90 degrees from what the camera captures.
config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps_color) # note that the fps downsampling will be applied later
config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps_color)
frame_width_colorAndDepth = resolution_color[1] + resolution_depth[1]
frame_height_colorAndDepth = max(resolution_color[0], resolution_depth[0])
frame_width_color = resolution_color[1]
frame_height_color = resolution_color[0]
frame_width_depth = resolution_depth[1]
frame_height_depth = resolution_depth[0]

# Create writer(s) to save the stream(s) if desired.
writer_color = None
writer_depth = None
writer_colorAndDepth = None
if (output_filepath is not None) and (os.access(str(pathlib.Path(output_filepath).parent), os.W_OK)):
    # Add a timestamp to the filename if desired.
    if output_filepath_addTimestamp:
        date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        (dir, filename) = os.path.split(output_filepath)
        (filename_base, ext) = os.path.splitext(os.path.basename(output_filepath))
        output_filepath = os.path.join(dir, '%s_%s%s' % (date_str, filename_base, ext))
    # Strip the extension if one was provided so we can ensure it's an AVI file.
    (dir, filename) = os.path.split(output_filepath)
    (filename_base, ext) = os.path.splitext(os.path.basename(output_filepath))
    # Indicate whether it's color and/or depth images.
    output_filepath_color = os.path.join(dir, '%s_color.avi' % (filename_base))
    output_filepath_depth = os.path.join(dir, '%s_depth.avi' % (filename_base))
    output_filepath_colorAndDepth = os.path.join(dir, '%s_colorAndDepth.avi' % (filename_base))
    # Open the writer(s).
    if save_video_color:
        writer_color = cv2.VideoWriter(str(pathlib.Path(output_filepath_color)), cv2.VideoWriter_fourcc(*'MJPG'), fps_color, (frame_width_color, frame_height_color))
    if save_video_depth:
        writer_depth = cv2.VideoWriter(str(pathlib.Path(output_filepath_depth)), cv2.VideoWriter_fourcc(*'MJPG'), fps_color/fps_depth_downsample_factor, (frame_width_depth, frame_height_depth))
    if save_video_colorAndDepth:
        writer_colorAndDepth = cv2.VideoWriter(str(pathlib.Path(output_filepath_colorAndDepth)), cv2.VideoWriter_fourcc(*'MJPG'), fps_color, (frame_width_colorAndDepth, frame_height_colorAndDepth))

# Check if no display exists.
try:
    if hu.get_display() is None:
        if args.no_gui is False:
            print('No display found. Setting no_gui=true')
        args.no_gui = True
except:
    args.no_gui = True

####################################
# Stream!
####################################

frameBuffer_color = []
frameBuffer_depth = []
frameBuffer_colorAndDepth = []
frameBuffer_limit_color = round(frameBuffer_limit_s * fps_color)
frameBuffer_limit_depth = round(frameBuffer_limit_s * fps_color / fps_depth_downsample_factor)
capture_limit_color = round(fps_color * 60 * args.save_limit)

pipeline.start(config)
# Set exposure for D405
if args.d405:
    if args.exposure == 'auto':
        # Use autoexposure
        stereo_sensor = pipeline.get_active_profile().get_device().query_sensors()[0]
        stereo_sensor.set_option(rs.option.enable_auto_exposure, True)
    else:
        if args.exposure == 'low':
            exposure_value = 33000
        elif args.exposure == 'medium':
            exposure_value = 85000
        else:
            exposure_value = int(args.exposure)

        stereo_sensor = pipeline.get_active_profile().get_device().query_sensors()[0]
        stereo_sensor.set_option(rs.option.exposure, exposure_value)

start_capture_time_s = time.time()
end_capture_time_color_s = start_capture_time_s
end_capture_time_depth_s = start_capture_time_s
frame_count_color = 0
frame_count_depth = 0
print("Press Ctrl-C to exit...")
try:
    while frame_count_color < capture_limit_color:
        # Get the latest frames from the camera.
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        capture_time_s = time.time()

        # Convert images to numpy arrays.
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply histogram equalization if desired.
        if apply_local_histogram_equalization:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            color_image_lab = cv2.cvtColor(color_image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
            color_image_l, color_image_a, color_image_b = cv2.split(color_image_lab)  # split on 3 different channels
            color_image_l_equalized = clahe.apply(color_image_l)  # apply CLAHE to the L-channel
            color_image_lab = cv2.merge((color_image_l_equalized, color_image_a, color_image_b))  # merge channels
            color_image = cv2.cvtColor(color_image_lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR
        elif apply_global_histogram_equalization:
            color_image_lab = cv2.cvtColor(color_image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
            color_image_l, color_image_a, color_image_b = cv2.split(color_image_lab)  # split on 3 different channels
            color_image_l_equalized = cv2.equalizeHist(color_image_l) # apply global equalization to the L-channel
            color_image_lab = cv2.merge((color_image_l_equalized, color_image_a, color_image_b))  # merge channels
            color_image = cv2.cvtColor(color_image_lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first).
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=-0.04, beta=255.0), args.colormap)

        if not args.d405:
            # Rotate and flip images.
            color_image = np.moveaxis(color_image, 0, 1)
            color_image = np.fliplr(color_image)
            depth_image = np.moveaxis(depth_image, 0, 1)
            depth_image = np.fliplr(depth_image)

        # Concatenate images horizontally if desired.
        if save_video_colorAndDepth or not args.no_gui:
            # Pad images if they are different heights
            pad_y = color_image.shape[0] - depth_image.shape[0]
            if pad_y > 0:
                color_image_padded = color_image
                depth_image_padded = np.pad(depth_image, ((pad_y, 0), (0, 0), (0, 0)), mode='constant', constant_values=0)
            else:
                pad_y = abs(pad_y)
                color_image_padded = np.pad(color_image, ((pad_y, 0), (0, 0), (0, 0)), mode='constant', constant_values=0)
                depth_image_padded = depth_image
            # Tile the images
            colorAndDepth_image = np.hstack((color_image_padded, depth_image_padded))
        else:
            colorAndDepth_image = None

        # Show stream if no_gui disabled.
        if not args.no_gui:
            cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Realsense', colorAndDepth_image)
            if cv2.waitKey(1) & 0xFF != 255:
                raise KeyboardInterrupt()

        # Maintain stream cache.
        # Important to use copy() here depending on how the stream is used.
        frameBuffer_color.append(color_image.copy())
        frameBuffer_depth.append(depth_image.copy())
        frameBuffer_colorAndDepth.append(colorAndDepth_image)
        if len(frameBuffer_color) > frameBuffer_limit_color:
            frameBuffer_color.pop(0)
            frameBuffer_colorAndDepth.pop(0)
        if len(frameBuffer_depth) > frameBuffer_limit_depth:
            frameBuffer_depth.pop(0)
        # Write to video output(s) if desired.
        if writer_color is not None:
            writer_color.write(frameBuffer_color[-1])
            end_capture_time_color_s = capture_time_s
        if writer_colorAndDepth is not None:
            writer_colorAndDepth.write(frameBuffer_colorAndDepth[-1])
            end_capture_time_color_s = capture_time_s
        if writer_depth is not None and frame_count_color % fps_depth_downsample_factor == 0:
            writer_depth.write(frameBuffer_depth[-1])
            end_capture_time_depth_s = capture_time_s
            frame_count_depth += 1
        frame_count_color += 1
except KeyboardInterrupt:
    pass
finally:
    ####################################
    # Stop video streaming
    ####################################

    # Stop the streaming pipeline
    pipeline.stop()
    # Stop the video writer(s)
    if writer_color is not None:
        writer_color.release()
    if writer_colorAndDepth is not None:
        writer_colorAndDepth.release()
    if writer_depth is not None:
        writer_depth.release()

####################################
# Clean up and finish saving videos
####################################

# Get some information about the videos
capture_duration_color_s = end_capture_time_color_s - start_capture_time_s
capture_duration_depth_s = end_capture_time_depth_s - start_capture_time_s
capture_fps_color = (float(frame_count_color)/float(capture_duration_color_s)) if capture_duration_color_s > 0 else None
capture_fps_depth = (float(frame_count_depth)/float(capture_duration_depth_s)) if capture_duration_depth_s > 0 else None

# Add additional information to the filenames if desired
def addDurationToFilename(filepath, duration_s):
    duration_str = ('%0.3f' % duration_s)
    duration_str_forFilename = duration_str.replace('.', '-')
    (dir, filename) = os.path.split(filepath)
    (filename_base, ext) = os.path.splitext(os.path.basename(filepath))
    filepath_withDuration = os.path.join(dir, '%s_duration%ss%s' % (filename_base, duration_str_forFilename, ext))
    os.rename(filepath, filepath_withDuration)
    return filepath_withDuration
def addFpsToFilename(filepath, fps):
    fps_str = ('%0.3f' % fps)
    fps_str_forFilename = fps_str.replace('.', '-')
    (dir, filename) = os.path.split(filepath)
    (filename_base, ext) = os.path.splitext(os.path.basename(filepath))
    filepath_withFps = os.path.join(dir, '%s_fps%s%s' % (filename_base, fps_str_forFilename, ext))
    os.rename(filepath, filepath_withFps)
    return filepath_withFps
if output_filepath_addDuration:
    if writer_color is not None and os.path.exists(output_filepath_color):
        output_filepath_color = addDurationToFilename(output_filepath_color, capture_duration_color_s)
    if writer_colorAndDepth is not None and os.path.exists(output_filepath_colorAndDepth):
        output_filepath_colorAndDepth = addDurationToFilename(output_filepath_colorAndDepth, capture_duration_color_s)
    if writer_depth is not None and os.path.exists(output_filepath_depth):
        output_filepath_depth = addDurationToFilename(output_filepath_depth, capture_duration_depth_s)
if output_filepath_addFps:
    if writer_color is not None and os.path.exists(output_filepath_color):
        output_filepath_color = addFpsToFilename(output_filepath_color, capture_fps_color)
    if writer_colorAndDepth is not None and os.path.exists(output_filepath_colorAndDepth):
        output_filepath_colorAndDepth = addFpsToFilename(output_filepath_colorAndDepth, capture_fps_color)
    if writer_depth is not None and os.path.exists(output_filepath_depth):
        output_filepath_depth = addFpsToFilename(output_filepath_depth, capture_fps_depth)

print('')
print('')
if capture_fps_color is not None:
    print('Captured %d color frames in %0.3f seconds' % (frame_count_color, capture_duration_color_s))
    print('(average FPS: %8.5f)' % capture_fps_color)
if capture_fps_depth is not None:
    print('Captured %d depth frames in %0.3f seconds' % (frame_count_depth, capture_duration_depth_s))
    print('(average FPS: %8.5f)' % capture_fps_depth)
print('')
if writer_color is not None:
    print('Saved color video at %g fps to %s' % (fps_color, output_filepath_color))
if writer_depth is not None:
    print('Saved depth video at %g fps to %s' % (fps_color/fps_depth_downsample_factor, output_filepath_depth))
if writer_colorAndDepth is not None:
    print('Saved color+depth video at %g fps to %s' % (fps_color, output_filepath_colorAndDepth))
if writer_color is not None or writer_depth is not None or writer_colorAndDepth is not None:
    print('')
    print('* Note that the actual frame rates are different than the frame rates ')
    print('  of the saved videos, so playback speed may be inaccurate.')
print('')
