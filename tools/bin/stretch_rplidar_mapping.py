#!/usr/bin/env python
import pickle
import pathlib
import argparse
import numpy as np
import rplidar as rp
from drawnow import drawnow
import matplotlib.pyplot as plt
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import lidar_to_grid_map as lg

plt.ion()
plt.figure(figsize=(10, 10))
ybounds = None


def to_cartesian(polar_scans):
    """Cartesian cloud generator

    Given scans where each measurement is composed
    of angle and distance, this generator yield a
    "cloud", a numpy array of shape (2, n). Each 
    cloud is contains `n` cartesian points.
    `cloud[0]` and `cloud[1]` are the x and y
    coordinates respectively.
    """
    for polar_scan in polar_scans:
        angles = np.radians(np.array([measurement[1] for measurement in polar_scan], dtype=np.float64))
        distances = np.array([measurement[2] for measurement in polar_scan], dtype=np.float64) / 1000.0
        yield np.array([np.cos(angles) * distances, -np.sin(angles) * distances])


def plotter(data):
    global ybounds
    try:
        pmap, minx, maxx, miny, maxy, xyreso = lg.generate_ray_casting_grid_map(data[0], data[1], 0.1, True)
        print(xyreso)
        if not ybounds:
            ybounds = (miny, maxy)
        xyres = np.array(pmap).shape
        plt.imshow(pmap, cmap="PiYG_r")
        # plt.clim(-0.4, 1.4)
        # plt.ylim(ybounds)
        # plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor=True)
        # plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=.6, alpha=0.5)
    except:
        pass
    #     plt.scatter(data[0], data[1], marker='o')
    #     plt.axis('equal')
    #     plt.grid(True)


def mapper(cloud_iterator):
    for cloud in cloud_iterator:
        drawnow(plotter, stop_on_close=True, data=cloud)
        time.sleep(1.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Use the 2D lidar to build out a map.')
    parser.add_argument("--lidar_file", nargs="?", type=str, dest="lidar_file", const="",
                        help="Filepath to a saved pickle of lidar scans")
    args, _ = parser.parse_known_args()

    iterable = None
    if args.lidar_file:
        lidar_file = pathlib.Path(args.lidar_file)
        if not lidar_file.is_file():
            raise Exception("Expected file at given path '%s'" % lidar_file)
        with lidar_file.open('rb') as f:
            iterable = to_cartesian(pickle.load(f))
    else:
        try:
            lidar = rp.RPLidar('/dev/hello-lrf')
            iterable = to_cartesian(lidar.iter_scans())
            next(iterable) # skip first abridged cloud
        except rp.RPLidarException:
            print('RPLidar not present')
            exit()

    try:
        print("Press 'q' to exit visualization")
        mapper(iterable)
    except SystemExit, KeyboardInterrupt:
        pass

    if not args.lidar_file:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
