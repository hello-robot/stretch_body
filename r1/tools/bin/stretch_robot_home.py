#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.robot as robot
import argparse
import sys
import time
import json
import yaml
import pathlib
import requests
import xmltodict
import multiprocessing
import http.client as httplib
try:
    import stretch_factory.firmware_available
    factory_installed = True
except:
    factory_installed = False
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Find zeros for all robot joints')
args=parser.parse_args()

def fetch_updates_in_background():
    def have_internet():
        conn = httplib.HTTPSConnection("8.8.8.8", timeout=5)
        try:
            conn.request("HEAD", "/")
            return True
        except Exception:
            return False
        finally:
            conn.close()
    def get_latest_pypi_version(url):
        resp = requests.get(url)
        if resp.status_code == 200:
            releases = xmltodict.parse(resp.text)['rss']['channel']['item']
            for i in range(len(releases)):
                if 'dev' not in releases[i]['title']:
                    return releases[i]['title']
        return None
    def get_latest_github_commit(url):
        resp = requests.get(url)
        if resp.status_code == 200:
            return json.loads(resp.text)[0]['sha']
        return None

    try:
        start = time.time()
        scans_dir = pathlib.Path('~/stretch_user/log/updates_logger').expanduser()
        if not scans_dir.is_dir():
            scans_dir.mkdir(parents=True, exist_ok=True)
        datetime_str = hu.create_time_string()
        scan_file = scans_dir / f'updates_scan.{datetime_str}.yaml'
        scan_log = scans_dir / f'log_updates_scan.{datetime_str}.log'

        # override stdout/stderr
        sys.stdout = open(str(scan_log), "a")
        sys.stderr = open(str(scan_log), "a")

        # check for Stretch Factory installed
        if not factory_installed:
            return
        print('background: got factory installed!', time.time() - start, flush=True)

        # check for internet connection
        if not have_internet():
            return
        print('background: got internet!', time.time() - start, flush=True)

        # check for firmware updates
        use_device = {'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True,'hello-motor-lift': True}
        fa = stretch_factory.firmware_available.FirmwareAvailable(use_device)
        scan_dict = {
            'firmware': {
                'hello-pimu': str(fa.get_most_recent_version('hello-pimu', None)),
                'hello-wacc': str(fa.get_most_recent_version('hello-wacc', None)),
                'hello-motor-arm': str(fa.get_most_recent_version('hello-motor-arm', None)),
                'hello-motor-lift': str(fa.get_most_recent_version('hello-motor-lift', None)),
                'hello-motor-left-wheel': str(fa.get_most_recent_version('hello-motor-left-wheel', None)),
                'hello-motor-right-wheel': str(fa.get_most_recent_version('hello-motor-right-wheel', None)),
            },
            'version': '0.0',
        }
        print('background: got latest firmware!', time.time() - start, flush=True)

        # check for pip updates
        scan_dict['pip'] = {
            'hello-robot-stretch-body': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-body/releases.xml"),
            'hello-robot-stretch-body-tools': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-body-tools/releases.xml"),
            'hello-robot-stretch-tool-share': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-tool-share/releases.xml"),
            'hello-robot-stretch-factory': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-factory/releases.xml"),
            'hello-robot-stretch-diagnostics': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-diagnostics/releases.xml"),
            'hello-robot-stretch-urdf': get_latest_pypi_version("https://pypi.org/rss/project/hello-robot-stretch-urdf/releases.xml"),
        }
        print('background: got latest pip!', time.time() - start, flush=True)

        # check latest pushed github commits
        scan_dict['ros'] = {
            'stretch_ros': get_latest_github_commit("https://api.github.com/repos/hello-robot/stretch_ros/commits"),
            'stretch_ros2': get_latest_github_commit("https://api.github.com/repos/hello-robot/stretch_ros2/commits"),
        }
        print('background: got latest github!', time.time() - start, flush=True)

        # save to updates_scan directory
        with open(str(scan_file), 'w') as outfile:
            yaml.dump(scan_dict, outfile)
        print('background: saved to scan logger dir', time.time() - start, flush=True)
    except:
        pass
background_process = multiprocessing.Process(target=fetch_updates_in_background)

r=robot.Robot()
if not r.startup():
    sys.exit(1)
if r.pimu.status['runstop_event']:
    r.logger.error('Cannot home while run-stopped')
    r.stop()
    sys.exit(1)

background_process.start()
r.home()
background_process.terminate()
background_process.join()

r.stop()
