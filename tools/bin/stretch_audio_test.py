#!/usr/bin/env python
from __future__ import print_function
import os
import stretch_body.hello_utils as hu
import lsb_release
hu.print_stretch_re_use()

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo')
os.system('amixer set Master 80%')
if lsb_release.get_distro_information()['RELEASE']=='20.04':
    os.system('canberra-gtk-play --id="system-ready"')
if lsb_release.get_distro_information()['RELEASE']=='18.04':
    os.system('paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-login.ogg')

