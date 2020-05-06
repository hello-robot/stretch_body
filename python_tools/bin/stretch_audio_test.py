#!/usr/bin/env python
import os

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo')
os.system('amixer set Master 80%')
os.system('paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')

