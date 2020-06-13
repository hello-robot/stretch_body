#!/usr/bin/env python
import os
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo')
os.system('amixer set Master 80%')
os.system('paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')

