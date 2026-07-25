#!/usr/bin/env python3
'''gopro control over mavlink for the solo-gimbal

To use this module connect to a Solo with a GoPro installed on the gimbal.
'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
from MAVProxy.modules.lib.mp_i18n import tr

class GoProModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(GoProModule, self).__init__(mpstate, "gopro", tr("mod_gopro_handling"))

        self.add_command('gopro', self.cmd_gopro,   tr("cmd_gopro_control"), [
                                        'status',
                                        'shutter <start|stop>',
                                        'mode <video|camera>',
                                        'power <on|off>'])

    def cmd_gopro(self, args):
        '''gopro commands'''
        usage = tr("usage_status_shutter_start_stop_mode_video_camera_power")
        mav = self.master.mav

        if args[0] == "status":
            self.cmd_gopro_status(args[1:])
            return

        if args[0] == "shutter":
            name = args[1].lower()
            if name == 'start':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_SHUTTER, [1, 0 ,0 , 0])
                return
            elif name == 'stop':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_SHUTTER, [0, 0 ,0 , 0])
                return
            else:
                print(tr("unrecognized"))
                return

        if args[0] == "mode":
            name = args[1].lower()
            if name == 'video':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, [0, 0 ,0 , 0])
                return
            elif name == 'camera':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, [1, 0 ,0 , 0])
                return
            else:
                print(tr("unrecognized"))
                return

        if args[0] == "power":
            name = args[1].lower()
            if name == 'on':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_POWER, [1, 0 ,0 , 0])
                return
            elif name == 'off':
                mav.gopro_set_request_send(self.target_system, mavutil.mavlink.MAV_COMP_ID_GIMBAL,
                 mavutil.mavlink.GOPRO_COMMAND_POWER, [0, 0 ,0 , 0])
                return
            else:
                print(tr("unrecognized"))
                return

        print(usage)

    def cmd_gopro_status(self, args):
        '''show gopro status'''
        master = self.master
        if 'GOPRO_HEARTBEAT' in master.messages:
            print(master.messages['GOPRO_HEARTBEAT'])
        else:
            print(tr("no_gopro_heartbeat_messages"))

def init(mpstate):
    '''initialise module'''
    return GoProModule(mpstate)
