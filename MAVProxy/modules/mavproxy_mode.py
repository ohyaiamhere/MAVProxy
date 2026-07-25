#!/usr/bin/env python3
'''mode command handling'''

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib.mp_i18n import tr

'''
AP_FLAKE8_CLEAN
'''


class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode", public=True)
        self.add_command('mode', self.cmd_mode, tr("cmd_mode_change"), [
            '(MODE)'
        ])
        self.add_command('guided', self.cmd_guided, tr("cmd_fly_to_a_clicked_location_on_map"))
        self.add_command('confirm', self.cmd_confirm, tr("cmd_confirm_a_command"))
        self.add_completion_function('(MODE)', self.complete_available_modes)

    def cmd_mode(self, args):
        '''set arbitrary mode'''
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print(tr("no_mode_mapping_available"))
            return
        if len(args) != 1:
            print(tr("available_modes"), ', '.join(self.available_modes()))
            return
        if args[0].isdigit():
            modenum = int(args[0])
        else:
            mode = args[0].upper()
            if mode not in mode_mapping:
                print(tr("unknown_mode") % mode)
                return
            modenum = mode_mapping[mode]
        self.master.set_mode(modenum)

    def cmd_confirm(self, args):
        '''confirm a command'''
        if len(args) < 2:
            print(tr("usage_confirm_question_to_display_command"))
            return
        question = args[0].strip('"')
        command = ' '.join(args[1:])
        if not mp_util.has_wxpython:
            print(tr("no_ui_available_for_confirm"))
            return
        from MAVProxy.modules.lib import mp_menu
        mp_menu.MPMenuConfirmDialog(question, callback=self.mpstate.functions.process_stdin, args=command)

    def complete_available_modes(self, text):
        return self.available_modes()

    def available_modes(self):
        if self.master is None:
            print(tr("no_mode_mapping_available"))
            return []
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print(tr("no_mode_mapping_available"))
            return []
        return mode_mapping.keys()

    def unknown_command(self, args):
        '''handle mode switch by mode name as command'''
        mode_mapping = self.master.mode_mapping()
        mode = args[0].upper()
        if mode in mode_mapping:
            self.master.set_mode(mode_mapping[mode])
            return True
        return False

    def cmd_guided(self, args):
        '''set GUIDED target'''
        if len(args) > 0:
            if args[0] == "forward":
                return self.cmd_guided_forward(args[1:])

        if len(args) == 2:
            frames = ['AboveHome', 'AGL', 'AMSL']
            if args[1] in frames:
                self.settings.flytoframe = args[1]
            else:
                print(tr("usage_guided_altitude") % '|'.join(frames))
                return
        elif len(args) != 1 and len(args) != 3:
            print(tr("usage_guided_altitude_guided_lat_lon"))
            return

        frame = self.flyto_frame()

        if len(args) == 3:
            latitude = float(args[0])
            longitude = float(args[1])
            altitude = float(args[2])
            latlon = (latitude, longitude)
        else:
            latlon = self.mpstate.click_location
            if latlon is None:
                print(tr("no_map_click_position_available"))
                return
            altitude = float(args[0])

        altitude = self.height_convert_from_units(altitude)

        print(tr("guided_frame_u") % (str(latlon), str(altitude), frame))

        if self.settings.guided_use_reposition:
            self.master.mav.command_int_send(
                self.settings.target_system,
                self.settings.target_component,
                frame,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,  # current
                0,  # autocontinue
                -1,   # p1 - ground speed, -1 is use-default
                mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,   # p2 - flags
                0,   # p3 - loiter radius for Planes, 0 is ignored
                0,   # p4 - yaw - 0 is loiter clockwise
                int(latlon[0]*1.0e7),
                int(latlon[1]*1.0e7),
                altitude
            )
            return

        self.master.mav.mission_item_int_send(
            self.settings.target_system,
            self.settings.target_component,
            0,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, 0, 0, 0,
            int(latlon[0]*1.0e7),
            int(latlon[1]*1.0e7),
            altitude
        )

    def build_pt_ignoremask(self, bits, force_not_accel=False):
        '''creates an ignore bitmask which ignores all bits except the ones passed in'''
        ignore_bits = {
            "X": 1,  # POSITION_TARGET_TYPEMASK_X_IGNORE
            "Y": 2,
            "Z": 4,
            "VX": 8,
            "VY": 16,
            "VZ": 32,
            "AX": 64,
            "AY": 128,
            "AZ": 256,
            "YAW": 1024,
            "YAWRATE": 2048,
        }
        value = 0
        for (n, v) in ignore_bits.items():
            if n not in bits:
                value |= v

        # as of ArduCopter 4.6.0-dev, you can't ignore any axis if you
        # want the others to be honoured.
        for prefix in "", "V", "A":
            for axis in "X", "Y", "Z":
                name = f"{prefix}{axis}"
                if (value & ignore_bits[name]) == 0:
                    # not ignoring this axis, so unmark the other axes as ignored
                    for resetaxis in "X", "Y", "Z":
                        resetname = f"{prefix}{resetaxis}"
                        value = value & ~ignore_bits[resetname]
                    break

        if force_not_accel:
            value |= 512  # POSITION_TARGET_TYPEMASK_FORCE_SET

        return value

    def cmd_guided_forward(self, args):
        if len(args) != 1:
            print(tr("usage_guided_forward_metres"))
            return
        offset = args[0]
        # see also "cmd_position" in mavproxy_cmdlong.py
        self.master.mav.set_position_target_local_ned_send(
            0,  # system time in milliseconds
            self.settings.target_system,  # target system
            0,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.build_pt_ignoremask(["X", "YAWRATE"]),     # type mask (pos-x-only)
            float(offset), 0, 0,  # position x,y,z
            0, 0, 0,  # velocity x,y,z
            0, 0, 0,  # accel x,y,z
            0, 0      # yaw, yaw rate
        )

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == 'HIGH_LATENCY2':
            mode_map = mavutil.mode_mapping_bynumber(m.type)
            if mode_map and m.custom_mode in mode_map:
                self.master.flightmode = mode_map[m.custom_mode]


def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
