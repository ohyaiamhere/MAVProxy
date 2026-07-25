#!/usr/bin/env python3
'''
command long

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr


class CmdlongModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CmdlongModule, self).__init__(mpstate, "cmdlong", public=True)
        self.add_command('setspeed', self.cmd_do_change_speed, tr("cmd_do_change_speed"))
        self.add_command('setyaw', self.cmd_condition_yaw, tr("cmd_condition_yaw"))
        self.add_command('takeoff', self.cmd_takeoff, tr("cmd_takeoff"))
        self.add_command('velocity', self.cmd_velocity, tr("cmd_velocity"))
        self.add_command('position', self.cmd_position, tr("cmd_position"))
        self.add_command('attitude', self.cmd_attitude, tr("cmd_attitude"))
        self.add_command('cammsg', self.cmd_cammsg, tr("cmd_cammsg"))
        self.add_command('cammsg_old', self.cmd_cammsg_old, tr("cmd_cammsg_old"))
        self.add_command('camctrlmsg', self.cmd_camctrlmsg, tr("cmd_camctrlmsg"))
        self.add_command('posvel', self.cmd_posvel, tr("cmd_posvel"))
        self.add_command('parachute', self.cmd_parachute, tr("cmd_parachute"),
                         ['<enable|disable|release>'])
        self.add_command('long', self.cmd_long, tr("cmd_execute_mavlink_long_command"),
                         self.cmd_long_commands())
        self.add_command('command_int', self.cmd_command_int, tr("cmd_execute_mavlink_command_int"),
                         self.cmd_long_commands())
        self.add_command('engine', self.cmd_engine, tr("cmd_engine"))
        self.add_command('pause', self.cmd_pause, tr("cmd_pause_auto_guided_modes"))
        self.add_command('resume', self.cmd_resume, tr("cmd_resume_auto_guided_modes"))

    def cmd_long_commands(self):
        atts = dir(mavutil.mavlink)
        atts = filter(lambda x : x.lower().startswith("mav_cmd"), atts)
        ret = []
        for att in atts:
            ret.append(att)
            ret.append(str(att[8:]))
        return ret

    def cmd_takeoff(self, args):
        '''take off ALTITUDE_IN_METERS'''
        if len(args) != 1:
            print(tr("usage_takeoff_altitude_in_meters"))
            return

        try:
            altitude = float(args[0])
        except ValueError:
            print(tr("error_invalid_altitude_value"))
            print(tr("usage_takeoff_altitude_in_meters"))
            return

        # Warn if disarmed
        if not self.master.motors_armed():
            print(tr("warning_vehicle_is_disarmed_use_arm"))

        print(tr("taking_off_to_meters") % altitude)
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            0,  # confirmation
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            altitude)  # param7

    def cmd_parachute(self, args):
        '''parachute control'''
        usage = tr("usage_usage_parachute_enable_disable_release")
        if len(args) != 1:
            print(usage)
            return

        cmds = {
            'enable'  : mavutil.mavlink.PARACHUTE_ENABLE,
            'disable' : mavutil.mavlink.PARACHUTE_DISABLE,
            'release' : mavutil.mavlink.PARACHUTE_RELEASE
        }
        if not args[0] in cmds:
            print(usage)
            return
        cmd = cmds[args[0]]
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            0,
            cmd,
            0, 0, 0, 0, 0, 0)

    def cmd_camctrlmsg(self, args):
        '''camctrlmsg'''

        print(tr("sent_digicam_configure_cmd_long"))
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE, # command
            0, # confirmation
            10, # param1
            20, # param2
            30, # param3
            40, # param4
            50, # param5
            60, # param6
            70) # param7

    def cmd_cammsg(self, args):
        '''cammsg'''

        params = [0, 0, 0, 0, 1, 0, 0]

        # fill in any args passed by user
        for i in range(min(len(args), len(params))):
            params[i] = float(args[i])

        print(tr("sent_digicam_control_cmd_long"))
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL, # command
            0, # confirmation
            params[0], # param1
            params[1], # param2
            params[2], # param3
            params[3], # param4
            params[4], # param5
            params[5], # param6
            params[6]) # param7

    def cmd_engine(self, args):
        '''engine control'''
        if len(args) < 1:
            print(tr("usage_engine_1_0"))
            return
        params = [0, 0, 0, 0, 0, 0, 0]

        if args[0] == 'start':
            args[0] = '1'
        if args[0] == 'stop':
            args[0] = '0'

        # fill in any args passed by user
        for i in range(min(len(args), len(params))):
            params[i] = float(args[i])

        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, # command
            0, # confirmation
            params[0], # param1
            params[1], # param2
            params[2], # param3
            params[3], # param4
            params[4], # param5
            params[5], # param6
            params[6]) # param7

    def cmd_cammsg_old(self, args):
        '''cammsg_old'''

        print(tr("sent_old_digicam_control"))
        self.master.mav.digicam_control_send(
            self.settings.target_system,  # target_system
            self.settings.target_component, # target_component
            0, 0, 0, 0, 1, 0, 0, 0)

    def cmd_do_change_speed(self, args):
        '''speed value'''
        if (len(args) != 1):
            print(tr("usage_setspeed_speed_value"))
            return

        if (len(args) == 1):
            speed = float(args[0])
            print(tr("speed") % (str(speed)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                self.settings.target_component, # target_component
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                0, # confirmation
                0, # param1
                speed, # param2 (Speed value)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

    def cmd_condition_yaw(self, args):
        '''yaw angle angular_speed angle_mode'''
        if (len(args) != 3):
            print(tr("usage_yaw_angle_angular_speed_mode"))
            return

        if (len(args) == 3):
            angle = float(args[0])
            angular_speed = float(args[1])
            angle_mode = float(args[2])
            print(tr("angle") % (str(angle)))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                self.settings.target_component, # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                0, # confirmation
                angle, # param1 (angle value)
                angular_speed, # param2 (angular speed value)
                0, # param3
                angle_mode, # param4 (mode: 0->absolute / 1->relative)
                0, # param5
                0, # param6
                0) # param7

    def cmd_velocity(self, args):
        '''velocity x-ms y-ms z-ms'''
        if (len(args) != 3):
            print(tr("usage_velocity_x_y_z_m"))
            return

        if (len(args) == 3):
            x_mps = float(args[0])
            y_mps = float(args[1])
            z_mps = float(args[2])
            print(tr("x_y_z") % (x_mps, y_mps, z_mps))
            self.master.mav.set_position_target_local_ned_send(
                0,  # system time in milliseconds
                self.settings.target_system,  # target system
                self.settings.target_component,  # target component
                8,  # coordinate frame MAV_FRAME_BODY_NED
                4039,     # type mask (vel only)
                0, 0, 0,  # position x,y,z
                x_mps, y_mps, z_mps,  # velocity x,y,z
                0, 0, 0,  # accel x,y,z
                0, 0)     # yaw, yaw rate

    def cmd_position(self, args):
        '''position x-m y-m z-m'''
        if (len(args) != 3):
            print(tr("usage_position_x_y_z_meters"))
            return

        if (len(args) == 3):
            x_m = float(args[0])
            y_m = float(args[1])
            z_m = float(args[2])
            print(tr("x_y_z") % (x_m, y_m, z_m))
            self.master.mav.set_position_target_local_ned_send(
                0,  # system time in milliseconds
                self.settings.target_system,  # target system
                self.settings.target_component,  # target component
                8,  # coordinate frame MAV_FRAME_BODY_NED
                3576,     # type mask (pos only)
                x_m, y_m, z_m,  # position x,y,z
                0, 0, 0,  # velocity x,y,z
                0, 0, 0,  # accel x,y,z
                0, 0)     # yaw, yaw rate

    def cmd_attitude(self, args):
        '''attitude mask q0 q1 q2 q3 roll_rate pitch_rate yaw_rate thrust'''
        if len(args) < 5:
            print(tr("usage_attitude_q0_q1_q2_q3"))
            print(tr("q0_q1_q2_q3_w_x"))
            print(tr("thrust_0_1"))
            return
        elif len(args) not in [5, 9]:
            print(tr("usage_attitude_mask_q0_q1_q2"))
            print(tr("mask_example_7_0b00000111_ignore_roll"))
            print(tr("mask_example_128_0b10000000_ignore_attitude"))
            print(tr("mask_example_132_0b10000100_ignore_yaw"))
            print(tr("see_https_mavlink_io_en_messages"))
            print(tr("q0_q1_q2_q3_w_x"))
            print(tr("roll_rate_pitch_rate_yaw_rate"))
            print(tr("thrust_0_1"))
            return

        if len(args) == 5:
            mask = 7                    # ignore angular rates
            q0 = float(args[0])
            q1 = float(args[1])
            q2 = float(args[2])
            q3 = float(args[3])
            thrust = float(args[4])
            roll_rate = 0.0
            pitch_rate = 0.0
            yaw_rate = 0.0
            print(tr("q0_q1_q2_q3_thrust") % (q0, q1, q2, q3, thrust))

        elif len(args) == 9:
            mask = int(args[0])
            q0 = float(args[1])
            q1 = float(args[2])
            q2 = float(args[3])
            q3 = float(args[4])
            roll_rate = float(args[5])
            pitch_rate = float(args[6])
            yaw_rate = float(args[7])
            thrust = float(args[8])
            print(tr("mask_q0_q1_q2_q3_roll") %
                  (mask, q0, q1, q2, q3, roll_rate, pitch_rate, yaw_rate, thrust))

        att_target = [q0, q1, q2, q3]
        self.master.mav.set_attitude_target_send(
            0,  # system time in milliseconds
            self.settings.target_system,  # target system
            self.settings.target_component,  # target component
            mask,         # type mask
            att_target,   # quaternion attitude
            math.radians(roll_rate),    # body roll rate
            math.radians(pitch_rate),   # body pitch rate
            math.radians(yaw_rate),     # body yaw rate
            thrust)       # thrust

    def cmd_posvel(self, args):
        '''posvel mapclick vN vE vD'''
        ignoremask = 511
        latlon = None
        latlon = self.mpstate.click_location
        if latlon is None:
            print(tr("set_latlon_to_zeros"))
            latlon = [0, 0]
        else:
            ignoremask = ignoremask & 504
            print(tr("found_latlon"), ignoremask)
        vN = 0
        vE = 0
        vD = 0
        if (len(args) == 3):
            vN = float(args[0])
            vE = float(args[1])
            vD = float(args[2])
            ignoremask = ignoremask & 455

        print(tr("ignoremask"), ignoremask)
        print(latlon)
        self.master.mav.set_position_target_global_int_send(
            0,  # system time in ms
            self.settings.target_system,  # target system
            self.settings.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            ignoremask, # ignore
            int(latlon[0] * 1e7),
            int(latlon[1] * 1e7),
            10,
            vN, vE, vD, # velocity
            0, 0, 0, # accel x,y,z
            0, 0) # yaw, yaw rate

    def cmd_pause(self, args):
        '''pause AUTO/GUIDED modes'''
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,  # command
            0,  # confirmation
            0,  # 0: pause, 1: continue
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

    def cmd_resume(self, args):
        '''resume AUTO/GUIDED modes'''
        self.master.mav.command_long_send(
            self.settings.target_system,  # target_system
            self.settings.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,  # command
            0,  # confirmation
            1,  # 0: pause, 1: continue
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

    def cmd_long(self, args):
        '''execute supplied command long'''
        if len(args) < 1:
            print(tr("usage_long_command_arg1_arg2"))
            return
        command = None
        if args[0].isdigit():
            command = int(args[0])
        else:
            try:
                command = getattr(mavutil.mavlink, args[0])
            except AttributeError:
                try:
                    command = getattr(mavutil.mavlink, "MAV_CMD_" + args[0])
                except AttributeError:
                    pass

        if command is None:
            print(tr("unknown_command_long").format(args[0]))
            return

        if command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            if not args[1].isdigit():
                try:
                    args[1] = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + args[1])
                except AttributeError:
                    pass

        floating_args = [float(x) for x in args[1:]]
        while len(floating_args) < 7:
            floating_args.append(float(0))
        self.master.mav.command_long_send(self.settings.target_system,
                                          self.settings.target_component,
                                          command,
                                          0,
                                          *floating_args)

    def cmd_command_int(self, args):
        '''execute supplied command_int'''
        want_args = 11
        if len(args) != want_args:
            print(tr("argument_count_issue_want_got").format(want_args, len(args)))
            print(tr("usage_command_int_frame_command_current"))
            print(tr("e_g_command_int_global_relative"))
            print(tr("e_g_command_int_global_mav"))
            return

        frame = None
        if args[0].isdigit():
            frame = int(args[0])
        else:
            try:
                # attempt to allow MAV_FRAME_GLOBAL for frame
                frame = getattr(mavutil.mavlink, args[0])
            except AttributeError:
                try:
                    # attempt to allow GLOBAL for frame
                    frame = getattr(mavutil.mavlink, "MAV_FRAME_" + args[0])
                except AttributeError:
                    pass

        if frame is None:
            print(tr("unknown_frame").format(args[0]))
            return

        command = None
        if args[1].isdigit():
            command = int(args[1])
        else:
            # let "command_int ... MAV_CMD_DO_SET_HOME ..." work
            try:
                command = getattr(mavutil.mavlink, args[1])
            except AttributeError:
                try:
                    # let "command_int ... DO_SET_HOME" work
                    command = getattr(mavutil.mavlink, "MAV_CMD_" + args[1])
                except AttributeError:
                    pass

        # current = int(args[2])
        # autocontinue = int(args[3])
        param1 = float(args[4])
        param2 = float(args[5])
        param3 = float(args[6])
        param4 = float(args[7])
        x = int(args[8])
        y = int(args[9])
        z = float(args[10])
        self.master.mav.command_int_send(self.settings.target_system,
                                         self.settings.target_component,
                                         frame,
                                         command,
                                         0,
                                         0,
                                         param1,
                                         param2,
                                         param3,
                                         param4,
                                         x,
                                         y,
                                         z)


def init(mpstate):
    '''initialise module'''
    return CmdlongModule(mpstate)
