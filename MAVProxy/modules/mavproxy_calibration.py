#!/usr/bin/env python3
'''calibration command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr

class CalibrationModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CalibrationModule, self).__init__(mpstate, "calibration")
        self.add_command('ground', self.cmd_ground,   tr("cmd_do_a_ground_start"))
        self.add_command('level', self.cmd_level,    tr("cmd_set_level_on_a_multicopter"))
        self.add_command('compassmot', self.cmd_compassmot, tr("cmd_do_compass_motor_interference_calibration"))
        self.add_command('calpress', self.cmd_calpressure,tr("cmd_calibrate_pressure_sensors"))
        self.add_command('accelcal', self.cmd_accelcal, tr("cmd_do_3d_accelerometer_calibration"))
        self.add_command('accelcalsimple', self.cmd_accelcal_simple, tr("cmd_do_simple_accelerometer_calibration"))
        self.add_command('gyrocal', self.cmd_gyrocal, tr("cmd_do_gyro_calibration"))
        self.add_command('ahrstrim', self.cmd_ahrstrim, tr("cmd_do_ahrs_trim"))
        self.add_command('magcal', self.cmd_magcal, tr("cmd_magcal"))
        self.add_command('forcecal', self.cmd_forcecal, tr("cmd_force_calibration_save"))
        self.accelcal_count = -1
        self.accelcal_wait_enter = False
        self.compassmot_running = False
        self.empty_input_count = 0
        self.magcal_progess = []

    def cmd_ground(self, args):
        '''do a ground start mode'''
        self.master.calibrate_imu()

    def cmd_level(self, args):
        '''run a accel level'''
        print(tr("level_is_no_longer_supported_use"))

    def cmd_accelcal(self, args):
        '''do a full 3D accel calibration'''
        mav = self.master
        # ack the APM to begin 3D calibration of accelerometers
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 1, 0, 0)
        self.accelcal_count = 0
        self.accelcal_wait_enter = False

    def cmd_accelcal_simple(self, args):
        '''do a simple accel calibration'''
        mav = self.master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 4, 0, 0)

    def cmd_forcecal(self, args):
        '''force calibration save'''
        usage = tr("usage_usage_forcecal_accel_compass_both")
        if len(args) < 1:
            print(usage)
            return
        param2 = 0
        param5 = 0
        if args[0].lower() == "accel":
            param5 = 76
        elif args[0].lower() == "compass":
            param2 = 76
        elif args[0].lower() == "both":
            param2 = 76
            param5 = 76
        else:
            print(usage)
            return
        mav = self.master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, param2, 0, 0, param5, 0, 0)
        
    def cmd_gyrocal(self, args):
        '''do a full gyro calibration'''
        mav = self.master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  1, 0, 0, 0, 0, 0, 0)

    def cmd_ahrstrim(self, args):
        '''do a AHRS trim'''
        mav = self.master
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 2, 0, 0)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        if self.accelcal_count != -1:
            if mtype == 'STATUSTEXT':
                # handle accelcal packet
                text = str(m.text)
                if text.startswith('Place '):
                    self.accelcal_wait_enter = True
                    self.empty_input_count = self.mpstate.empty_input_count
        if mtype == 'MAG_CAL_PROGRESS':
            while m.compass_id >= len(self.magcal_progess):
                self.magcal_progess.append("")
            self.magcal_progess[m.compass_id] = "%u%%" % m.completion_pct
            self.console.set_status('Progress', 'Calibration Progress: ' + " ".join(self.magcal_progess), row=4)
        elif mtype == 'MAG_CAL_REPORT':
            if m.cal_status == mavutil.mavlink.MAG_CAL_SUCCESS:
                result = "SUCCESS"
            else:
                result = "FAILED"
            self.magcal_progess[m.compass_id] = result
            self.console.set_status('Progress', 'Calibration Progress: ' + " ".join(self.magcal_progess), row=4)
            print(tr("calibration_of_compass_u_fitness") % (m.compass_id, result, m.fitness))
            mav = self.master
            mav.mav.command_long_send(mav.target_system, mav.target_component,
                                      mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, 0,
                                      1<<m.compass_id, 0, 0, 0, 0, 0, 0)

    def idle_task(self):
        '''handle mavlink packets'''
        if self.accelcal_count != -1:
            if self.accelcal_wait_enter and self.empty_input_count != self.mpstate.empty_input_count:
                self.accelcal_wait_enter = False
                self.accelcal_count += 1
                # tell the APM that user has done as requested
                self.master.mav.command_ack_send(self.accelcal_count, 1)
                if self.accelcal_count >= 6:
                    self.accelcal_count = -1

        if self.compassmot_running:
            if self.mpstate.empty_input_count != self.empty_input_count:
                # user has hit enter, stop the process
                    self.compassmot_running = False
                    print(tr("sending_stop"))
                    self.master.mav.command_ack_send(0, 1)


    def cmd_compassmot(self, args):
        '''do a compass/motor interference calibration'''
        mav = self.master
        print(tr("compassmot_starting"))
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                  0, 0, 0, 0, 0, 1, 0)
        self.compassmot_running = True
        self.empty_input_count = self.mpstate.empty_input_count

    def cmd_calpressure(self, args):
        '''calibrate pressure sensors'''
        self.master.calibrate_pressure()

    def print_magcal_usage(self):
        print(tr("usage_magcal_start_accept_cancel_yaw"))

    def cmd_magcal(self, args):
        '''control magnetometer calibration'''
        if len(args) < 1:
            self.print_magcal_usage()
            return

        if args[0] == 'start':
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                0, # target_component
                mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL, # command
                0, # confirmation
                0, # p1: mag_mask
                0, # p2: retry
                1, # p3: autosave
                0, # p4: delay
                0, # param5
                0, # param6
                0) # param7
        elif args[0] == 'accept':
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                0, # target_component
                mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, # command
                0, # confirmation
                0, # p1: mag_mask
                0, # param2
                1, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        elif args[0] == 'cancel':
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                0, # target_component
                mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, # command
                0, # confirmation
                0, # p1: mag_mask
                0, # param2
                1, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        elif args[0] == 'yaw':
            if len(args) < 2:
                print(tr("usage_magcal_yaw_yaw_degrees_mask"))
                return
            yaw_deg = float(args[1])
            mask = 0
            if len(args) > 2:
                mask = int(args[2])
            print(tr("calibrating_for_yaw_degrees_with_mask") % (yaw_deg, mask))
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                0, # target_component
                mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW, # command
                0, # confirmation
                yaw_deg, # p1: yaw in degrees
                mask, # p2: mask
                0, # p3: lat_deg
                0, # p4: lon_deg
                0, # param5
                0, # param6
                0) # param7
        else:
            self.print_magcal_usage()
            return

def init(mpstate):
    '''initialise module'''
    return CalibrationModule(mpstate)
