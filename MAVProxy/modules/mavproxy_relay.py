#!/usr/bin/env python3
'''relay handling module'''

import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr

class RelayModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RelayModule, self).__init__(mpstate, "relay")
        self.add_command('relay', self.cmd_relay, tr("cmd_relay_commands"))
        self.add_command('servo', self.cmd_servo, tr("cmd_servo_commands"))
        self.add_command('motortest', self.cmd_motortest, tr("cmd_motortest_commands"))

    def cmd_relay(self, args):
        '''set relays'''
        if len(args) == 0 or args[0] not in ['set', 'repeat']:
            print(tr("usage_relay_set_repeat"))
            return
        if args[0] == "set":
            if len(args) < 3:
                print(tr("usage_relay_set_relay_num_0"))
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
                                                   int(args[1]), int(args[2]),
                                                   0, 0, 0, 0, 0)
        if args[0] == "repeat":
            if len(args) < 4:
                print(tr("usage_relay_repeat_relay_num_count"))
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY, 0,
                                                   int(args[1]), int(args[2]), float(args[3]),
                                                   0, 0, 0, 0)

    def cmd_servo(self, args):
        '''set servos'''
        if len(args) == 0 or args[0] not in ['set', 'repeat']:
            print(tr("usage_servo_set_repeat"))
            return
        if args[0] == "set":
            if len(args) < 3:
                print(tr("usage_servo_set_servo_num_pwm"))
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                                   int(args[1]), int(args[2]),
                                                   0, 0, 0, 0, 0)
        if args[0] == "repeat":
            if len(args) < 5:
                print(tr("usage_servo_repeat_servo_num_pwm"))
                return
            self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0,
                                                   int(args[1]), int(args[2]), int(args[3]), float(args[4]),
                                                   0, 0, 0)


    def cmd_motortest(self, args):
        '''run motortests on copter'''
        if len(args) < 4:
            print(tr("usage_motortest_motor_test_sequence_number"))
            return
        if len(args) == 5:
            count = int(args[4])
        else:
            count = 0
        self.master.mav.command_long_send(self.target_system,
                                          0,
                                          mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                          int(args[0]), int(args[1]), float(args[2]), int(args[3]), count,
                                          0, 0)


def init(mpstate):
    '''initialise module'''
    return RelayModule(mpstate)
