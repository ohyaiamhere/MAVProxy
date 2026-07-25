'''
RC min/max setup

AP_FLAKE8_CLEAN
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr


class RCSetupModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RCSetupModule, self).__init__(mpstate, "rcsetup")
        self.calibrating = False
        self.num_channels = 4
        self.clear_rc_cal()
        self.add_command('rccal', self.cmd_rccal, tr("cmd_rc_calibration_start_stop"))
        self.add_command('rctrim', self.cmd_rctrim, tr("cmd_rc_min_max_trim"))
        self.empty_input_count = None
        print(tr("rcsetup_initialised"))

    def clear_rc_cal(self):
        self.rc_cal = []
        self.rc_cal.append("") # 0 will be empty
        for i in range(1, self.num_channels+1):
            # min, max, modified
            self.rc_cal.append([1500, 1500, False])

    def apply_rc_cal(self):
        for i in range(1, len(self.rc_cal)):
            # only apply calibration changes to channels that
            # were modified during calibration
            if self.rc_cal[i][2] is False:
                continue

            self.param_set('RC%u_MIN' % i, self.rc_cal[i][0], 5)
            self.console.writeln(tr("set_rc_u_min_u") % (i, self.rc_cal[i][0]))
            self.param_set('RC%u_MAX' % i, self.rc_cal[i][1], 5)
            self.console.writeln(tr("set_rc_u_max_u") % (i, self.rc_cal[i][1]))

    def get_cal_min(self, channel):
        return self.rc_cal[channel][0]

    def get_cal_max(self, channel):
        return self.rc_cal[channel][1]

    def set_cal_min(self, channel, val):
        self.rc_cal[channel][0] = val
        self.rc_cal[channel][2] = True

    def set_cal_max(self, channel, val):
        self.rc_cal[channel][1] = val
        self.rc_cal[channel][2] = True

    def cmd_rccal(self, args):
        '''start/stop RC calibration'''
        if len(args) < 1:
            self.print_cal_usage()
            return

        if (args[0] == "start"):
            if len(args) > 1:
                self.num_channels = int(args[1])
            print(tr("calibrating_u_channels") % self.num_channels)
            print(tr("warning_remove_propellers_from_electric_planes"))
            print(tr("push_return_when_ready_to_calibrate"))
            self.empty_input_count = self.mpstate.empty_input_count
        elif (args[0] == "done"):
            self.calibrating = False
            self.apply_rc_cal()
        else:
            self.print_cal_usage()

    def idle_task(self):
        if self.empty_input_count is None:
            # not waiting for input
            return
        if self.empty_input_count == self.mpstate.empty_input_count:
            # haven't received input
            return

        self.clear_rc_cal()
        self.calibrating = True
        self.empty_input_count = None

    def cmd_rctrim(self, args):
        '''set RCx_TRIM'''
        if 'RC_CHANNELS' not in self.status.msgs:
            print(tr("no_rc_channels_to_trim_with"))
            return
        m = self.status.msgs['RC_CHANNELS']
        for ch in range(1, 5):
            self.param_set('RC%u_TRIM' % ch, getattr(m, 'chan%u_raw' % ch))

    def unload(self):
        if 'rcreset' in self.mpstate.command_map:
            self.mpstate.command_map.pop('rcreset')
        if 'rctrim' in self.mpstate.command_map:
            self.mpstate.command_map.pop('rctrim')

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        # do nothing if not caibrating
        if self.calibrating is False:
            return

        if m.get_type() == 'RC_CHANNELS':
            for i in range(1, self.num_channels+1):
                v = getattr(m, 'chan%u_raw' % i)

                if self.get_cal_min(i) > v:
                    self.set_cal_min(i, v)
                    self.console.writeln(tr("calibrating_rc_u_min_u") % (i, v))
                if self.get_cal_max(i) < v:
                    self.set_cal_max(i, v)
                    self.console.writeln(tr("calibrating_rc_u_max_u") % (i, v))

    def print_cal_usage(self):
        print(tr("usage_rccal_start_done"))


def init(mpstate):
    '''initialise module'''
    return RCSetupModule(mpstate)
