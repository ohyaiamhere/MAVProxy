#!/usr/bin/env python3
'''tune command handling'''

import time, os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr

tune_options = {
    'None':             '0',
    'StabRollPitchkP':  '1',
    'RateRollPitchkP':  '4',
    'RateRollPitchkI':  '5',
    'RateRollPitchkD':  '21',
    'StabYawkP':        '3',
    'RateYawkP':        '6',
    'RateYawkD':        '26',
    'AltitudeHoldkP':   '14',
    'ThrottleRatekP':   '7',
    'ThrottleAccelkP':  '34',
    'ThrottleAccelkI':  '35',
    'ThrottleAccelkD':  '36',
    'LoiterSpeed':      '42',
    'LoiterPoskP':      '12',
    'VelocityXYkP':     '22',
    'VelocityXYkI':     '28',
    'WPSpeed':          '10',
    'AcroRollPitchkP':  '25',
    'AcroYawkP':        '40',
    'HeliExtGyro':      '13',
    'OFLoiterkP':       '17',
    'OFLoiterkI':       '18',
    'OFLoiterkD':       '19',
    'Declination':      '38',
    'CircleRate':       '39',
    'RangeFinderGain':  '41',
    'RatePitchkP':      '46',
    'RatePitchkI':      '47',
    'RatePitchkD':      '48',
    'RateRollkP':       '49',
    'RateRollkI':       '50',
    'RateRollkD':       '51',
    'RatePitchFF':      '52',
    'RateRollFF':       '53',
    'RateYawFF':        '54',
}

class TuneoptModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TuneoptModule, self).__init__(mpstate, "tuneopt", tr("mod_tuneopt_command_handling"))
        self.add_command('tuneopt', self.cmd_tuneopt,  tr("cmd_select_option_for_tune_pot_on_channel_6"))

    def tune_show(self):
        opt_num = str(int(self.get_mav_param('TUNE')))
        option = None
        for k in tune_options.keys():
            if opt_num == tune_options[k]:
                option = k
                break
        else:
            print("TUNE is currently set to unknown value " + opt_num)
            return
        low = self.get_mav_param('TUNE_LOW')
        high = self.get_mav_param('TUNE_HIGH')
        print(tr("tune_is_currently_set_to_low") % (option, low/1000, high/1000))

    def tune_option_validate(self, option):
        for k in tune_options:
            if option.upper() == k.upper():
                return k
        return None

    # TODO: Check/show the limits of LOW and HIGH
    def cmd_tuneopt(self, args):
        '''Select option for Tune Pot on Channel 6 (quadcopter only)'''
        usage = tr("usage_usage_tuneopt_set_show_reset_list")
        if self.mpstate.vehicle_type != 'copter':
            print(tr("this_command_is_only_available_for"))
            return
        if len(args) < 1:
            print(usage)
            return
        if args[0].lower() == 'reset':
            self.param_set('TUNE', '0')
        elif args[0].lower() == 'set':
            if len(args) < 4:
                print(tr("usage_tuneopt_set_option_low_high"))
                return
            option = self.tune_option_validate(args[1])
            if not option:
                print('Invalid Tune option: ' + args[1])
                return
            low = args[2]
            high = args[3]
            self.param_set('TUNE', tune_options[option])
            self.param_set('TUNE_LOW', float(low) * 1000)
            self.param_set('TUNE_HIGH', float(high) * 1000)
        elif args[0].lower() == 'show':
            self.tune_show()
        elif args[0].lower() == 'list':
            print(tr("options_available"))
            for s in sorted(tune_options.keys()):
                print('  ' + s)
        else:
            print(usage)

def init(mpstate):
    '''initialise module'''
    return TuneoptModule(mpstate)
