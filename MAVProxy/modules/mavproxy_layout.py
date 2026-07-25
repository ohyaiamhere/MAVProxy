#!/usr/bin/env python3
'''window layout command handling'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_i18n import tr

class LayoutModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(LayoutModule, self).__init__(mpstate, "layout", tr("mod_window_layout_handling"), public = False)
        self.add_command('layout', self.cmd_layout,
                         tr("cmd_window_layout_management"),
                         ["<save|load>"])

    def cmd_layout(self, args):
        '''handle layout command'''
        from MAVProxy.modules.lib import win_layout
        if len(args) < 1:
            print(tr("usage_layout_save_load"))
            return
        if args[0] == "load":
            win_layout.load_layout(self.mpstate.settings.vehicle_name)
        elif args[0] == "save":
            win_layout.save_layout(self.mpstate.settings.vehicle_name)

def init(mpstate):
    '''initialise module'''
    return LayoutModule(mpstate)
