#!/usr/bin/env python3
'''enable run-time addition and removal of UDP clients , just like --out on the cnd line'''
from MAVProxy.modules.lib.mp_i18n import tr
''' TO USE:
    output add 10.11.12.13:14550
    output list
    output remove 3      # to remove 3rd output
'''

from pymavlink import mavutil


from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

class OutputModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OutputModule, self).__init__(mpstate, "output", tr("mod_output_control"), public=True)
        self.add_command('output', self.cmd_output, tr("mod_output_control"),
                         ["<list|add|remove|sysid>"])

    def cmd_output(self, args):
        '''handle output commands'''
        if len(args) < 1 or args[0] == "list":
            self.cmd_output_list()
        elif args[0] == "add":
            if len(args) != 2:
                print(tr("usage_output_add_output"))
                return
            self.cmd_output_add(args[1:])
        elif args[0] == "remove":
            if len(args) != 2:
                print(tr("usage_output_remove_output"))
                return
            self.cmd_output_remove(args[1:])
        elif args[0] == "sysid":
            if len(args) != 3:
                print(tr("usage_output_sysid_sysid_output"))
                return
            self.cmd_output_sysid(args[1:])
        else:
            print(tr("usage_output_list_add_remove_sysid"))

    def cmd_output_list(self):
        '''list outputs'''
        print(tr("u_outputs") % len(self.mpstate.mav_outputs))
        for i in range(len(self.mpstate.mav_outputs)):
            conn = self.mpstate.mav_outputs[i]
            print("%u: %s" % (i, conn.address))
        if len(self.mpstate.sysid_outputs) > 0:
            print(tr("u_sysid_outputs") % len(self.mpstate.sysid_outputs))
            for sysid in self.mpstate.sysid_outputs:
                conn = self.mpstate.sysid_outputs[sysid]
                print("%u: %s" % (sysid, conn.address))

    def cmd_output_add(self, args):
        '''add new output'''
        device = args[0]
        print(tr("adding_output") % device)
        try:
            conn = mavutil.mavlink_connection(device, input=False, source_system=self.settings.source_system, autoreconnect=True)
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print(tr("failed_to_connect_to_2") % device)
            return
        self.mpstate.mav_outputs.append(conn)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass

    def cmd_output_sysid(self, args):
        '''add new output for a specific MAVLink sysID'''
        sysid = int(args[0])
        device = args[1]
        print(tr("adding_output_for_sysid_u") % (device, sysid))
        try:
            conn = mavutil.mavlink_connection(device, input=False, source_system=self.settings.source_system, autoreconnect=True)
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print(tr("failed_to_connect_to_2") % device)
            return
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        if sysid in self.mpstate.sysid_outputs:
            self.mpstate.sysid_outputs[sysid].close()
        self.mpstate.sysid_outputs[sysid] = conn

    def cmd_output_remove(self, args):
        '''remove an output'''
        device = args[0]
        for i in range(len(self.mpstate.mav_outputs)):
            conn = self.mpstate.mav_outputs[i]
            if str(i) == device or conn.address == device:
                print(tr("removing_output") % conn.address)
                try:
                    mp_util.child_fd_list_add(conn.port.fileno())
                except Exception:
                    pass
                conn.close()
                self.mpstate.mav_outputs.pop(i)
                return

    def idle_task(self):
        '''called on idle'''
        for m in self.mpstate.mav_outputs:
            m.source_system = self.settings.source_system
            m.mav.srcSystem = m.source_system
            m.mav.srcComponent = self.settings.source_component

def init(mpstate):
    '''initialise module'''
    return OutputModule(mpstate)
