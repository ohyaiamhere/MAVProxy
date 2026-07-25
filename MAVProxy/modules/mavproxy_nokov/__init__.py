"""
use nokov data to provide motion capture data
it works with nokov software
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_i18n import tr
#from MAVProxy.modules.mavproxy_nokov.nokov import nokovsdk

Descriptor_MarkerSet = 0
Descriptor_RigidBody = 1
Descriptor_Skeleton = 2
Descriptor_ForcePlate = 3

nokov_module = None


def py_data_func(pFrameOfMocapData, userdata):
    if pFrameOfMocapData == None:
        print(tr("not_get_the_data_frame"))
        return
    frameData = pFrameOfMocapData.contents
    names_rigid = nokov_module.names_rigid
    for i in range(frameData.nRigidBodies):
        if i >= len(names_rigid):
            break
        rigid = frameData.RigidBodies[i]
        name = names_rigid[i]
        if nokov_module.nokov_settings.tracker_name == name:
            print(tr("msg_21") % (rigid.x, rigid.y, rigid.z))
            now = time.time()
            time_us = int(now * 1.0e6)
            x = rigid.x / 1000
            y = rigid.y / 1000
            z = rigid.z / 1000
            qx = rigid.qx
            qy = rigid.qy
            qz = rigid.qz
            qw = rigid.qw
            if nokov_module.nokov_settings.axis == 'z':
                nokov_module.master.mav.att_pos_mocap_send(time_us, (qw, qy, qx, -qz), y, x, -z)
            elif nokov_module.nokov_settings.axis == 'y':
                nokov_module.master.mav.att_pos_mocap_send(time_us, (qw, qx, qz, -qy), x, z, -y)
            return


class NokovModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NokovModule, self).__init__(mpstate, "nokov", "nokov")
        global nokov_module
        nokov_module = self
        self.client = None
        self.names_rigid = []
        self.nokov_settings = mp_settings.MPSettings(
            [('host', str, '127.0.0.1'),
             ('axis', str, 'z'),
             ('tracker_name', str, None)]
        )
        self.add_command('nokov', self.cmd_nokov, tr("cmd_nokov_control"), ['<start>', '<stop>', 'set (NOKOVSETTING)'])

    def cmd_stop(self):
        del self.client
        self.client = None
        self.names_rigid = []

    def cmd_start(self):
        if self.client != None:
            print(tr("the_connection_already_exists_please_disconnect"))
            return
        print(tr("serverip_is") % self.nokov_settings.host)
        client = nokovsdk.PySDKClient()
        ver = client.PyNokovVersion()
        print(tr("seekersdk_ver") % (ver[0], ver[1], ver[2], ver[3]))
        client.PySetDataCallback(py_data_func, None)
        ret = client.Initialize(bytes(self.nokov_settings.host, encoding="utf8"))
        if ret == 0:
            print(tr("connect_to_the_seeker_succeed"))
            dsc = nokovsdk.DataDescriptions()
            handle = nokovsdk.c_void_p()
            ret = client.PyGetDataDescriptionsEx(dsc, handle)
            if ret == 0:
                print(tr("getdatadescriptions_succeed"))
                self.parseDescriptions(dsc)
                self.client = client
                client.PyFreeDataDescriptionsEx(handle)
            else:
                print(tr("getdatadescriptions_failed") % ret)
                self.cmd_stop()
        else:
            print(tr("connect_failed") % ret)
            self.cmd_stop()

    def parseDescriptions(self, dsc):
        for i in range(dsc.nDataDescriptions):
            dc = dsc.arrDataDescriptions[i]
            if dc.type == Descriptor_RigidBody:
                name = str(dc.Data.RigidBodyDescription.contents.szName, encoding="utf8")
                self.names_rigid.append(name)
                print(tr("descriptions_rigid"), name)

    def usage(self):
        '''show help on command line options'''
        return "Usage: nokov <start|stop|set>"

    def cmd_nokov(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "start":
            self.cmd_start()
        elif args[0] == "stop":
            self.cmd_stop()
        elif args[0] == "set":
            self.nokov_settings.command(args[1:])
        else:
            print(self.usage())

    def idle_task(self):
        '''called rapidly by mavproxy'''
        pass


def init(mpstate):
    '''initialise module'''
    return NokovModule(mpstate)
