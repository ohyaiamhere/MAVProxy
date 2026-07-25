#!/usr/bin/env python3
'''
control MAVLink2 signing
'''

from pymavlink import mavutil
import time, struct, math, sys
import os

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib.mp_i18n import tr

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class SigningModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SigningModule, self).__init__(mpstate, "signing", tr("mod_signing_control"), public=True)
        self.add_command('signing', self.cmd_signing, tr("mod_signing_control"),
                         ["<setup|remove|disable|key>"])
        self.allow = None
        self.saved_key = None
        self.last_timestamp_update = time.time()

    def cmd_signing(self, args):
        '''handle link commands'''
        usage = tr("usage_signing_setup_remove_disable_key_passphrase")
        if len(args) == 0:
            print(usage)
        elif args[0] == 'setup':
            self.cmd_signing_setup(args[1:])
        elif args[0] == 'key':
            self.cmd_signing_key(args[1:])
        elif args[0] == 'disable':
            self.cmd_signing_disable(args[1:])
        elif args[0] == 'remove':
            self.cmd_signing_remove(args[1:])
        else:
            print(usage)

    def passphrase_to_key(self, passphrase):
        '''convert a passphrase to a 32 byte key'''
        import hashlib
        h = hashlib.new('sha256')
        if sys.version_info[0] >= 3:
            passphrase = passphrase.encode('ascii')
        h.update(passphrase)
        return h.digest()

    def cmd_signing_setup(self, args):
        '''setup signing key on board'''
        if len(args) == 0:
            print(tr("usage_signing_setup_passphrase"))
            return
        if not self.master.mavlink20():
            print(tr("you_must_be_using_mavlink2_for"))
            return
        passphrase = args[0]
        key = self.passphrase_to_key(passphrase)
        secret_key = []
        for b in key:
            if sys.version_info[0] >= 3:
                secret_key.append(b)
            else:
                secret_key.append(ord(b))

        initial_timestamp = self.get_signing_timestamp()
        self.master.mav.setup_signing_send(self.target_system, self.target_component,
                                           secret_key, initial_timestamp)
        print(tr("sent_secret_key"))
        self.cmd_signing_key([passphrase])

    def get_signing_timestamp(self):
        '''get a timestamp from current clock in units for signing'''
        epoch_offset = 1420070400
        now = max(time.time(), epoch_offset)
        return int((now - epoch_offset)*1e5)

    def allow_unsigned(self, mav, msgId):
        '''see if an unsigned packet should be allowed'''
        if self.allow is None:
            self.allow = {
                mavutil.mavlink.MAVLINK_MSG_ID_RADIO : True,
                mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS : True
                }
        if msgId in self.allow:
            return True
        if self.settings.allow_unsigned:
            return True
        return False

    def setup_signing_link(self, m):
        '''add signing to a link'''
        if self.saved_key is not None:
            m.setup_signing(self.saved_key, sign_outgoing=True, allow_unsigned_callback=self.allow_unsigned)

    def find_signing_passphrase(self, device):
        '''
        look for a signing passphrase in ~/.mavproxy/signing.keys
        file format is lines of form "device passphrase"
        an entry of the form "host:port" (no protocol) matches a device
        of the form "proto:host:port" for any protocol
        '''
        path = mp_util.dot_mavproxy("signing.keys")
        try:
            lines = open(path,'r').readlines()
        except Exception:
            return None
        device = device.lower()
        device_parts = device.split(':')
        for line in lines:
            a = line.split()
            if len(a) != 2:
                continue
            entry = a[0].lower()
            entry_parts = entry.split(':')
            if len(entry_parts) == 3:
                # "proto:host:port" - exact match against device
                if entry == device:
                    return a[1]
            elif len(entry_parts) == 2:
                # "host:port" - match the host:port of device, any protocol
                if len(device_parts) == 3 and entry_parts == device_parts[1:]:
                    return a[1]
                if len(device_parts) == 2 and entry_parts == device_parts:
                    return a[1]
            else:
                # other forms (e.g. serial device "/dev/ttyUSB0") - exact match
                if entry == device:
                    return a[1]
        return None

    def setup_signing_device(self, m, device):
        '''add signing to a link, with a device name'''
        passphrase = self.find_signing_passphrase(device)
        key = self.saved_key
        if passphrase:
            key = self.passphrase_to_key(passphrase)
        if key is not None:
            m.setup_signing(key, sign_outgoing=True, allow_unsigned_callback=self.allow_unsigned)
            
    def cmd_signing_key(self, args):
        '''set signing key on connection'''
        if len(args) == 0:
            print(tr("usage_signing_setup_passphrase"))
            return
        if not self.master.mavlink20():
            print(tr("you_must_be_using_mavlink2_for"))
            return
        passphrase = args[0]
        key = self.passphrase_to_key(passphrase)
        self.saved_key = key
        for m in self.mpstate.mav_master:
            self.setup_signing_link(m)
        print(tr("setup_signing_key"))

    def cmd_signing_disable(self, args):
        '''disable signing locally'''
        self.saved_key = None
        self.master.disable_signing()
        print(tr("disabled_signing"))

    def cmd_signing_remove(self, args):
        '''remove signing from server'''
        self.saved_key = None
        if not self.master.mavlink20():
            print(tr("you_must_be_using_mavlink2_for"))
            return
        self.master.mav.setup_signing_send(self.target_system, self.target_component, [0]*32, 0)
        self.master.disable_signing()
        print(tr("removed_signing"))

    def idle_task(self):
        now = time.time()
        # every 10 seconds ensure our signing timestamps are at least the
        # current clock, this ensures if we have an idle link we don't fall
        # behind in the timestamp too far and send packets that will be
        # rejected
        if now - self.last_timestamp_update > 10:
            self.last_timestamp_update = now
            signing_timestamp = self.get_signing_timestamp()
            for m in self.mpstate.mav_master:
                if m.mav.signing and m.mav.signing.sign_outgoing:
                    m.mav.signing.timestamp = max(m.mav.signing.timestamp, signing_timestamp)

def init(mpstate):
    '''initialise module'''
    return SigningModule(mpstate)
