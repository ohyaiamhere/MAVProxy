'''
Support for MAVLink SECURE_COMMAND
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import time
import base64
import struct
import random
import glob
from MAVProxy.modules.lib.mp_i18n import tr
try:
    import monocypher
except ImportError:
    print(tr("please_install_monocypher_with_python3_m"))

class SecureCommandModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SecureCommandModule, self).__init__(mpstate, "SecureCommand", tr("mod_securecommand_support"), public = True)
        self.add_command('securecommand', self.cmd_securecommand, tr("cmd_securecommand_control"),
                         ["<getsessionkey|getpublickeys|setpublickeys|removepublickeys|setconfig>", "set (SECURECOMMANDSETTING)"])

        from MAVProxy.modules.lib.mp_settings import MPSetting
        self.SecureCommand_settings = mp_settings.MPSettings([
            MPSetting("private_keyfile", str, None),
            ])
        self.add_completion_function('(SECURECOMMANDSETTING)',
                                     self.SecureCommand_settings.completion)
        self.session_key = None
        self.public_keys = [None]*10
        self.sequence = random.randint(0, 0xFFFFFFFF)
        self.sent_sequence = None

    def cmd_securecommand(self, args):
        '''securecommand command parser'''
        usage = tr("usage_usage_securecommand_set_getsessionkey_getpublickeys_set")
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.SecureCommand_settings.command(args[1:])
        elif args[0] == "getsessionkey":
            self.cmd_getsessionkey()
        elif args[0] == "getpublickeys":
            self.cmd_getpublickeys(args[1:])
        elif args[0] == "setpublickeys":
            self.cmd_setpublickeys(args[1:])
        elif args[0] == "removepublickeys":
            self.cmd_removepublickeys(args[1:])
        elif args[0] == "setconfig":
            self.cmd_setconfig(args[1:])
        else:
            print(usage)

    def advance_sequence(self):
        '''add one to sequence'''
        self.sequence = (self.sequence+1) & 0xFFFFFFFF

    def get_private_key(self):
        '''get private key, return 32 byte key or None'''
        if self.SecureCommand_settings.private_keyfile is None:
            return None
        try:
            d = open(self.SecureCommand_settings.private_keyfile,'r').read()
        except Exception as ex:
            return None
        ktype = "PRIVATE_KEYV1:"
        if not d.startswith(ktype):
            return None
        return base64.b64decode(d[len(ktype):])

    def read_public_key(self, keyfile):
        '''read a public key, return 32 byte key or None'''
        try:
            d = open(keyfile,'r').read()
        except Exception as ex:
            return None
        ktype = "PUBLIC_KEYV1:"
        if not d.startswith(ktype):
            return None
        return base64.b64decode(d[len(ktype):])
    
    def have_private_key(self):
        '''return true if we have a valid private key'''
        return self.get_private_key() is not None

    def make_signature(self, seq, command, data):
        '''make a signature'''
        private_key = self.get_private_key()
        d = struct.pack("<II", seq, command)
        d += data
        if command != mavutil.mavlink.SECURE_COMMAND_GET_SESSION_KEY:
            if self.session_key is None:
                print(tr("no_session_key"))
                raise Exception("No session key")
            d += self.session_key
        self.sent_sequence = seq
        return monocypher.signature_sign(private_key, d)

    def pad_data(self, data, dlen=220):
        '''pad data with 0x00 to given length'''
        clen = len(data)
        plen = dlen-clen
        return data + bytearray([0]*plen)

    def cmd_getsessionkey(self):
        '''request session key'''
        if not self.have_private_key():
            print(tr("no_private_key_set"))
            return
        sig = self.make_signature(self.sequence, mavutil.mavlink.SECURE_COMMAND_GET_SESSION_KEY, bytes())
        self.master.mav.secure_command_send(self.target_system, self.target_component,
                                            self.sequence, mavutil.mavlink.SECURE_COMMAND_GET_SESSION_KEY,
                                            0, len(sig), self.pad_data(sig))
        self.advance_sequence()

    def cmd_getpublickeys(self, args):
        '''get public keys'''
        if not self.have_private_key():
            print(tr("no_private_key_set"))
            return
        if not self.session_key:
            print(tr("no_session_key"))
            return
        idx = 0
        nkeys = 6
        if len(args) > 0:
            idx = int(args[0])
        if len(args) > 1:
            nkeys = int(args[1])
        req = struct.pack("<BB", idx, nkeys)
        sig = self.make_signature(self.sequence, mavutil.mavlink.SECURE_COMMAND_GET_PUBLIC_KEYS, req)
        self.master.mav.secure_command_send(self.target_system, self.target_component,
                                            self.sequence, mavutil.mavlink.SECURE_COMMAND_GET_PUBLIC_KEYS,
                                            len(req), len(sig), self.pad_data(req+sig))
        self.advance_sequence()

    def cmd_removepublickeys(self, args):
        '''remove public keys'''
        if not self.have_private_key():
            print(tr("no_private_key_set"))
            return
        if not self.session_key:
            print(tr("no_session_key"))
            return
        if len(args) != 2:
            print(tr("usage_removepublickeys_index_count"))
            return
        idx = int(args[0])
        nkeys = int(args[1])
        req = struct.pack("<BB", idx, nkeys)
        sig = self.make_signature(self.sequence, mavutil.mavlink.SECURE_COMMAND_REMOVE_PUBLIC_KEYS, req)
        self.master.mav.secure_command_send(self.target_system, self.target_component,
                                            self.sequence, mavutil.mavlink.SECURE_COMMAND_REMOVE_PUBLIC_KEYS,
                                            len(req), len(sig), self.pad_data(req+sig))
        self.advance_sequence()
        
    def cmd_setpublickeys(self, args):
        '''set public keys'''
        if not self.have_private_key():
            print(tr("no_private_key_set"))
            return
        if not self.session_key:
            print(tr("no_session_key"))
            return
        if len(args) < 2:
            print(tr("usage_setpublickeys_keyindex_keyfiles"))
            return
        idx = int(args[0])
        keys = []
        for kfile in args[1:]:
            for fname in sorted(glob.glob(kfile)):
                k = self.read_public_key(fname)
                if k is None:
                    print(tr("unable_to_load_keyfile") % fname)
                    return
                print(tr("loaded_key") % fname)
                keys.append(k)
        if len(keys) > 6:
            print(tr("too_many_keys_u_max_is") % len(keys))
            return
        if len(keys) == 0:
            print(tr("no_keys_found"))
            return
        req = struct.pack("<B", idx)
        for k in keys:
            req += k
        sig = self.make_signature(self.sequence, mavutil.mavlink.SECURE_COMMAND_SET_PUBLIC_KEYS, req)
        self.master.mav.secure_command_send(self.target_system, self.target_component,
                                            self.sequence, mavutil.mavlink.SECURE_COMMAND_SET_PUBLIC_KEYS,
                                            len(req), len(sig), self.pad_data(req+sig))
        print(tr("sent_u_public_keys_starting_at") % (len(keys), idx))
        self.advance_sequence()

    def cmd_setconfig(self, args):
        '''set configuration parameters'''
        if not self.have_private_key():
            print(tr("no_private_key_set"))
            return
        if not self.session_key:
            print(tr("no_session_key"))
            return
        if len(args) < 1:
            print(tr("usage_setconfig_param_value"))
            return
        req = bytearray()
        for i in range(len(args)):
            p = args[i]
            req += p.encode('utf-8')
            if i < len(args)-1:
                req += bytearray([0])
        sig = self.make_signature(self.sequence, mavutil.mavlink.SECURE_COMMAND_SET_REMOTEID_CONFIG, req)
        self.master.mav.secure_command_send(self.target_system, self.target_component,
                                            self.sequence, mavutil.mavlink.SECURE_COMMAND_SET_REMOTEID_CONFIG,
                                            len(req), len(sig), self.pad_data(req+sig))
        print(tr("sent_u_config_commands") % len(args))
        self.advance_sequence()
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == "SECURE_COMMAND_REPLY":
            if m.sequence != self.sent_sequence:
                print(tr("invalid_reply_sequence"))
                return
            m.sent_sequence = None

            if m.operation == mavutil.mavlink.SECURE_COMMAND_GET_SESSION_KEY:
                if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.session_key = bytearray(m.data[:m.data_length])
                    print(tr("got_session_key_length_u") % len(self.session_key))
                else:
                    print(tr("get_session_key_failed_u") % m.result)

            if m.operation == mavutil.mavlink.SECURE_COMMAND_GET_PUBLIC_KEYS:
                if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    idx = m.data[0]
                    if idx >= len(self.public_keys):
                        print(tr("invalid_key_index_u") % idx)
                        return
                    keys = bytearray(m.data[1:m.data_length])
                    numkeys = len(keys) // 32
                    if numkeys == 0:
                        print(tr("no_public_keys_returned"))
                        return
                    for i in range(idx, numkeys):
                        self.public_keys[i] = keys[32*i:32*(i+1)]
                        keyfile = "public_key%u.dat" % i
                        open(keyfile, "w").write("PUBLIC_KEYV1:" + base64.b64encode(self.public_keys[i]).decode('utf-8') + "\n")
                        print(tr("wrote") % keyfile)
                    print(tr("got_public_keys_u_to_u") % (idx, numkeys+idx-1))
                else:
                    print(tr("get_public_keys_failed_u") % m.result)

            if m.operation == mavutil.mavlink.SECURE_COMMAND_SET_PUBLIC_KEYS:
                if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(tr("set_public_keys_ok"))
                else:
                    print(tr("set_public_keys_failed_u") % m.result)

            if m.operation == mavutil.mavlink.SECURE_COMMAND_REMOVE_PUBLIC_KEYS:
                if m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(tr("remove_public_keys_ok"))
                else:
                    print(tr("remove_public_keys_failed_u") % m.result)
                    
def init(mpstate):
    '''initialise module'''
    return SecureCommandModule(mpstate)
