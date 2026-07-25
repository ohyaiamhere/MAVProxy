#!/usr/bin/env python3
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

AP_FLAKE8_CLEAN
'''

import fnmatch
import glob
import json
import os
import platform
import select
import serial
import shlex
import signal
import socket
import struct
import sys
import threading
import time
import traceback

# When this file is run as a script (python MAVProxy/mavproxy.py), sys.path[0]
# is the MAVProxy/ package directory, so an installed site-packages copy of
# MAVProxy would be imported instead of this source tree. Prefer the local
# checkout so new modules (e.g. mp_i18n) are found.
_mp_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _mp_repo_root not in sys.path:
    sys.path.insert(0, _mp_repo_root)

from importlib import reload

from pymavlink import mavutil

from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_substitute
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.mavproxy_link import preferred_ports
from MAVProxy.modules.lib.mp_i18n import (
    tr, init as i18n_init, set_language, get_language, available_languages,
    early_language_from_argv, I18nOptionParser,
)

# adding all this allows pyinstaller to build a working windows executable
# note that using --hidden-import does not work for these modules
try:
    multiproc.freeze_support()
    from pymavlink import mavwp  # noqa
    import matplotlib  # noqa
    import HTMLParser  # noqa
except Exception:
    pass

# screensaver dbus syntax swiped from
# https://stackoverflow.com/questions/10885337/inhibit-screensaver-with-python
screensaver_interface = None
screensaver_cookie = None
try:
    import atexit
    import dbus
    bus = dbus.SessionBus()
    saver = bus.get_object('org.freedesktop.ScreenSaver', '/ScreenSaver')
    screensaver_interface = dbus.Interface(saver, dbus_interface='org.freedesktop.ScreenSaver')
    if screensaver_cookie is not None:
        atexit.register(screensaver_interface.UnInhibit, [screensaver_cookie])
except Exception:
    pass

if __name__ == '__main__':
    multiproc.freeze_support()

# The MAVLink version being used (None, "1.0", "2.0")
mavversion = None
mpstate = None


class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        self.gps = None
        self.msgs = {}
        self.msg_count = {}
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.bytecounters = {'MasterIn': []}
        self.setup_mode = opts.setup
        self.mav_error = 0
        self.altitude = 0
        self.last_distance_announce = 0.0
        self.stop_event = threading.Event()
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        self.last_mode_announced = 'MAV'
        self.logdir = None
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = False
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.statustexts_by_sysidcompid = {}
        self.highest_msec = {}
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        self.watch = None
        self.watch_verbose = False
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        self.last_seq = 0
        self.armed = False
        self.last_bytecounter_calc = 0

    class ByteCounter(object):
        def __init__(self):
            self.total_count = 0
            self.current_count = 0
            self.buckets = []
            self.max_buckets = 10  # 10 seconds

        def update(self, bytecount):
            self.total_count += bytecount
            self.current_count += bytecount

        def rotate(self):
            '''move current count into a bucket, zero count'''
            # huge assumption made that we're called rapidly enough to
            # not need to rotate multiple buckets.
            self.buckets.append(self.current_count)
            self.current_count = 0
            if len(self.buckets) > self.max_buckets:
                self.buckets = self.buckets[-self.max_buckets:]

        def rate(self):
            if len(self.buckets) == 0:
                return 0
            total = 0
            for bucket in self.buckets:
                total += bucket
            return total/float(len(self.buckets))

        def total(self):
            return self.total_count

    def update_bytecounters(self):
        '''rotate bytecounter buckets if required'''
        now = time.time()
        time_delta = now - self.last_bytecounter_calc
        if time_delta < 1:
            return
        self.last_bytecounter_calc = now

        for counter in self.bytecounters['MasterIn']:
            counter.rotate()

    def show(self, f, pattern=None, verbose=False):
        '''write status to status.txt'''
        if pattern is None:
            f.write('Counters: ')
            for c in self.counters:
                f.write('%s:%s ' % (c, self.counters[c]))
            f.write('\n')
            f.write('MAV Errors: %u\n' % self.mav_error)
            f.write(str(self.gps)+'\n')
        for m in sorted(self.msgs.keys()):
            if pattern is not None:
                if not fnmatch.fnmatch(str(m).upper(), pattern.upper()):
                    continue
                if getattr(self.msgs[m], '_instance_field', None) is not None and m.find('[') == -1 and pattern.find('*') != -1:  # noqa:E501
                    # only show instance versions for patterns
                    continue
            msg = None
            sysid = mpstate.settings.target_system
            for mav in mpstate.mav_master:
                if sysid not in mav.sysid_state:
                    continue
                if m not in mav.sysid_state[sysid].messages:
                    continue
                msg2 = mav.sysid_state[sysid].messages[m]
                if msg is None or msg2._timestamp > msg._timestamp:
                    msg = msg2
            if msg is None:
                continue
            if verbose:
                try:
                    mavutil.dump_message_verbose(f, msg)
                    f.write("\n")
                except AttributeError as e:
                    if "has no attribute 'dump_message_verbose'" in str(e):
                        print(tr("pymavlink_update_required_for_verbose"))
                    else:
                        raise e
            else:
                f.write("%u: %s\n" % (self.msg_count[m], str(msg)))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()


def say_text(text, priority='important'):
    '''text output - default function for say()'''
    mpstate.console.writeln(text)


def say(text, priority='important'):
    '''text and/or speech output'''
    mpstate.functions.say(text, priority)


def add_input(cmd, immediate=False):
    '''add some command input to be processed'''
    if immediate:
        process_stdin(cmd)
    else:
        mpstate.input_queue.put(cmd)


class MAVFunctions(object):
    '''core functions available in modules'''
    def __init__(self):
        self.process_stdin = add_input
        self.param_set = param_set
        self.get_mav_param = get_mav_param
        self.say = say_text
        # input handler can be overridden by a module
        self.input_handler = None


class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.command_map = None
        self.map = None
        self.map_functions = {}
        self.click_location = None
        self.click_time = None
        self.vehicle_type = None
        self.vehicle_name = None
        self.aircraft_dir = None
        self.logqueue_raw = None
        self.logqueue = None
        self.rl = None
        self.input_queue = None
        self.input_count = None
        self.empty_input_count = None
        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        self.settings = MPSettings([
            MPSetting('link', int, 1, tr("set_primary_link"), tab=tr("set_link"), range=(0, 100), increment=1),
            MPSetting('streamrate', int, 4, tr("set_stream_rate_link1"), range=(-1, 500), increment=1),
            MPSetting('streamrate2', int, 4, tr("set_stream_rate_link2"), range=(-1, 500), increment=1),
            MPSetting('heartbeat', float, 1, tr("set_heartbeat_rate_hz"), range=(0, 100), increment=0.1),
            MPSetting('mavfwd', bool, True, tr("set_allow_forwarded_control")),
            MPSetting('mavfwd_disarmed', bool, True, tr("set_allow_forwarded_control_when_disarmed")),
            MPSetting('mavfwd_rate', bool, False, tr("set_allow_forwarded_rate_control")),
            MPSetting('mavfwd_link', int, -1, tr("set_forward_to_a_specific_link")),
            MPSetting('mavfwd_signing', bool, True, tr("set_sign_forwarded_messages")),
            MPSetting('shownoise', bool, True, tr("set_show_non_mavlink_data")),
            MPSetting('baudrate', int, opts.baudrate, tr("set_baudrate_for_new_links"), range=(0, 10000000), increment=1),
            MPSetting('rtscts', bool, opts.rtscts, tr("set_enable_flow_control")),
            MPSetting('select_timeout', float, 0.01, tr("set_select_timeout")),

            MPSetting('altreadout', int, 10, tr("set_altitude_readout"),
                      range=(0, 100), increment=1, tab=tr("set_announcements")),
            MPSetting('distreadout', int, 200, tr("set_distance_readout"), range=(0, 10000), increment=1),

            MPSetting('moddebug', int, opts.moddebug, tr("set_module_debug_level"), range=(0, 4), increment=1, tab=tr("set_debug")),
            MPSetting('script_fatal', bool, False, tr("set_fatal_error_on_bad_script"), tab=tr("set_debug")),
            MPSetting('compdebug', int, 0, tr("set_computation_debug_mask"), range=(0, 3), tab=tr("set_debug")),
            MPSetting('flushlogs', bool, False, tr("set_flush_logs_on_every_packet")),
            MPSetting('requireexit', bool, False, tr("set_require_exit_command")),
            MPSetting('wpupdates', bool, True, tr("set_announce_waypoint_updates")),
            MPSetting('wpterrainadjust', bool, True, tr("set_adjust_alt_of_moved_wp_using_terrain")),
            MPSetting('wp_use_mission_int', bool, True, tr("set_use_mission_item_int_messages")),
            MPSetting('wp_use_waypoint_set_current', bool, False, tr("set_use_deprecated_waypoint_set_current_message")),

            MPSetting('basealt', int, 0, tr("set_base_altitude"), range=(0, 30000), increment=1, tab=tr("set_altitude")),
            MPSetting('wpalt', int, 100, tr("set_default_wp_altitude"), range=(0, 10000), increment=1),
            MPSetting('rallyalt', int, 90, tr("set_default_rally_altitude"), range=(0, 10000), increment=1),
            MPSetting('terrainalt', str, 'Auto', tr("set_use_terrain_altitudes"), choice=['Auto', 'True', 'False']),
            MPSetting('guidedalt', int, 100, tr("set_default_fly_to_altitude"), range=(0, 10000), increment=1),
            MPSetting('guided_use_reposition', bool, True, tr("set_use_mav_cmd_do_reposition_for_guided_fly")),
            MPSetting('rally_breakalt', int, 40, tr("set_default_rally_break_altitude"), range=(0, 10000), increment=1),
            MPSetting('rally_flags', int, 0, tr("set_default_rally_flags"), range=(0, 10000), increment=1),

            MPSetting('source_system', int, 255, tr("set_mavlink_source_system"), range=(0, 255), increment=1, tab=tr("set_mavlink")),
            MPSetting('source_component', int, 230, tr("set_mavlink_source_component"), range=(0, 255), increment=1),
            MPSetting('target_system', int, 0, tr("set_mavlink_target_system"), range=(0, 255), increment=1),
            MPSetting('target_component', int, 0, tr("set_mavlink_target_component"), range=(0, 255), increment=1),
            MPSetting('state_basedir', str, None, tr("set_base_directory_for_logs_and_aircraft_directories")),
            MPSetting('allow_unsigned', bool, True, tr("set_whether_unsigned_packets_will_be_accepted")),

            MPSetting('dist_unit', str, 'm', tr("set_distance_unit"), choice=['m', 'nm', 'miles'], tab=tr("set_units")),
            MPSetting('height_unit', str, 'm', tr("set_height_unit"), choice=['m', 'feet']),
            MPSetting('speed_unit', str, 'm/s', tr("set_height_unit"), choice=['m/s', 'knots', 'mph']),
            MPSetting('flytoframe', str, 'AboveHome', tr("set_frame_for_flyto"), choice=['AboveHome', 'AGL', 'AMSL']),

            MPSetting('fwdpos', bool, False, tr("set_forward_global_position_int_on_all_links")),
            MPSetting('checkdelay', bool, True, tr("set_check_for_link_delay")),
            MPSetting('param_ftp', bool, True, tr("set_try_ftp_for_parameter_download")),
            MPSetting('param_docs', bool, True, tr("set_show_help_for_parameters")),

            MPSetting('vehicle_name', str, '', tr("set_vehicle_name"), tab=tr("set_vehicle")),

            MPSetting('all_vehicle_command_acks', bool, True, tr("set_show_command_acks_even_if_they_re_targetted")),

            MPSetting('sys_status_error_warn_interval', int, 30, tr("set_interval_to_warn_of_autopilot_software_failure")),

            MPSetting('inhibit_screensaver_when_armed', bool, False, tr("set_inhibit_screensaver_while_vehicle_armed")),

            MPSetting('timeout', int, 5, tr("set_number_of_seconds_with_no_packets_for_a"), range=(0, 255), increment=1),  # noqa

            MPSetting('language', str, 'en', tr("set_ui_language_code")),
        ])

        self.completions = {
            "script"         : ["(FILENAME)"],
            "set"            : ["(SETTING)"],
            "status"         : ["(VARIABLE)"],
            "module"    : ["list",
                           "load (AVAILMODULES)",
                           "<unload|reload> (LOADEDMODULES)"]
        }

        self.status = MPStatus()

        # master mavlink device
        self.mav_master = None

        # mavlink outputs
        self.mav_outputs = []
        self.sysid_outputs = {}

        # Mapping of all detected sysid's to links
        # Key is link id, value is all detected sysid's/compid's in that link
        # Dict of self.vehicle_link_map[linknumber] = set([(sysid1,compid1), (sysid2,compid2), ...])
        self.vehicle_link_map = {}

        # SITL output
        self.sitl_output = None

        self.mav_param_by_sysid = {}
        self.mav_param_by_sysid[(self.settings.target_system, self.settings.target_component)] = mavparm.MAVParmDict()
        self.modules = []
        self.public_modules = {}
        self.functions = MAVFunctions()
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}
        import platform
        self.system = platform.system()
        self.multi_instance = {}
        self.instance_count = {}
        self.is_sitl = False
        self.start_time_s = time.time()
        self.attitude_time_s = 0
        self.position = None

    @property
    def mav_param(self):
        '''map mav_param onto the current target system parameters'''
        compid = self.settings.target_component
        if compid == 0:
            compid = 1
        sysid = (self.settings.target_system, compid)
        if sysid not in self.mav_param_by_sysid:
            self.mav_param_by_sysid[sysid] = mavparm.MAVParmDict()
        return self.mav_param_by_sysid[sysid]

    def module(self, name):
        '''Find a public module (most modules are private)'''
        if name in self.public_modules:
            return self.public_modules[name]
        return None

    def load_module(self, modname, quiet=False, **kwargs):
        '''load a module'''
        modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
        for (m, pm) in mpstate.modules:
            if m.name == modname and modname not in mpstate.multi_instance:
                if not quiet:
                    print(tr("module_already_loaded") % modname)
                # don't report an error
                return True
        ex = None
        for modpath in modpaths:
            try:
                m = import_package(modpath)
                reload(m)
                module = m.init(mpstate, **kwargs)
                if isinstance(module, mp_module.MPModule):
                    mpstate.modules.append((module, m))
                    if not quiet:
                        if kwargs:
                            print(tr("loaded_module_with_kwargs") % (modname, kwargs))
                        else:
                            print(tr("loaded_module") % (modname,))
                    return True
                else:
                    ex = "%s.init did not return a MPModule instance" % modname
                    break
            except ImportError as msg:
                ex = msg
                if mpstate.settings.moddebug > 1:
                    print(get_exception_stacktrace(ex))
        help_traceback = ""
        if mpstate.settings.moddebug < 3:
            help_traceback = " Use 'set moddebug 3' in the MAVProxy console to enable traceback"
        print(tr("failed_to_load_module") % (ex, help_traceback))
        return False

    def unload_module(self, modname):
        '''unload a module'''
        for (m, pm) in mpstate.modules:
            if m.name == modname:
                if hasattr(m, 'unload'):
                    t = threading.Thread(target=lambda : m.unload(), name="unload %s" % modname)
                    t.start()
                    t.join(timeout=5)
                    if t.is_alive():
                        print(tr("unload_on_module_did_not_complete") % m.name)
                        mpstate.modules.remove((m, pm))
                        return False
                mpstate.modules.remove((m, pm))
                if modname in mpstate.public_modules:
                    del mpstate.public_modules[modname]
                print(tr("unloaded_module") % modname)
                return True
        print(tr("unable_to_find_module") % modname)
        return False

    def master(self, target_sysid=-1):
        '''return the currently chosen mavlink master object'''
        if len(self.mav_master) == 0:
            return None
        if self.settings.link > len(self.mav_master):
            self.settings.link = 1

        if target_sysid != -1:
            # if we're looking for a specific system ID then try to find best
            # link for that
            best_link = None
            best_timestamp = 0
            for m in self.mav_master:
                try:
                    tstamp = m.sysid_state[target_sysid].messages['HEARTBEAT']._timestamp
                except Exception:
                    continue
                if tstamp > best_timestamp:
                    best_link = m
                    best_timestamp = tstamp
            if best_link is not None:
                return best_link

        # try to use one with no link error
        if not self.mav_master[self.settings.link-1].linkerror:
            return self.mav_master[self.settings.link-1]
        for m in self.mav_master:
            if not m.linkerror:
                return m
        return self.mav_master[self.settings.link-1]

    def foreach_mav(self, sysid, compid, closure):
        # Send mavlink message only on all links that contain vehicle (sysid, compid)
        # More efficient than just blasting all links, when sending targetted messages
        # Also useful for sending messages to non-selected vehicles
        for linkNumber, vehicleList in self.vehicle_link_map.items():
            if (sysid, compid) not in vehicleList:
                continue
            closure(self.mav_master[linkNumber].mav)

    def notify_click(self):
        notify_mods = ['map', 'misseditor']
        for modname in notify_mods:
            mod = self.module(modname)
            if mod is not None:
                mod.click_updated()

    def click(self, latlng):
        if latlng is None:
            self.click_location = None
            self.click_time = None
            self.notify_click()
            return

        (lat, lng) = latlng
        if lat is None:
            print(tr("bad_lat"))
            return
        if lng is None:
            print(tr("bad_lng"))
            return
        self.click_location = (lat, lng)
        self.click_time = time.time()
        self.notify_click()


def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)


def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)


def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print(tr("usage_script_filename"))
        return

    run_script(args[0])


def cmd_set(args):
    '''control mavproxy options'''
    mpstate.settings.command(args)


def cmd_status(args):
    '''show status'''
    verbose = False
    if "--verbose" in args:
        verbose = True
        args = list(filter(lambda x : x != "--verbose", args))
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None, verbose=verbose)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern, verbose=verbose)


def cmd_setup(args):
    mpstate.status.setup_mode = True
    mpstate.rl.set_prompt("")


def cmd_reset(args):
    print(tr("resetting_master"))
    mpstate.master().reset()


def cmd_click(args):
    '''synthesise click at lat/lon; no arguments is "unclick"'''
    usage = tr("usage_click")
    if len(args) == 0:
        mpstate.click(None)
        return
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "show":
        print(tr("msg") % mpstate.click_location)
        return
    if len(args) < 2:
        print(usage)
        return
    lat = mavutil.evaluate_expression(args[0], mpstate.master().messages)
    lng = mavutil.evaluate_expression(args[1], mpstate.master().messages)
    mpstate.click((lat, lng))


def cmd_watch(args):
    '''watch a mavlink packet pattern'''
    if len(args) == 0:
        mpstate.status.watch = None
        mpstate.status.watch_verbose = False
        return
    if "--verbose" in args:
        mpstate.status.watch_verbose = True
        args.remove('--verbose')
    else:
        mpstate.status.watch_verbose = False

    mpstate.status.watch = args
    print(tr("watching") % mpstate.status.watch)


def generate_kwargs(args):
    kwargs = {}
    module_components = args.split(":{", 1)
    module_name = module_components[0]
    if (len(module_components) == 2 and module_components[1].endswith("}")):
        # assume json
        try:
            module_args = "{"+module_components[1]
            kwargs = json.loads(module_args)
        except ValueError as e:
            print(tr("invalid_json_argument").format(module_args,
                                                            repr(e)))
    return (module_name, kwargs)


def get_exception_stacktrace(e):
    if sys.version_info[0] >= 3:
        ret = "%s\n" % e
        ret += ''.join(traceback.format_exception(type(e),
                                                  e,
                                                  tb=e.__traceback__))
        return ret
    return traceback.format_exc(e)


def cmd_module(args):
    '''module commands'''
    usage = tr("usage_module")
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        mods = []
        for (m, pm) in mpstate.modules:
            mods.append(m)
        mods = sorted(mods, key=lambda m : m.name)
        for m in mods:
            print("%s: %s" % (m.name, tr(m.description)))
    elif args[0] == "load":
        if len(args) < 2:
            print(tr("usage_module_load_name"))
            return
        (modname, kwargs) = generate_kwargs(args[1])
        try:
            mpstate.load_module(modname, **kwargs)
        except TypeError as ex:
            print(ex)
            print(tr("module_does_not_support_keyword_arguments") % modname)
            return
    elif args[0] == "reload":
        if len(args) < 2:
            print(tr("usage_module_reload_name"))
            return
        (modname, kwargs) = generate_kwargs(args[1])
        pmodule = None
        for (m, pm) in mpstate.modules:
            if m.name == modname:
                pmodule = pm
        if pmodule is None:
            print(tr("module_not_loaded") % modname)
            return
        if mpstate.unload_module(modname):
            try:
                reload(pmodule)
            except ImportError:
                clear_zipimport_cache()
                reload(pmodule)
            try:
                if mpstate.load_module(modname, quiet=True, **kwargs):
                    print(tr("reloaded_module") % modname)
            except TypeError:
                print(tr("module_does_not_support_keyword_arguments") % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print(tr("usage_module_unload_name"))
            return
        modname = os.path.basename(args[1])
        mpstate.unload_module(modname)
    else:
        print(usage)


def cmd_alias(args):
    '''alias commands'''
    usage = tr("usage_alias")
    if len(args) < 1 or args[0] == "list":
        if len(args) >= 2:
            wildcard = args[1].upper()
        else:
            wildcard = '*'
        for a in sorted(mpstate.aliases.keys()):
            if fnmatch.fnmatch(a.upper(), wildcard):
                print("%-15s : %s" % (a, mpstate.aliases[a]))
    elif args[0] == "add":
        if len(args) < 3:
            print(usage)
            return
        a = args[1]
        mpstate.aliases[a] = ' '.join(args[2:])
    elif args[0] == "remove":
        if len(args) != 2:
            print(usage)
            return
        a = args[1]
        if a in mpstate.aliases:
            mpstate.aliases.pop(a)
        else:
            print(tr("no_alias") % a)
    else:
        print(usage)
        return


def clear_zipimport_cache():
    """Clear out cached entries from _zip_directory_cache.
    See http://www.digi.com/wiki/developer/index.php/Error_messages"""
    import sys
    import zipimport

    syspath_backup = list(sys.path)
    zipimport._zip_directory_cache.clear()

    # load back items onto sys.path
    sys.path = syspath_backup
    # add this too: see https://mail.python.org/pipermail/python-list/2005-May/353229.html
    sys.path_importer_cache.clear()

# http://stackoverflow.com/questions/211100/pythons-import-doesnt-work-as-expected
# has info on why this is necessary.


def import_package(name):
    """Given a package name like 'foo.bar.quux', imports the package
    and returns the desired module."""
    try:
        mod = __import__(name)
    except ImportError:
        clear_zipimport_cache()
        mod = __import__(name)

    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod


# Help entries are message keys looked up via tr() at display time so they
# follow the active language (see process_stdin help handling).
command_map = {
    'script'  : (cmd_script,   "cmdhelp_script"), # noqa:E241
    'setup'   : (cmd_setup,    "cmdhelp_setup"), # noqa:E241
    'reset'   : (cmd_reset,    "cmdhelp_reset"), # noqa:E241
    'click'   : (cmd_click,    "cmdhelp_click"), # noqa:E241
    'status'  : (cmd_status,   "cmdhelp_status"), # noqa:E241
    'set'     : (cmd_set,      "cmdhelp_set"), # noqa:E241
    'watch'   : (cmd_watch,    "cmdhelp_watch"), # noqa:E241
    'module'  : (cmd_module,   "cmdhelp_module"), # noqa:E241
    'alias'   : (cmd_alias,    "cmdhelp_alias") # noqa:E241
}


def shlex_quotes(value):
    '''see http://stackoverflow.com/questions/6868382/python-shlex-split-ignore-single-quotes'''
    lex = shlex.shlex(value)
    lex.quotes = '"'
    lex.whitespace_split = True
    lex.commenters = ''
    return list(lex)


def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)

    # allow for modules to override input handling
    if mpstate.functions.input_handler is not None:
        mpstate.functions.input_handler(line)
        return

    line = line.strip()

    if mpstate.status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            mpstate.status.setup_mode = False
            mpstate.status.flightmode = "MAV"
            mpstate.rl.set_prompt("MAV> ")
            return
        if line != '+++':
            line += '\r'
        for c in line:
            time.sleep(0.01)
            if sys.version_info.major >= 3:
                mpstate.master().write(bytes(c, "ascii"))
            else:
                mpstate.master().write(c)
        return

    if not line:
        return

    try:
        args = shlex_quotes(line)
    except Exception as e:
        print(tr("caught_shlex_exception") % str(e))
        return

    # strip surrounding quotes - shlex leaves them in place
    new_args = []
    for arg in args:
        new_arg = arg
        for q in "'", '"':
            if arg.startswith(q) and arg.endswith(q):
                new_arg = arg[1:-1]
                break
        new_args.append(new_arg)
    args = new_args

    cmd = args[0]
    while cmd in mpstate.aliases:
        line = mpstate.aliases[cmd]
        args = shlex.split(line) + args[1:]
        cmd = args[0]

    if cmd == 'help':
        # Use mpstate.command_map (includes module commands) and translate at
        # display time so `set language` updates help text immediately.
        cmap = mpstate.command_map if mpstate is not None else command_map
        for cname in sorted(cmap.keys()):
            (fn, help_key) = cmap[cname]
            print("%-15s : %s" % (cname, tr(help_key)))
        return
    if cmd == 'exit' and mpstate.settings.requireexit:
        mpstate.status.stop_event.set()
        return

    if cmd not in command_map:
        for (m, pm) in mpstate.modules:
            if hasattr(m, 'unknown_command'):
                try:
                    if m.unknown_command(args):
                        return
                except Exception as e:
                    print(tr("error_in_command") % str(e))
        print(tr("unknown_command") % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print(tr("error_in_command_2") % (args[1:], str(e)))
        if mpstate.settings.moddebug > 1:
            traceback.print_exc()


def process_master(m):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        time.sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        time.sleep(0.1)
        return

    mpstate.status.bytecounters['MasterIn'][m.linknum].update(len(s))

    if (mpstate.settings.compdebug & 1) != 0:
        return

    if mpstate.logqueue_raw:
        mpstate.logqueue_raw.put(bytearray(s))

    if mpstate.status.setup_mode:
        if mpstate.system == 'Windows':
            # strip nsh ansi codes
            s = s.replace("\033[K", "")
        if sys.version_info.major >= 3:
            sys.stdout.write(str(s, "ascii", "ignore"))
        else:
            sys.stdout.write(str(s))
        sys.stdout.flush()
        return

    if m.first_byte and mavversion is None:
        m.auto_mavlink_version(s)
    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            sysid = msg.get_srcSystem()
            if sysid in mpstate.sysid_outputs:
                # the message has been handled by a specialised
                # handler for this system
                continue
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)
            if msg.get_type() == "BAD_DATA":
                if opts.show_errors:
                    mpstate.console.writeln(tr("mav_error") % msg)
                mpstate.status.mav_error += 1


def process_mavlink(slave):
    '''process packets from MAVLink slaves, forwarding to the master'''
    try:
        buf = slave.recv()
    except socket.error:
        return
    try:
        if slave.first_byte and mavversion is None:
            slave.auto_mavlink_version(buf)
        msgs = slave.mav.parse_buffer(buf)
    except mavutil.mavlink.MAVError as e:
        mpstate.console.error(tr("bad_mavlink_slave_message_from") % (slave.address, e.message))
        return
    if msgs is None:
        return
    allow_fwd = mpstate.settings.mavfwd
    if not allow_fwd and mpstate.settings.mavfwd_disarmed and not mpstate.master(-1).motors_armed():
        allow_fwd = True
    if mpstate.status.setup_mode:
        allow_fwd = False
    if allow_fwd:
        for m in msgs:
            target_sysid = getattr(m, 'target_system', -1)
            if mpstate.settings.mavfwd_link > 0 and mpstate.settings.mavfwd_link <= len(mpstate.mav_master):
                output = mpstate.mav_master[mpstate.settings.mavfwd_link-1]
            else:
                # find best link by sysid
                output = mpstate.master(target_sysid)
            if (mpstate.settings.mavfwd_signing and
                    output.mav.signing.sign_outgoing and
                    (m._header.incompat_flags & mavutil.mavlink.MAVLINK_IFLAG_SIGNED) == 0):
                # repack the message if this is a signed link and not already signed
                m.pack(output.mav)

            output.write(m.get_msgbuf())
            if mpstate.logqueue:
                usec = int(time.time() * 1.0e6)
                mpstate.logqueue.put(bytearray(struct.pack('>Q', usec) + m.get_msgbuf()))
            if mpstate.status.watch:
                for msg_type in mpstate.status.watch:
                    if fnmatch.fnmatch(m.get_type().upper(), msg_type.upper()):
                        mpstate.console.writeln('> ' + str(m))
                        break
    mpstate.status.counters['Slave'] += 1


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)


def log_writer():
    '''log writing thread'''
    while not mpstate.status.stop_event.is_set():
        if not mpstate.logqueue_raw.empty():
            bytes = mpstate.logqueue_raw.get(block=False)
            mpstate.logfile_raw.write(bytearray(bytes))
        time.sleep(0.001)

        # TODO consider wait() the stop event instead
        timeout = time.time() + 10
        while not mpstate.logqueue_raw.empty() and time.time() < timeout:
            mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue.empty() and time.time() < timeout:
            mpstate.logfile.write(mpstate.logqueue.get())
        if mpstate.settings.flushlogs or time.time() >= timeout:
            mpstate.logfile.flush()
            mpstate.logfile_raw.flush()


# If state_basedir is NOT set then paths for logs and aircraft
# directories are relative to mavproxy's cwd
def log_paths():
    '''Returns tuple (logdir, telemetry_log_filepath, raw_telemetry_log_filepath)'''
    if opts.aircraft is not None:
        dirname = ""
        if opts.mission is not None:
            print(opts.mission)
            dirname += "%s/logs/%s/Mission%s" % (opts.aircraft, time.strftime("%Y-%m-%d"), opts.mission)
        else:
            dirname += "%s/logs/%s" % (opts.aircraft, time.strftime("%Y-%m-%d"))
        # dirname is currently relative.  Possibly add state_basedir:
        if mpstate.settings.state_basedir is not None:
            dirname = os.path.join(mpstate.settings.state_basedir, dirname)
        mkdir_p(dirname)
        highest = None
        for i in range(1, 10000):
            fdir = os.path.join(dirname, 'flight%u' % i)
            if not os.path.exists(fdir):
                break
            highest = fdir
        if mpstate.continue_mode and highest is not None:
            fdir = highest
        elif os.path.exists(fdir):
            print(tr("flight_logs_full"))
            sys.exit(1)
        logname = 'flight.tlog'
        logdir = fdir
    else:
        logname = os.path.basename(opts.logfile)
        dir_path = os.path.dirname(opts.logfile)
        if not os.path.isabs(dir_path) and mpstate.settings.state_basedir is not None:
            dir_path = os.path.join(mpstate.settings.state_basedir, dir_path)

        logdir = dir_path

    mkdir_p(logdir)
    return (logdir,
            os.path.join(logdir, logname),
            os.path.join(logdir, logname + '.raw'))


def open_telemetry_logs(logpath_telem, logpath_telem_raw):
    '''open log files'''
    if opts.append_log or opts.continue_mode:
        mode = 'ab'
    else:
        mode = 'wb'

    try:
        mpstate.logfile = open(logpath_telem, mode=mode)
        mpstate.logfile_raw = open(logpath_telem_raw, mode=mode)
        print(tr("log_directory") % mpstate.status.logdir)
        print(tr("telemetry_log") % logpath_telem)

        # make sure there's enough free disk space for the logfile (>200Mb)
        # statvfs doesn't work in Windows
        if platform.system() != 'Windows':
            stat = os.statvfs(logpath_telem)
            if stat.f_bfree*stat.f_bsize < 209715200:
                print(tr("error_not_enough_free_disk_space"))
                mpstate.status.stop_event.set()
                return

        # use a separate thread for writing to the logfile to prevent
        # delays during disk writes (important as delays can be long if camera
        # app is running)
        t = threading.Thread(target=log_writer, name='log_writer')
        t.start()
    except Exception as e:
        print(tr("error_opening_log_file_for_writing") % e)
        mpstate.status.stop_event.set()
        return


def set_stream_rates():
    '''set mavlink stream rates'''
    if (not msg_period.trigger() and
        mpstate.status.last_streamrate1 == mpstate.settings.streamrate and
            mpstate.status.last_streamrate2 == mpstate.settings.streamrate2):
        return
    mpstate.status.last_streamrate1 = mpstate.settings.streamrate
    mpstate.status.last_streamrate2 = mpstate.settings.streamrate2
    for master in mpstate.mav_master:
        if master.linknum == 0:
            rate = mpstate.settings.streamrate
        else:
            rate = mpstate.settings.streamrate2
        if rate != -1 and mpstate.settings.streamrate != -1:
            # Send to all detected vehicles in this link
            for (sysid, compid) in mpstate.vehicle_link_map[master.linknum]:
                master.mav.request_data_stream_send(sysid, compid,
                                                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                    rate, 1)


def check_link_status():
    '''check status of master links'''
    tnow = time.time()
    if mpstate.status.last_message != 0 and tnow > mpstate.status.last_message + mpstate.settings.timeout:
        say(tr("no_link"))
        mpstate.status.heartbeat_error = True
    for master in mpstate.mav_master:
        if not master.linkerror and (tnow > master.last_message + mpstate.settings.timeout or master.portdead):
            say(tr("link_down") % (mp_module.MPModule.link_label(master)))
            master.linkerror = True


def send_heartbeat(master):
    if master.mavlink10():
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
    else:
        MAV_GROUND = 5
        MAV_AUTOPILOT_NONE = 4
        master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)


def periodic_tasks():
    '''run periodic checks'''
    if mpstate.status.setup_mode:
        return

    if (mpstate.settings.compdebug & 2) != 0:
        return

    if mpstate.settings.heartbeat != 0:
        heartbeat_period.frequency = mpstate.settings.heartbeat

    if heartbeat_period.trigger() and mpstate.settings.heartbeat != 0:
        mpstate.status.counters['MasterOut'] += 1
        for master in mpstate.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    mpstate.status.update_bytecounters()

    # call optional module idle tasks. These are called at several hundred Hz
    for (m, pm) in mpstate.modules:
        if hasattr(m, 'idle_task'):
            try:
                m.idle_task()
            except Exception as msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    print(get_exception_stacktrace(msg))

        # also see if the module should be unloaded:
        if m.needs_unloading:
            mpstate.unload_module(m.name)


def main_loop():
    '''main processing loop'''

    global screensaver_cookie

    if not mpstate.status.setup_mode and not opts.nowait:
        for master in mpstate.mav_master:
            if master.linknum != 0:
                break
            print(tr("waiting_for_heartbeat_from") % master.address)
            send_heartbeat(master)
            master.wait_heartbeat(timeout=0.1)
        set_stream_rates()

    while True:
        if mpstate is None or mpstate.status.stop_event.is_set():
            return

        # enable or disable screensaver:
        if (mpstate.settings.inhibit_screensaver_when_armed and
                screensaver_interface is not None):
            if mpstate.status.armed and screensaver_cookie is None:
                # now we can inhibit the screensaver
                screensaver_cookie = screensaver_interface.Inhibit("MAVProxy",
                                                                   "Vehicle is armed")
            elif not mpstate.status.armed and screensaver_cookie is not None:
                # we can also restore it
                screensaver_interface.UnInhibit(screensaver_cookie)
                screensaver_cookie = None

        while not mpstate.input_queue.empty():
            line = mpstate.input_queue.get()
            mpstate.input_count += 1
            cmds = line.split(';')
            if len(cmds) == 1 and cmds[0] == "":
                mpstate.empty_input_count += 1
            for c in cmds:
                process_stdin(c)

        for master in mpstate.mav_master:
            if master.fd is None:
                try:
                    if master.port.inWaiting() > 0:
                        process_master(master)
                except serial.SerialException:
                    pass

        periodic_tasks()

        rin = []
        for master in mpstate.mav_master:
            if master.fd is not None and not master.portdead:
                rin.append(master.fd)
        for m in mpstate.mav_outputs:
            if m.fd is not None:
                rin.append(m.fd)
        for sysid in mpstate.sysid_outputs:
            m = mpstate.sysid_outputs[sysid]
            if m.fd is not None:
                rin.append(m.fd)
        if rin == []:
            time.sleep(0.0001)
            continue

        for fd in mpstate.select_extra:
            if fd is not None:
                rin.append(fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], mpstate.settings.select_timeout)
        except select.error:
            continue

        if mpstate is None:
            return

        for fd in rin:
            if mpstate is None:
                return
            for master in mpstate.mav_master:
                if fd == master.fd:
                    process_master(master)
                    if mpstate is None:
                        return
                    continue
            for m in mpstate.mav_outputs:
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                        return
                    continue

            for sysid in mpstate.sysid_outputs:
                m = mpstate.sysid_outputs[sysid]
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                        return
                    continue

            # this allow modules to register their own file descriptors
            # for the main select loop
            if fd in mpstate.select_extra:
                try:
                    # call the registered read function
                    (fn, args) = mpstate.select_extra[fd]
                    fn(args)
                except Exception as msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)


def input_loop():
    '''wait for user input'''
    while not mpstate.status.stop_event.is_set():
        try:
            line = mpstate.rl.input()
            mpstate.input_queue.put(line)
        except (EOFError, IOError):
            mpstate.status.stop_event.set()


def run_script(scriptfile):
    '''run a script file'''
    try:
        f = open(scriptfile, mode='r')
    except Exception:
        return
    mpstate.console.writeln(tr("running_script") % scriptfile)
    sub = mp_substitute.MAVSubstitute()
    for line in f:
        line = line.strip()
        if line == "" or line.startswith('#'):
            continue
        try:
            line = sub.substitute(line, os.environ)
        except mp_substitute.MAVSubstituteError as ex:
            print(tr("bad_variable") % str(ex))
            if mpstate.settings.script_fatal:
                sys.exit(1)
            continue
        if line.startswith('@'):
            line = line[1:]
        else:
            mpstate.console.writeln(tr("msg_2") % line)
        process_stdin(line)
    f.close()


def set_mav_version(mav10, mav20, autoProtocol, mavversionArg):
    '''Set the Mavlink version based on commandline options'''
#    if(mav10 == True or mav20 == True or autoProtocol == True):
#        print("Warning: Using deprecated --mav10, --mav20 or --auto-protocol options. Use --mavversion instead")

    # sanity check the options
    if (mav10 or mav20) and autoProtocol:
        print(tr("error_can_t_have_mav10_mav20"))
        sys.exit(1)
    if mav10 and mav20:
        print(tr("error_can_t_have_mav10_and"))
        sys.exit(1)
    if mavversionArg is not None and (mav10 or mav20 or autoProtocol):
        print(tr("error_can_t_use_mavversion_with"))
        sys.exit(1)

    # and set the specific mavlink version (False = autodetect)
    global mavversion
    if mavversionArg == "1.0" or mav10:
        os.environ['MAVLINK09'] = '1'
        mavversion = "1"
    else:
        os.environ['MAVLINK20'] = '1'
        mavversion = "2"


def run_startup_scripts():
    start_scripts = []
    if not opts.setup:
        if 'HOME' in os.environ:
            start_scripts.append(os.path.join(os.environ['HOME'], ".mavinit.scr"))
        start_script = mp_util.dot_mavproxy("mavinit.scr")
        start_scripts.append(start_script)
    if (mpstate.settings.state_basedir is not None and
            opts.aircraft is not None):
        start_script = os.path.join(mpstate.aircraft_dir, "mavinit.scr")
        start_scripts.append(start_script)
    for start_script in start_scripts:
        if os.path.exists(start_script):
            print(tr("running_script_2") % (start_script))
            run_script(start_script)

    if opts.aircraft is not None:
        start_script = os.path.join(opts.aircraft, "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
        else:
            print(tr("no_script") % start_script)


if __name__ == '__main__':
    # Language must be active before help=tr(...) is evaluated and before --help prints.
    _early_lang = early_language_from_argv()
    if not i18n_init(_early_lang):
        # Only warn when user explicitly asked for a language; still continue so --help works.
        if any(a == '--language' or a.startswith('--language=') for a in sys.argv[1:]):
            print(tr("language_not_found_using", _early_lang, get_language()))
            print(tr("available_languages", ", ".join(available_languages())))

    parser = I18nOptionParser(tr("opt_usage_mavproxy_py_options"))

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE[,BAUD]", help=tr("opt_mavlink_master_port_and_optional_baud_rate"),
                      default=[])
    parser.add_option("", "--force-connected", dest="force_connected", help=tr("opt_use_master_even_if_initial_connection_fails"),
                      action='store_true', default=False)
    parser.add_option("--out", dest="output", action='append',
                      metavar="DEVICE[,BAUD]", help=tr("opt_mavlink_output_port_and_optional_baud_rate"),
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help=tr("opt_default_serial_baud_rate"), default=57600)
    parser.add_option("--sitl", dest="sitl", default=None, help=tr("opt_sitl_output_port"))
    parser.add_option("--streamrate", dest="streamrate", default=4, type='int',
                      help=tr("opt_mavlink_stream_rate"))
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help=tr("opt_mavlink_source_system_for_this_gcs"))
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int',
                      default=230, help=tr("opt_mavlink_source_component_for_this_gcs"))
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=0, help=tr("opt_mavlink_target_master_system"))
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help=tr("opt_mavlink_target_master_component"))
    parser.add_option("--logfile", dest="logfile", help=tr("opt_mavlink_master_logfile"),
                      default='mav.tlog')
    parser.add_option("-a", "--append-log", dest="append_log", help=tr("opt_append_to_log_files"),
                      action='store_true', default=False)
    parser.add_option("--quadcopter", dest="quadcopter", help=tr("opt_use_quadcopter_controls"),
                      action='store_true', default=False)
    parser.add_option("--setup", dest="setup", help=tr("opt_start_in_setup_mode"),
                      action='store_true', default=False)
    parser.add_option("--nodtr", dest="nodtr", help=tr("opt_disable_dtr_drop_on_close"),
                      action='store_true', default=False)
    parser.add_option("--show-errors", dest="show_errors", help=tr("opt_show_mavlink_error_packets"),
                      action='store_true', default=False)
    parser.add_option("--speech", dest="speech", help=tr("opt_use_text_to_speech"),
                      action='store_true', default=False)
    parser.add_option("--aircraft", dest="aircraft", help=tr("opt_aircraft_name"), default=None)
    parser.add_option("--cmd", dest="cmd", help=tr("opt_initial_commands"), default=None, action='append')
    parser.add_option("--console", action='store_true', help=tr("opt_use_gui_console"))
    parser.add_option("--heartbeat-rate", dest="heartbeat", default=1, type='float',
                      help=tr("opt_mavlink_heartbeat_rate"))
    if platform.system() == 'Windows':
        parser.add_option("--no-console", action='store_true', help=tr("opt_don_t_use_gui_console"))
    parser.add_option("--map", action='store_true', help=tr("opt_load_map_module"))
    parser.add_option(
        '--load-module',
        action='append',
        default=[],
        help=tr("opt_load_the_specified_module_can_be_used_multiple"))
    parser.add_option("--mav10", action='store_true', default=False, help=tr("opt_use_mavlink_protocol_1_0"))
    parser.add_option("--mav20", action='store_true', default=False, help=tr("opt_use_mavlink_protocol_2_0"))
    parser.add_option("--auto-protocol", action='store_true', default=False, help=tr("opt_auto_detect_mavlink_protocol_version"))
    parser.add_option("--mavversion", type='choice', choices=['1.0', '2.0'] , help=tr("opt_force_mavlink_version_1_0_2_0_otherwise"))  # noqa:E501
    parser.add_option("--nowait", action='store_true', default=False, help=tr("opt_don_t_wait_for_heartbeat_on_startup"))
    parser.add_option("-c", "--continue", dest='continue_mode', action='store_true', default=False, help=tr("opt_continue_logs"))
    parser.add_option("--dialect", default="all", help=tr("opt_mavlink_dialect"))
    parser.add_option("--rtscts", action='store_true', help=tr("opt_enable_hardware_rts_cts_flow_control"))
    parser.add_option("--moddebug", type=int, help=tr("opt_module_debug_level"), default=0)
    parser.add_option("--mission", dest="mission", help=tr("opt_mission_name"), default=None)
    parser.add_option("--daemon", action='store_true', help=tr("opt_run_in_daemon_mode_do_not_start_interactive"))
    parser.add_option("--non-interactive", action='store_true', help=tr("opt_do_not_start_interactive_shell"))
    parser.add_option("--profile", action='store_true', help=tr("opt_run_the_yappi_python_profiler"))
    parser.add_option("--state-basedir", default=None, help=tr("set_base_directory_for_logs_and_aircraft_directories"))
    parser.add_option("--no-state", action='store_true', default=False, help=tr("opt_don_t_save_logs_and_other_state_to"))  # noqa:E501
    parser.add_option("--version", action='store_true', help=tr("opt_version_information"))
    parser.add_option("--default-modules", default="log,signing,wp,rally,fence,ftp,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,adsb,layout", help=tr("opt_default_module_list"))  # noqa:E501
    parser.add_option("--udp-timeout", dest="udp_timeout", default=0.0, type='float', help=tr("opt_timeout_for_udp_clients_in_seconds"))  # noqa:E501
    parser.add_option("--retries", type=int, help=tr("opt_number_of_times_to_retry_connection"), default=3)
    parser.add_option("--language", dest="language", default="en",
                      help=tr("opt_ui_language_for_user_facing_messages"))

    (opts, args) = parser.parse_args()

    # Keep runtime language in sync (also covers aliases resolved by i18n_init)
    if opts.language and opts.language != get_language():
        if not set_language(opts.language):
            print(tr("language_not_found_using", opts.language, get_language()))
            print(tr("available_languages", ", ".join(available_languages())))

    if len(args) != 0:
        print(tr("error_mavproxy_takes_no_position_arguments") % str(args))
        sys.exit(1)

    # warn people about ModemManager which interferes badly with APM and Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print(tr("warning_you_should_uninstall_modemmanager_as"))

    # set the Mavlink version, if required
    set_mav_version(opts.mav10, opts.mav20, opts.auto_protocol, opts.mavversion)

    from pymavlink import mavparm
    mavutil.set_dialect(opts.dialect)

    # version information
    if opts.version:
        import importlib.metadata
        version = importlib.metadata.version("mavproxy")

        print(tr("mavproxy_is_a_modular_ground_station"))
        print("MAVProxy Version: " + version)
        sys.exit(0)

    # global mavproxy state
    mpstate = MPState()
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode
    # apply CLI language to the runtime setting and react to later changes
    mpstate.settings.language = get_language()

    def _on_setting_change(setting):
        if setting.name == 'language':
            if set_language(setting.value):
                # Re-key command descriptions that were stored as English text
                # before reverse-mapping existed, so help re-translates cleanly.
                from MAVProxy.modules.lib.mp_i18n import resolve_key
                for cname, (fn, desc) in list(mpstate.command_map.items()):
                    mpstate.command_map[cname] = (fn, resolve_key(desc))
                for (m, pm) in mpstate.modules:
                    if getattr(m, 'description', None):
                        m.description = resolve_key(m.description)
                print(tr("language_set_to", setting.value))
            else:
                print(tr("language_not_found_using", setting.value, get_language()))
                # snap setting back to whatever is actually active
                mpstate.settings.language = get_language()

    mpstate.settings.set_callback(_on_setting_change)

    # queues for logging

    if not opts.no_state:
        mpstate.logqueue = multiproc.Queue()
        mpstate.logqueue_raw = multiproc.Queue()
    else:
        mpstate.logqueue = None
        mpstate.logqueue_raw = None

    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        mpstate.load_module('speech')

    serial_list = mavutil.auto_detect_serial(preferred_list=preferred_ports)
    serial_list.sort(key=lambda x: x.device)

    # remove OTG2 ports for dual CDC
    if len(serial_list) == 2 and serial_list[0].device.startswith("/dev/serial/by-id"):
        if serial_list[0].device[:-1] == serial_list[1].device[0:-1]:
            serial_list.pop(1)

    if not opts.master:
        print(tr("auto_detected_serial_ports_are"))
        for port in serial_list:
            print(tr("msg_3") % port)

    # container for status information
    mpstate.settings.target_system = opts.TARGET_SYSTEM
    mpstate.settings.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []

    mpstate.rl = rline.rline("MAV> ", mpstate)

    def quit_handler(signum=None, frame=None):
        # print('Signal handler called with signal', signum)
        if mpstate.status.stop_event.is_set():
            print(tr("clean_shutdown_impossible_forcing_an_exit"))
            sys.exit(0)
        else:
            mpstate.status.stop_event.set()

    # Listen for kill signals to cleanly shutdown modules
    fatalsignals = [signal.SIGTERM]
    try:
        fatalsignals.append(signal.SIGQUIT)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    except Exception:
        pass
    if opts.daemon or opts.non_interactive: # SIGINT breaks readline parsing - if we are interactive, just let things die
        fatalsignals.append(signal.SIGINT)

    for sig in fatalsignals:
        signal.signal(sig, quit_handler)

    mpstate.load_module('link', quiet=True)

    # load signing early to allow for use of ~/.mavproxy/signing.keys
    standard_modules = opts.default_modules.split(',')
    if 'signing' in standard_modules:
        mpstate.load_module('signing', quiet=True)

    mpstate.settings.source_system = opts.SOURCE_SYSTEM
    mpstate.settings.source_component = opts.SOURCE_COMPONENT

    # open master link
    for mdev in opts.master:
        if mdev.find('?') != -1 or mdev.find('*') != -1:
            for m in glob.glob(mdev):
                if not mpstate.module('link').link_add(m, force_connected=opts.force_connected, retries=opts.retries):
                    sys.exit(1)
        elif not mpstate.module('link').link_add(mdev, force_connected=opts.force_connected, retries=opts.retries):
            sys.exit(1)

    if not opts.master and len(serial_list) == 1:
        print(tr("connecting_to") % serial_list[0])
        mpstate.module('link').link_add(serial_list[0].device)
    elif not opts.master and len(serial_list) > 1:
        print(tr("warning_multiple_possible_serial_ports_use"))  # noqa:E501
        # if no display, assume running CLI mode and exit
        if platform.system() != 'Windows' and "DISPLAY" not in os.environ:
            sys.exit(1)
    elif not opts.master:
        wifi_device = '0.0.0.0:14550'
        mpstate.module('link').link_add(wifi_device)

    # open any mavlink output ports
    for port in opts.output:
        # cope with older pymavlink
        if opts.udp_timeout > 0:
            mpstate.mav_outputs.append(mavutil.mavlink_connection(
                port,
                baud=int(opts.baudrate),
                input=False,
                udp_timeout=opts.udp_timeout,
                autoreconnect=True
            ))
        else:
            mpstate.mav_outputs.append(mavutil.mavlink_connection(port, baud=int(opts.baudrate),
                                                                  input=False, autoreconnect=True))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    mpstate.settings.heartbeat = opts.heartbeat

    if opts.state_basedir is not None:
        mpstate.settings.state_basedir = opts.state_basedir

    msg_period = mavutil.periodic_event(1.0/15)
    heartbeat_period = mavutil.periodic_event(1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.input_queue = multiproc.Queue()
    mpstate.input_count = 0
    mpstate.empty_input_count = 0
    if opts.setup:
        mpstate.rl.set_prompt("")

    # call this early so that logdir is setup based on --aircraft
    if not opts.no_state:
        (mpstate.status.logdir, logpath_telem, logpath_telem_raw) = log_paths()

    for module in opts.load_module:
        modlist = module.split(',')
        for mod in modlist:
            process_stdin('module load %s' % (mod))

    if not opts.setup:
        # some core functionality is in modules
        for m in standard_modules:
            if m:
                mpstate.load_module(m, quiet=True)

    if platform.system() != 'Windows':
        if opts.console:
            process_stdin('module load console')
    else:
        # default to having console on windows
        if opts.console or not opts.no_console:
            process_stdin('module load console')

    if opts.map:
        process_stdin('module load map')

    if (mpstate.settings.state_basedir is not None and
            opts.aircraft is not None):
        mpstate.aircraft_dir = os.path.join(mpstate.settings.state_basedir, opts.aircraft)
    elif opts.aircraft is not None:
        mpstate.aircraft_dir = opts.aircraft

    run_startup_scripts()

    if opts.cmd is not None:
        for cstr in opts.cmd:
            cmds = cstr.split(';')
            for c in cmds:
                process_stdin(c)

    if opts.profile:
        import yappi    # We do the import here so that we won't barf if run normally and yappi not available
        yappi.start()

    # log all packets from the master, for later replay
    if not opts.no_state:
        open_telemetry_logs(logpath_telem, logpath_telem_raw)
    else:
        print(tr("note_not_saving_telemetry_logs"))

    # run main loop as a thread
    mpstate.status.thread = threading.Thread(target=main_loop, name='main_loop')
    mpstate.status.thread.daemon = True
    mpstate.status.thread.start()

    # use main program for input. This ensures the terminal cleans
    # up on exit
    while not mpstate.status.stop_event.is_set():
        try:
            if opts.daemon or opts.non_interactive:
                time.sleep(0.1)
            else:
                input_loop()
        except KeyboardInterrupt:
            if mpstate.settings.requireexit:
                print(tr("interrupt_caught_use_exit_to_quit"))

                # Just lost the map and console, get them back:
                for (m, pm) in mpstate.modules:
                    if m.name in ["map", "console"]:
                        if hasattr(m, 'unload'):
                            try:
                                m.unload()
                            except Exception:
                                pass
                        reload(m)
                        m.init(mpstate)

            else:
                mpstate.status.stop_event.set()
                sys.exit(1)

    if opts.profile:
        yappi.get_func_stats().print_all()
        yappi.get_thread_stats().print_all()

    # this loop executes after leaving the above loop and is for cleanup on exit
    for (m, pm) in mpstate.modules:
        if hasattr(m, 'unload'):
            print(tr("unloading_module") % m.name)
            m.unload()

    sys.exit(1)
