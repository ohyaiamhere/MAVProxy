#!/usr/bin/env python3
'''
miscellaneous commands

AP_FLAKE8_CLEAN
'''

import math
import os
import sys
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

from signal import signal
from subprocess import PIPE, Popen
from MAVProxy.modules.lib.mp_i18n import tr


class RepeatCommand(object):
    '''repeated command object'''
    def __init__(self, period, cmd):
        self.period = period
        self.cmd = cmd
        self.event = mavutil.periodic_event(1.0/period)
        self.enabled = True

    def __str__(self):
        return "[%s] Every %.1f seconds: %s" % (("x" if self.enabled else " "), self.period, self.cmd)


def run_command(args, cwd=None, shell=False, timeout=None, env=None):
    '''
    Run a shell command with a timeout.
    See http://stackoverflow.com/questions/1191374/subprocess-with-timeout
    '''
    try:
        # py2
        from StringIO import StringIO
    except ImportError:
        # py3
        from io import StringIO
    import fcntl
    p = Popen(args, shell=shell, cwd=cwd, stdout=PIPE, stderr=PIPE, env=env)
    tstart = time.time()
    buf = StringIO()

    # try to make it non-blocking
    try:
        fcntl.fcntl(p.stdout, fcntl.F_SETFL, fcntl.fcntl(p.stdout, fcntl.F_GETFL) | os.O_NONBLOCK)
    except Exception:
        pass

    while True:
        time.sleep(0.1)
        retcode = p.poll()
        try:
            s = p.stdout.read()
            if sys.version_info.major >= 3:
                s = s.decode('utf-8')
            buf.write(s)
        except Exception:
            pass
        if retcode is not None:
            break
        if timeout is not None and time.time() > tstart + timeout:
            print(tr("timeout_in_process_u") % p.pid)
            try:
                os.kill(p.pid, signal.SIGKILL)
            except OSError:
                pass
            p.wait()
    return buf.getvalue()


class MiscModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MiscModule, self).__init__(mpstate, "misc", tr("mod_misc_commands"), public=True)
        self.add_command('alt', self.cmd_alt, tr("cmd_show_altitude_information"))
        self.add_command('up', self.cmd_up, tr("cmd_adjust_pitch_trim_by_up_to_5_degrees"))
        self.add_command('reboot', self.cmd_reboot, tr("cmd_reboot_autopilot"))
        self.add_command('time', self.cmd_time, tr("cmd_show_autopilot_time"))
        self.add_command('shell', self.cmd_shell, tr("cmd_run_shell_command"))
        self.add_command('changealt', self.cmd_changealt, tr("cmd_change_target_altitude"))
        self.add_command('changealt_abs', self.cmd_changealt_abs, tr("cmd_change_target_absolute_altitude"))
        self.add_command('land', self.cmd_land, tr("cmd_auto_land"))
        self.add_command('repeat', self.cmd_repeat, tr("cmd_repeat_a_command_at_regular_intervals"),
                         ["<add|remove|clear>"])
        self.add_command('version', self.cmd_version, tr("cmd_fetch_autopilot_version"))
        self.add_command('capabilities', self.cmd_capabilities, tr("cmd_fetch_autopilot_capabilities"))
        self.add_command('rcbind', self.cmd_rcbind, tr("cmd_bind_rc_receiver"))
        self.add_command('led', self.cmd_led, tr("cmd_control_board_led"))
        self.add_command('oreoled', self.cmd_oreoled, tr("cmd_control_oreoleds"))
        self.add_command('playtune', self.cmd_playtune, tr("cmd_play_tune_remotely"))
        self.add_command('devid', self.cmd_devid, tr("cmd_show_device_names_from_parameter_ids"))
        self.add_command('gethome', self.cmd_gethome, tr("cmd_get_home_position"))
        self.add_command('flashbootloader', self.cmd_flashbootloader, tr("cmd_flash_bootloader_dangerous"))
        self.add_command('wipe_parameters', self.cmd_wipe_parameters, tr("cmd_wipe_autopilot_parameters"))
        self.add_command('lockup_autopilot', self.cmd_lockup_autopilot, tr("cmd_lockup_autopilot"))
        self.add_command('corrupt_params', self.cmd_corrupt_param, tr("cmd_corrupt_param_storage"))
        self.add_command('hardfault_autopilot', self.cmd_hardfault_autopilot, tr("cmd_hardfault_autopilot"))
        self.add_command('panic_autopilot', self.cmd_panic_autopilot, tr("cmd_panic_autopilot"))
        self.add_command('longloop_autopilot', self.cmd_longloop_autopilot, tr("cmd_cause_long_loop_in_autopilot"))
        self.add_command('configerror_autopilot', self.cmd_config_error_autopilot, tr("cmd_ask_autopilot_to_jump_to_its_config_error"))  # noqa:E501
        self.add_command('internalerror_autopilot', self.cmd_internalerror_autopilot, tr("cmd_cause_internal_error_in_autopilot"))
        self.add_command('dfu_boot', self.cmd_dfu_boot, tr("cmd_boot_into_dfu_mode"))
        self.add_command('deadlock', self.cmd_deadlock, tr("cmd_trigger_deadlock"))
        self.add_command('nullptr_read', self.cmd_nullptr_read, tr("cmd_read_from_a_very_low_address"))
        self.add_command('nullptr_write', self.cmd_nullptr_write, tr("cmd_write_to_a_very_low_address"))
        self.add_command('batreset', self.cmd_battery_reset, tr("cmd_reset_battery_remaining"))
        self.add_command('setorigin', self.cmd_setorigin, tr("cmd_set_global_origin"))
        self.add_command('magsetfield', self.cmd_magset_field, tr("cmd_set_expected_mag_field_by_field"))
        self.add_command('magresetofs', self.cmd_magreset_ofs, tr("cmd_reset_offsets_for_all_compasses"))
        self.add_command('namedvaluefloat', self.cmd_namedvaluefloat, tr("cmd_send_a_named_value_float"))
        self.add_command('scripting', self.cmd_scripting, tr("cmd_control_onboard_scripting"), ["<stop|restart>"])
        self.add_command('formatsdcard', self.cmd_formatsdcard, tr("cmd_format_sd_card"))
        self.add_command('canforward', self.cmd_canforward, tr("cmd_enable_can_forwarding"))

        self.add_command('gear', self.cmd_landing_gear, tr("cmd_landing_gear_control"))

        self.repeats = []

        # support for changing altitude via command rather than mission item:
        self.accepts_DO_CMD_CHANGE_ALTITUDE = {}  # keyed by (sysid, compid)

    def altitude_difference(self, pressure1, pressure2, ground_temp):
        '''calculate barometric altitude'''
        scaling = pressure2 / pressure1
        temp = ground_temp + 273.15
        return 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))

    def qnh_estimate(self):
        '''estimate QNH pressure from GPS altitude and scaled pressure'''
        alt_gps = self.master.field('GPS_RAW_INT', 'alt', 0) * 0.001
        pressure2 = self.master.field('SCALED_PRESSURE', 'press_abs', 0)
        ground_temp = self.get_mav_param('GND_TEMP', 21)
        temp = ground_temp + 273.15
        pressure1 = pressure2 / math.exp(math.log(1.0 - (alt_gps / (153.8462 * temp))) / 0.190259)
        return pressure1

    def cmd_alt(self, args):
        '''show altitude'''
        print(tr("altitude") % self.status.altitude)
        qnh_pressure = self.get_mav_param('AFS_QNH_PRESSURE', None)
        if qnh_pressure is not None and qnh_pressure > 0:
            ground_temp = self.get_mav_param('GND_TEMP', 21)
            pressure = self.master.field('SCALED_PRESSURE', 'press_abs', 0)
            qnh_alt = self.altitude_difference(qnh_pressure, pressure, ground_temp)
            print(tr("qnh_alt_u_meters_u_feet") % (qnh_alt, qnh_alt*3.2808, qnh_pressure))
        print(tr("qnh_estimate_millibars") % self.qnh_estimate())

    def cmd_shell(self, args):
        '''shell command'''
        print(run_command(args, shell=False, timeout=3))

    def cmd_up(self, args):
        '''adjust TRIM_PITCH_CD up by 5 degrees'''
        if len(args) == 0:
            adjust = 5.0
        else:
            adjust = float(args[0])
        old_trim = self.get_mav_param('TRIM_PITCH_CD', None)
        if old_trim is None:
            print(tr("existing_trim_value_unknown"))
            return
        new_trim = int(old_trim + (adjust*100))
        if math.fabs(new_trim - old_trim) > 1000:
            print(tr("adjustment_by_too_large_from_to") % (adjust*100, old_trim, new_trim))
            return
        print(tr("adjusting_trim_pitch_cd_from_to") % (old_trim, new_trim))
        self.param_set('TRIM_PITCH_CD', new_trim)

    def cmd_reboot(self, args):
        '''reboot autopilot'''

        hold_in_bootloader = "bootloader" in args
        force = "force" in args

        # different path for force/not force to avoid dependency on
        # pymavlink's force-reboot support:
        if force:
            if hold_in_bootloader:
                param1 = 3
            else:
                param1 = 1
            param6 = 20190226
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                param1,
                0,
                0,
                0,
                0,
                param6,
                0
            )
            return

        if len(args) > 0 and args[0] == 'bootloader':
            self.master.reboot_autopilot(True)
        else:
            self.master.reboot_autopilot()

    def cmd_wipe_parameters(self, args):
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
            0,
            2,
            0,
            0,
            0,
            0,
            0,
            0)

    def cmd_dosomethingreallynastyto_autopilot(self, args, description, code):
        '''helper function for the following commands which do unpleasant
        things to the autopilot'''
        if len(args) > 0 and args[0] == 'IREALLYMEANIT':
            print(tr("sending_command") % description)
            self.master.mav.command_long_send(
                self.settings.target_system,
                self.settings.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                42,
                24,
                71,
                code,
                0,
                0,
                0)
        else:
            print(tr("invalid_command") % description)

    def cmd_lockup_autopilot(self, args):
        '''lockup autopilot for watchdog testing'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'lockup', 93)

    def cmd_hardfault_autopilot(self, args):
        '''lockup autopilot for watchdog testing'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'hardfault', 94)

    def cmd_panic_autopilot(self, args):
        '''get ArduPilot to call AP_HAL::panic()'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'panic', 95)

    def cmd_corrupt_param(self, args):
        '''corrupt parameter storage for backup testing'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'corruption', 96)

    def cmd_longloop_autopilot(self, args):
        '''Ask the autopilot to create a long loop'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'long-loop', 97)

    def cmd_internalerror_autopilot(self, args):
        '''Ask the autopilot to create an internal error'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'internal-error', 98)

    def cmd_dfu_boot(self, args):
        '''boot into DFU bootloader without hold'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'DFU-boot-without-hold', 99)

    def cmd_config_error_autopilot(self, args):
        '''Ask the autopilot to jump into its config error loop'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'config-loop', 101)

    def cmd_deadlock(self, args):
        '''trigger a mutex deadlock'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'mutex-deadlock', 100)

    def cmd_nullptr_write(self, args):
        '''write to a low address (nullptr-deref)'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'nullptr-deref-write', 102)

    def cmd_nullptr_read(self, args):
        '''read from a low address (nullptr-deref)'''
        self.cmd_dosomethingreallynastyto_autopilot(args, 'nullptr-deref-read', 103)

    def cmd_battery_reset(self, args):
        '''reset battery remaining'''
        mask = -1
        remaining_pct = 100
        if len(args) > 0:
            mask = int(args[0])
        if len(args) > 1:
            remaining_pct = int(args[1])
        self.master.mav.command_long_send(self.settings.target_system, self.settings.target_component,
                                          mavutil.mavlink.MAV_CMD_BATTERY_RESET, 0,
                                          mask, remaining_pct, 0, 0, 0, 0, 0)

    def cmd_time(self, args):
        '''show autopilot time'''
        tusec = self.master.field('SYSTEM_TIME', 'time_unix_usec', 0)
        if tusec == 0:
            print(tr("no_system_time_time_available"))
            return
        print(tr("msg_7") % (time.ctime(tusec * 1.0e-6), time.ctime()))

    def _cmd_changealt(self, alt, frame):
        '''send commands.  May send both if we don't know which is the
        right one to set'''
        key = (self.target_system, self.target_component)
        supports = self.accepts_DO_CMD_CHANGE_ALTITUDE.get(key, None)
        if supports or supports is None:
            self.master.mav.command_long_send(
                self.settings.target_system,
                self.settings.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
                0,        # confirmation
                alt,      # p1
                frame,    # p2
                0,
                0,
                0,
                0,
                0
            )
            print(tr("sent_change_altitude_command_for_meters") % (alt,))

        if supports is True:
            return

        self.master.mav.mission_item_send(self.settings.target_system,
                                          self.settings.target_component,
                                          0,
                                          frame,
                                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                          3, 1, 0, 0, 0, 0,
                                          0, 0, alt)
        print(tr("sent_change_altitude_mission_item_command") % alt)

    def cmd_changealt(self, args):
        '''change target altitude'''
        if len(args) < 1:
            print(tr("usage_changealt_relaltitude"))
            return
        relalt = float(args[0])
        self._cmd_changealt(relalt, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def cmd_changealt_abs(self, args):
        '''change target altitude'''
        if len(args) < 1:
            print(tr("usage_changealt_abs_absaltitude"))
            return
        absalt = float(args[0])
        self._cmd_changealt(absalt, mavutil.mavlink.MAV_FRAME_GLOBAL)

    def cmd_land(self, args):
        '''auto land commands'''
        if len(args) < 1:
            self.master.mav.command_long_send(self.settings.target_system,
                                              0,
                                              mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                              0, 0, 0, 0, 0, 0, 0, 0)
        elif args[0] == 'abort':
            self.master.mav.command_long_send(self.settings.target_system,
                                              0,
                                              mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
                                              0, 0, 0, 0, 0, 0, 0, 0)
        else:
            print(tr("usage_land_abort"))

    def request_message(self, message_id, p1=0):
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0, # confirmation
            message_id, 0, 0, 0, 0, 0, 0)

    def cmd_version(self, args):
        '''show version'''
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)

    def cmd_capabilities(self, args):
        '''show capabilities'''
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)

    def cmd_rcbind(self, args):
        '''start RC bind'''
        if len(args) < 1:
            print(tr("usage_rcbind_dsmmode"))
            return
        self.master.mav.command_long_send(self.settings.target_system,
                                          self.settings.target_component,
                                          mavutil.mavlink.MAV_CMD_START_RX_PAIR,
                                          0,
                                          float(args[0]), 0, 0, 0, 0, 0, 0)

    def cmd_gethome(self, args):
        '''get home position'''
        self.master.mav.command_long_send(self.settings.target_system,
                                          0,
                                          mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                          0, 0, 0, 0, 0, 0, 0, 0)

    def cmd_led(self, args):
        '''send LED pattern as override'''
        if len(args) < 3:
            print(tr("usage_led_red_green_blue_rate"))
            return
        pattern = [0] * 24
        pattern[0] = int(args[0])
        pattern[1] = int(args[1])
        pattern[2] = int(args[2])

        if len(args) == 4:
            plen = 4
            pattern[3] = int(args[3])
        else:
            plen = 3

        self.master.mav.led_control_send(self.settings.target_system,
                                         self.settings.target_component,
                                         0, 0, plen, pattern)

    def cmd_scripting(self, args):
        '''control onboard scripting'''
        if len(args) < 1:
            print(tr("usage_scripting_stop_restart"))
            return

        if args[0] == 'restart':
            cmd = mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART
        elif args[0] == 'stop':
            cmd = mavutil.mavlink.SCRIPTING_CMD_STOP
        else:
            print(tr("usage_scripting_stop_restart"))
            return

        # MAVProxy command to stop and re-start is: command_int 0 42701 0 0 3 0 0 0 0 0 0

        self.master.mav.command_int_send(
            self.settings.target_system, self.settings.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_SCRIPTING,
            0, 0,
            cmd,
            0, 0, 0, 0, 0, 0)

    def cmd_formatsdcard(self, args):
        '''format SD card'''
        self.master.mav.command_int_send(
            self.settings.target_system, self.settings.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_STORAGE_FORMAT,
            0, 0,
            1, 1,
            0, 0, 0, 0, 0)

    def cmd_oreoled(self, args):
        '''send LED pattern as override, using OreoLED conventions'''
        if len(args) < 4:
            print(tr("usage_oreoled_lednum_red_green_blue"))
            return
        lednum = int(args[0])
        pattern = [0] * 24
        pattern[0] = ord('R')
        pattern[1] = ord('G')
        pattern[2] = ord('B')
        pattern[3] = ord('0')
        pattern[4] = 0
        pattern[5] = int(args[1])
        pattern[6] = int(args[2])
        pattern[7] = int(args[3])

        self.master.mav.led_control_send(self.settings.target_system,
                                         self.settings.target_component,
                                         lednum, 255, 8, pattern)

    def cmd_flashbootloader(self, args):
        '''flash bootloader'''
        self.master.mav.command_long_send(
            self.settings.target_system,
            0,
            mavutil.mavlink.MAV_CMD_FLASH_BOOTLOADER,
            0, 0, 0, 0, 0, 290876, 0, 0
        )

    def cmd_playtune(self, args):
        '''send PLAY_TUNE message'''
        if len(args) < 1:
            print(tr("usage_playtune_tune"))
            return
        tune = args[0]
        str1 = tune[0:30]
        str2 = tune[30:]
        if sys.version_info.major >= 3 and not isinstance(str1, bytes):
            str1 = bytes(str1, "ascii")
        if sys.version_info.major >= 3 and not isinstance(str2, bytes):
            str2 = bytes(str2, "ascii")
        self.master.mav.play_tune_send(self.settings.target_system,
                                       self.settings.target_component,
                                       str1, str2)

    def cmd_repeat(self, args):
        '''repeat a command at regular intervals'''
        if len(args) == 0:
            if len(self.repeats) == 0:
                print(tr("no_repeats"))
                return
            for i in range(len(self.repeats)):
                print("%u: %s" % (i, self.repeats[i]))
            return
        if args[0] == 'add':
            if len(args) < 3:
                print(tr("usage_repeat_add_period_cmd"))
                return
            self.repeats.append(RepeatCommand(float(args[1]), " ".join(args[2:])))
        elif args[0] == 'remove':
            if len(args) < 2:
                print(tr("usage_repeat_remove_index"))
                return
            i = int(args[1])
            if i < 0 or i >= len(self.repeats):
                print(tr("invalid_index") % i)
                return
            self.repeats.pop(i)
            return
        elif args[0] == 'toggle':
            if len(args) < 2:
                print(tr("usage_repeat_toggle_index"))
                return

            for i in range(1, len(args)):
                try:
                    i = int(args[i])
                except ValueError:
                    print(tr("unable_to_toggle_index_is_not") % (args[i],))
                    continue
                if i < 0 or i >= len(self.repeats):
                    print(tr("unable_to_toggle_invalid_index") % (i,))
                    continue
                self.repeats[i].enabled = not self.repeats[i].enabled
                print(f"{i}: {self.repeats[i]}")
            return
        elif args[0] == 'clean':
            self.repeats = []
        else:
            print(tr("usage_repeat_add_remove_clean"))

    def cmd_devid(self, args):
        '''decode device IDs from parameters'''
        for p in self.mav_param.keys():
            if p.startswith('COMPASS_DEV_ID') or p.startswith('COMPASS_PRIO') or (
                    p.startswith('COMPASS') and p.endswith('DEV_ID')):
                mp_util.decode_devid(self.mav_param[p], p)
            if p.startswith('INS') and p.endswith('_ID'):
                mp_util.decode_devid(self.mav_param[p], p)
            if p.startswith('GND_BARO') and p.endswith('_ID'):
                mp_util.decode_devid(self.mav_param[p], p)
            if p.startswith('BARO') and p.endswith('_DEVID'):
                mp_util.decode_devid(self.mav_param[p], p)
            if p.startswith('ARSPD') and p.endswith('_DEVID'):
                mp_util.decode_devid(self.mav_param[p], p)

    def cmd_setorigin(self, args):
        '''set global origin'''
        if len(args) < 3:
            print(tr("usage_setorigin_lat_deg_lon_deg"))
            return
        lat = float(args[0])
        lon = float(args[1])
        alt = float(args[2])
        print(tr("setting_origin_to"), lat, lon, alt)
        self.master.mav.set_gps_global_origin_send(
            self.settings.target_system,
            int(lat*10000000), # lat
            int(lon*10000000), # lon
            int(alt*1000)) # param7

    def cmd_magset_field(self, args):
        '''set compass offsets by field'''
        if len(args) < 3:
            print(tr("usage_magsetfield_magx_magy_magz"))
            return
        magX = int(args[0])
        magY = int(args[1])
        magZ = int(args[2])

        field1x = self.master.field('RAW_IMU', 'xmag', 0)
        field1y = self.master.field('RAW_IMU', 'ymag', 0)
        field1z = self.master.field('RAW_IMU', 'zmag', 0)

        field2x = self.master.field('SCALED_IMU2', 'xmag', 0)
        field2y = self.master.field('SCALED_IMU2', 'ymag', 0)
        field2z = self.master.field('SCALED_IMU2', 'zmag', 0)

        field3x = self.master.field('SCALED_IMU3', 'xmag', 0)
        field3y = self.master.field('SCALED_IMU3', 'ymag', 0)
        field3z = self.master.field('SCALED_IMU3', 'zmag', 0)

        self.param_set('COMPASS_OFS_X', magX - (field1x - self.get_mav_param('COMPASS_OFS_X', 0)))
        self.param_set('COMPASS_OFS_Y', magY - (field1y - self.get_mav_param('COMPASS_OFS_Y', 0)))
        self.param_set('COMPASS_OFS_Z', magZ - (field1z - self.get_mav_param('COMPASS_OFS_Z', 0)))

        self.param_set('COMPASS_OFS2_X', magX - (field2x - self.get_mav_param('COMPASS_OFS2_X', 0)))
        self.param_set('COMPASS_OFS2_Y', magY - (field2y - self.get_mav_param('COMPASS_OFS2_Y', 0)))
        self.param_set('COMPASS_OFS2_Z', magZ - (field2z - self.get_mav_param('COMPASS_OFS2_Z', 0)))

        self.param_set('COMPASS_OFS3_X', magX - (field3x - self.get_mav_param('COMPASS_OFS3_X', 0)))
        self.param_set('COMPASS_OFS3_Y', magY - (field3y - self.get_mav_param('COMPASS_OFS3_Y', 0)))
        self.param_set('COMPASS_OFS3_Z', magZ - (field3z - self.get_mav_param('COMPASS_OFS3_Z', 0)))

    def cmd_magreset_ofs(self, args):
        '''set compass offsets to all zero'''
        self.param_set('COMPASS_OFS_X', 0)
        self.param_set('COMPASS_OFS_Y', 0)
        self.param_set('COMPASS_OFS_Z', 0)
        self.param_set('COMPASS_DIA_X', 1)
        self.param_set('COMPASS_DIA_Y', 1)
        self.param_set('COMPASS_DIA_Z', 1)
        self.param_set('COMPASS_ODI_X', 0)
        self.param_set('COMPASS_ODI_Y', 0)
        self.param_set('COMPASS_ODI_Z', 0)

        self.param_set('COMPASS_OFS2_X', 0)
        self.param_set('COMPASS_OFS2_Y', 0)
        self.param_set('COMPASS_OFS2_Z', 0)
        self.param_set('COMPASS_DIA2_X', 1)
        self.param_set('COMPASS_DIA2_Y', 1)
        self.param_set('COMPASS_DIA2_Z', 1)
        self.param_set('COMPASS_ODI2_X', 0)
        self.param_set('COMPASS_ODI2_Y', 0)
        self.param_set('COMPASS_ODI2_Z', 0)

        self.param_set('COMPASS_OFS3_X', 0)
        self.param_set('COMPASS_OFS3_Y', 0)
        self.param_set('COMPASS_OFS3_Z', 0)
        self.param_set('COMPASS_DIA3_X', 1)
        self.param_set('COMPASS_DIA3_Y', 1)
        self.param_set('COMPASS_DIA3_Z', 1)
        self.param_set('COMPASS_ODI3_X', 0)
        self.param_set('COMPASS_ODI3_Y', 0)
        self.param_set('COMPASS_ODI3_Z', 0)

    def cmd_namedvaluefloat(self, args):
        '''send a NAMED_VALUE_FLOAT'''
        if len(args) < 2:
            print(tr("usage_namedvaluefloat_name_value"))
            return
        tnow_ms = int((time.time() - self.mpstate.start_time_s)*1000)
        name = args[0]
        value = float(args[1])
        self.master.mav.named_value_float_send(tnow_ms, name.encode("utf-8"), value)

    def cmd_canforward(self, args):
        if len(args) < 1:
            print(tr("usage_canforward_bus"))
            return
        bus = int(args[0])
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_CAN_FORWARD,
            0,
            bus,
            0,
            0,
            0,
            0,
            0,
            0)

    def cmd_landing_gear(self, args):
        usage = tr("usage_usage_gear_up_down_id_alt_gear_extend")
        if len(args) == 0 or args[0] not in ['up', 'down', 'extend', 'retract']:
            print(usage)
            return
        if args[0] in ['down', 'extend']:
            DesiredState = 0
        elif args[0] in ['up', 'retract']:
            DesiredState = 1
        else:
            print(usage)
            return
        if len(args) == 1:
            ID = -1
        elif len(args) == 2:
            try:
                ID = int(args[1])
            except ValueError:
                print(usage)
                return
            if ID < -1:
                print(usage)
                return
        else:
            print(usage)
            return
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION , 0,
            ID,
            DesiredState,
            0, 0, 0, 0, 0, 0
        )

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()

        if mtype == "COMMAND_ACK":
            # check to see if the vehicle has bounced our attempts to
            # set the current mission item via mavlink command (as
            # opposed to the old message):
            if m.command == mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE:
                key = (m.get_srcSystem(), m.get_srcComponent())
                if m.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    # stop sending the commands:
                    self.accepts_DO_CMD_CHANGE_ALTITUDE[key] = False
                elif m.result in [mavutil.mavlink.MAV_RESULT_ACCEPTED]:
                    self.accepts_DO_CMD_CHANGE_ALTITUDE[key] = True

    def idle_task(self):
        '''called on idle'''
        for r in self.repeats:
            if r.enabled and r.event.trigger():
                self.mpstate.functions.process_stdin(r.cmd, immediate=True)


def init(mpstate):
    '''initialise module'''
    return MiscModule(mpstate)
