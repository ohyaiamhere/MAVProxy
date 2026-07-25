#!/usr/bin/env python3
'''
movinghome module
André Kjellstrup, Norce

This module can update the home position of the ArduPilot vehicle to the position of a moving GCS.
requires package python-nmea2

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math
import serial
from MAVProxy.modules.lib.mp_i18n import tr
try:
    import pynmea2
except ImportError as e:
    print(tr("missing_package_do_sudo_apt_install"))
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class movinghome(mp_module.MPModule):
    def __init__(self, mpstate):
        #Initialise module
        super(movinghome, self).__init__(mpstate, "movinghome", "")
        #latest GCS coordinates
        self.lat = 0
        self.lon = 0
        self.alt = 0
        #last/set home coordinates
        self.lath = 0
        self.lonh = 0  
        self.alth = 0
        #misc settings
        self.device = "/dev/ttyUSB0"
        self.baud = 4800
        self.updating=False
        self.radius = 15 #min travelled distance (m) before update
        self.check_interval = 3 # seconds
        self.last_check = time.time()
        self.fresh = True #fresh start/first movement
        self.dist = 0
        self.last_decode_error_print = 0

        print(tr("default_nmea_source_is_at_baud") % (self.device , self.baud))
        self.add_command('movinghome', self.cmd_movinghome, tr("cmd_movinghome_module"))

            

    def cmd_movinghome(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(tr("usage_movinghome_status_on_off_radius"))
        elif args[0] == "status":
            self.status()
        elif args[0] == "on":
            self.movinghome_on()
        elif args[0] == "off":
            self.movinghome_off()
        elif args[0] == "radius":
            if len(args) < 2:
                print(tr("usage_moving_base_minimum_travel_radius"))
                return
            self.radius=float(args[1])
        elif args[0] == "device":
            if len(args) < 2:
                print(tr("usage_device_name_device_dev_ttyusb2"))
                return
            self.device=args[1]
        elif args[0] == "baud":
            if len(args) < 2:
                print(tr("usage_baud_rate_baud_9600"))
                return
            self.baud=args[1]
        else:
            print(self.usage)

    def status(self):
        #Returns information about module'''
        if self.updating == True:
            print(tr("last_known_gcs_position_lat_lon") %
                   {"lat": self.lath,
                    "lon": self.lonh,
                    "max": self.radius,
                   })
        else:
            print(tr("not_updating_home"))
        print(tr("radius_is_m_interval_is_s") % (self.radius, self.check_interval, self.device, self.baud ))


    def movinghome_on(self):
        #self.ser = serial.Serial('/dev/ttyUSB0',4800)
        self.ser = serial.Serial(self.device,self.baud)
        self.updating=True
        self.lath = 0 # ensure push of current home.
        print(tr("home_position_will_be_updated_if") %
               {"max": self.radius,
               })

    def movinghome_off(self):
        self.updating = False
        print(tr("home_position_will_not_be_updated"))

    def idle_task(self):
        #Called frequently by mavproxy
        if self.updating == True:
            data = self.ser.readline()
            try:
                data = data.decode("ascii")
            except UnicodeDecodeError as e:
                # this is probably a baudrate issue
                now = time.time()
                if now - self.last_decode_error_print > 10:
                    print(tr("movinghome_decode_error_baudrate_issue"))
                    self.last_decode_error_print = now
                return
            if (data.startswith("$GPGGA")):
                msg = pynmea2.parse(data)
                if int(msg.num_sats) > 5:
                    #convert LAT
                    DD = int(float(msg.lat)/100)
                    MM = float(msg.lat) - DD * 100
                    self.lat = DD + MM/60
                    if msg.lat_dir == "S":
                        self.lat = -self.lat;
                    #convert LON
                    DD = int(float(msg.lon)/100)
                    MM = float(msg.lon) - DD * 100
                    self.lon = DD + MM/60
                    if msg.lon_dir == "W":
                        self.lon = -self.lon;

                now = time.time()
                if now-self.last_check > self.check_interval:
                    self.last_check = now
                    #check if we moved enough
                    self.dist = self.haversine(self.lon, self.lat, self.alt, self.lonh, self.lath, self.alth)
                    if self.dist > self.radius:
                        if self.fresh == True:
                            self.say(tr("gcs_position_set_as_home"))    
                            self.fresh = False
                        else:
                            message = "GCS moved "
                            message2 = message + "%.0f" % self.dist + "meters"
                            self.say("%s: %s" % (self.name,message2))
                            message2_enc = message2.encode(bytes)
                            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message2)
                        self.console.writeln(tr("home_position_updated"))

                        self.master.mav.command_int_send(
                        self.settings.target_system, self.settings.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                        1, # (1, set current location as home)
                        0, # move on
                        0, # param1
                        0, # param2
                        0, # param3
                        0, # param4
                        int(self.lat*1e7), # param5
                        int(self.lon*1e7), # param6
                        0) # param7

                        self.lath = self.lat
                        self.lonh = self.lon
                        #print data
                        self.console.writeln(tr("gnss_quality_sats")% (self.name,self.lat,self.lon,msg.gps_qual,msg.num_sats))


    def haversine(self, lon1, lat1, alt1, lon2, lat2, alt2):
        r_earth = 6371000
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        d = c*r_earth
        return math.sqrt(d**2+(alt1 - alt2)**2)


def init(mpstate):
    return movinghome(mpstate)
