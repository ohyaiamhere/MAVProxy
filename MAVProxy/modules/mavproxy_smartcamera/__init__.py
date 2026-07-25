#!/usr/bin/env python3
#***************************************************************************
#                      Copyright Jaime Machuca
#***************************************************************************
# Title        : mavproxy_smartcamera.py
#
# Description  : This file is intended to be added as a module to MAVProxy,
#                it is intended to be used to control smart cameras that are
#                connected to a companion computer. It reads MAVlink commands
#                and uses them to control the cameras attached. The module
#                reads a configuration file called smart_camera.cnf that tells
#                it what cameras are connected, it then tries to connect to the
#                cameras and populates a list of available cameras.
#
# Environment  : Intended to be included in MAVproxy as a Module
#
# Responsible  : Jaime Machuca
#
# License      : GNU GPL version 3
#
# Editor Used  : Xcode 6.1.1 (6A2008a)
#
#****************************************************************************

#****************************************************************************
# HEADER-FILES (Only those that are needed in this file)
#****************************************************************************

# System Header files and Module Headers
import time, math, sched, threading

# Module Dependent Headers
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting


# Own Headers
from sc_webcam import SmartCameraWebCam
from sc_SonyQX1 import SmartCamera_SonyQX
import sc_config
from MAVProxy.modules.lib.mp_i18n import tr

#****************************************************************************
# LOCAL DEFINES
#****************************************************************************


#****************************************************************************
# Class name       : SmartCameraModule
#
# Public Methods   : init
#                    mavlink_packet
#
# Private Methods  : __vRegisterCameras
#                    __vCmdCamTrigger
#
#****************************************************************************
class SmartCameraModule(mp_module.MPModule):

#****************************************************************************
#   Method Name     : __init__ Class Initializer
#
#   Description     : Initializes the class
#
#   Parameters      : mpstate
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __init__(self, mpstate):
        super(SmartCameraModule, self).__init__(mpstate, "SmartCamera", tr("mod_smartcamera_commands"))
        self.add_command('camtrigger', self.__vCmdCamTrigger, tr("cmd_trigger_camera"))
        self.add_command('connectcams', self.__vCmdConnectCameras, tr("cmd_connect_to_cameras"))
        self.add_command('setCamISO', self.__vCmdSetCamISO, tr("cmd_set_camera_iso"))
        self.add_command('setCamAperture', self.__vCmdSetCamAperture, tr("cmd_set_camera_aperture"))
        self.add_command('setCamShutterSpeed', self.__vCmdSetCamShutterSpeed, tr("cmd_set_camera_shutter_speed"))
        self.add_command('setCamExposureMode', self.__vCmdSetCamExposureMode, tr("cmd_set_camera_exposure_mode"))
        self.add_command('getAllPictures', self.__vCmdGetAllPictures, tr("cmd_download_all_flight_pictures_filename_as_argument_optio"))
        self.CamRetryScheduler = sched.scheduler(time.time, time.sleep)
        self.ProgramAuto = 1
        self.Aperture = 2
        self.Shutter = 3
        self.Manual = 4
        self.IntelligentAuto = 5
        self.SuperiorAuto = 6
        self.WirelessPort = sc_config.config.get_string("general", 'WirelessPort', "wlan0")
        self.u8RetryTimeout = 0
        self.u8MaxRetries = 5
        self.tLastCheckTime = time.time()
        self.u8KillHeartbeatTimer = 100
        self.__vRegisterCameras()

        self.mpstate = mpstate
        
        # Start a 10 second timer to kill heartbeats as a workaround
        # threading.Timer(10, self.__vKillHeartbeat).start()
    
#****************************************************************************
#   Method Name     : __vKillHeartbeat
#
#   Description     : Sets heartbeat setting to 0 to stop sending heartbeats
#                     this is a temporary workaround for systems that do not
#                     properly interpret the heartbeat contents like 3DR Solo
#                     in such systems the heartbeats from the camera controller
#                     and the main system are confused causing potential issues
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vKillHeartbeat(self):
        print(tr("killing_heartbeat_solo_workaround"))
        self.mpstate.settings.heartbeat = 0

 #****************************************************************************
 #   Method Name     : __vRegisterQXCamera
 #
 #   Description     : Tries to connect to a QX camera on the specified Wireless
 #                     port. If no camera is found it will retry every 5 seconds
 #                     until u8MaxRetries is reached.
 #
 #   Parameters      : None
 #
 #   Return Value    : None
 #
 #   Author           : Jaime Machuca
 #
 #****************************************************************************

    def __vRegisterQXCamera(self,u8CamNumber):
        if (self.u8RetryTimeout < self.u8MaxRetries):
            new_camera = SmartCamera_SonyQX(u8CamNumber, self.WirelessPort)
            if new_camera.boValidCameraFound() is True:
                self.camera_list = self.camera_list + [new_camera]
                print(tr("found_qx_camera"))
                self.master.mav.statustext_send(6,"Camera Controller: Found QX Camera, Ready to Fly")
            else:
                print(tr("no_valid_camera_found_retry_in"))
                self.u8RetryTimeout = self.u8RetryTimeout + 1
                self.CamRetryScheduler.enter(5, 1, self.__vRegisterQXCamera, [u8CamNumber])
                self.CamRetryScheduler.run()
        else:
            print(tr("max_retries_reached_no_qx_camera"))
            self.master.mav.statustext_send(3,"Camera Controller: Warning! Camera not found")
            self.u8RetryTimeout = 0

#****************************************************************************
#   Method Name     : __vRegisterCameras
#
#   Description     : Creates camera objects based on camera-type configuration
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vRegisterCameras(self):

        # initialise list
        self.camera_list = []

        #look for up to 2 cameras
        for i in range(0,2):
            config_group = "camera%d" % i
            camera_type = sc_config.config.get_integer(config_group, 'type', 0)
            # webcam
            if camera_type == 1:
                new_camera = SmartCameraWebCam(i)
                self.camera_list = self.camera_list + [new_camera]

            # Sony QX1
            if camera_type == 2:
                self.__vRegisterQXCamera(i)

        # display number of cameras found
        print (tr("cameras_found") % len(self.camera_list))

#****************************************************************************
#   Method Name     : __vCmdCamTrigger
#
#   Description     : Triggers all the cameras and stores Geotag information
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamTrigger(self, args):
        '''Trigger Camera'''
        #print(self.camera_list)
        for cam in self.camera_list:
            cam.take_picture()
            print(tr("trigger_cam") % cam)

#****************************************************************************
#   Method Name     : __vCmdConnectCameras
#
#   Description     : Initiates connection to cameras
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdConnectCameras(self, args):
        '''ToDo: Validate the argument as a valid port'''
        if len(args) >= 1:
            self.WirelessPort = args[0]
        print (tr("connecting_to_cameras_on") % self.WirelessPort)
        self.__vRegisterCameras()

#****************************************************************************
#   Method Name     : __vCmdSetCamExposureMode
#
#   Description     : Sets the camera exposure mode
#
#   Parameters      : Exposure Mode, Cam number
#                     Valid values are Program Auto, Aperture, Shutter, Manual
#                     Intelligent Auto, Superior Auto
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamExposureMode(self, args):
        '''ToDo: Validate CAM number and Valid Mode Values'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetExposureMode(args[0])
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetExposureMode(args[0])
        else:
            print (tr("usage_setcamexposuremode_mode_camnumber_valid_valu"))

#****************************************************************************
#   Method Name     : __vCmdSetCamAperture
#
#   Description     : Sets the camera aperture
#
#   Parameters      : Aperture Value, Cam number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamAperture(self, args):
        '''ToDo: Validate CAM number and Valid Aperture Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetAperture(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetAperture(int(args[0]))
        else:
            print (tr("usage_setcamaperture_aperture_camnumber_aperture_i"))

#****************************************************************************
#   Method Name     : __vCmdSetCamShutterSpeed
#
#   Description     : Sets the shutter speed for the camera
#
#   Parameters      : Shutter speed, Cam Number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamShutterSpeed(self, args):
        '''ToDo: Validate CAM number and Valid Shutter Speed'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetShutterSpeed(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetShutterSpeed(int(args[0]))
        else:
            print (tr("usage_setcamshutterspeed_shuttervalue_camnumber_sh"))

#****************************************************************************
#   Method Name     : __vCmdSetCamISO
#
#   Description     : Sets the ISO value for the camera
#
#   Parameters      : ISO Value, Cam Number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamISO(self, args):
        '''ToDo: Validate CAM number and Valid ISO Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetISO(args[0])
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetISO(args[0])
        else:
            print (tr("usage_setcamiso_isovalue_camnumber"))

#****************************************************************************
#   Method Name     : __vCmdCamZoomIn
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomIn(self):
        for cam in self.camera_list:
            cam.boZoomIn()

#****************************************************************************
#   Method Name     : __vCmdCamZoomOut
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomOut(self):
        for cam in self.camera_list:
            cam.boZoomOut()

#****************************************************************************
#   Method Name     : __vCmdGetAllPictures
#
#   Description     : Downloads all the pics taken during this flight
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdGetAllPictures(self, args):
        
        #Download Pictures
        if len(args) >= 1:
            slogFileName = args[0]
            for cam in self.camera_list:
                print(tr("init_picture_download_for_cam_from") % cam, slogFileName)
                cam.boGetAllSessionPictures(slogFileName)
        else:
            for cam in self.camera_list:
                print(tr("init_picture_download_for_cam") % cam)
                cam.boGetAllSessionPictures(0)
    
#****************************************************************************
#   Method Name     : __vDecodeDIGICAMConfigure
#
#   Description     : Decode and process the camera configuration Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMConfigure(self, mCommand_Long):
        if mCommand_Long.param1 != 0:
            print (tr("exposure_mode") % mCommand_Long.param1)

            if mCommand_Long.param1 == self.ProgramAuto:
                self.__vCmdSetCamExposureMode(["Program Auto"])

            elif mCommand_Long.param1 == self.Aperture:
                self.__vCmdSetCamExposureMode(["Aperture"])

            elif mCommand_Long.param1 == self.Shutter:
                self.__vCmdSetCamExposureMode(["Shutter"])

        '''Shutter Speed'''
        if mCommand_Long.param2 != 0:
            print (tr("shutter_speed") % mCommand_Long.param2)
            self.__vCmdSetCamShutterSpeed([mCommand_Long.param2])

        '''Aperture'''
        if mCommand_Long.param3 != 0:
            print (tr("aperture") % mCommand_Long.param3)
            self.__vCmdSetCamAperture([mCommand_Long.param3])

        '''ISO'''
        if mCommand_Long.param4 != 0:
            print (tr("iso") % mCommand_Long.param4)
            self.__vCmdSetCamISO([mCommand_Long.param4])

        '''Exposure Type'''
        if mCommand_Long.param5 != 0:
            print (tr("exposure_type") % mCommand_Long.param5)


#****************************************************************************
#   Method Name     : __vDecodeDIGICAMControl
#
#   Description     : Decode and process the camera control Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMControl(self, mCommand_Long):
        '''Session'''
        if mCommand_Long.param1 != 0:
            print (tr("session") % mCommand_Long.param1)

        '''Zooming Step Value'''
        if mCommand_Long.param2 != 0:
            print (tr("zooming_step") % mCommand_Long.param2)

        '''Zooming Step Value'''
        if mCommand_Long.param3 != 0:
            print (tr("zooming_value") % mCommand_Long.param3)

            if (mCommand_Long.param3 == 1):
                self.__vCmdCamZoomIn()
            elif (mCommand_Long.param3 == -1):
                self.__vCmdCamZoomOut()
            else:
                print (tr("invalid_zoom_value"))

        '''Focus 0=Unlock/1=Lock/2=relock'''
        if mCommand_Long.param4 != 0:
            print (tr("focus") % mCommand_Long.param4)

        '''Trigger'''
        if mCommand_Long.param5 != 0:
            print (tr("trigger") % mCommand_Long.param5)
            self.__vCmdCamTrigger(mCommand_Long)



#****************************************************************************
#   Method Name     : mavlink_packet
#
#   Description     : MAVProxy requiered callback function used to receive MAVlink
#                     packets
#
#   Parameters      : MAVLink Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            for cam in self.camera_list:
                cam.boSet_GPS(m)
        if mtype == "ATTITUDE":
            for cam in self.camera_list:
                cam.boSet_Attitude(m)
        if mtype == "CAMERA_STATUS":
            print (tr("got_message_camera_status"))
        if mtype == "CAMERA_FEEDBACK":
            print (tr("got_message_camera_feedback"))
            '''self.__vCmdCamTrigger(m)'''
        if mtype == "COMMAND_LONG":
            if m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE:
                print (tr("got_message_digicam_configure"))
                self.__vDecodeDIGICAMConfigure(m)
            elif m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
                print (tr("got_message_digicam_control"))
                self.__vDecodeDIGICAMControl(m)

#****************************************************************************
#   Method Name     : idle_task
#
#   Description     : used for heartbeat work arround timer
#
#   Parameters      : none
#
#   Return Value    : none
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def idle_task(self):
        now = time.time()
        if not self.u8KillHeartbeatTimer == 0 and self.tLastCheckTime > 1:
            self.tLastCheckTime = now
            self.u8KillHeartbeatTimer -= 1
            if self.u8KillHeartbeatTimer == 0:
                self.__vKillHeartbeat();

#****************************************************************************
#   Method Name     : init
#
#   Description     :
#
#   Parameters      : mpstate
#
#   Return Value    : SmartCameraModule Instance
#
#   Author           : Jaime Machuca
#
#****************************************************************************

def init(mpstate):
    '''initialise module'''
    return SmartCameraModule(mpstate)
