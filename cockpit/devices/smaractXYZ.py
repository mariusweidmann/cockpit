#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2018 Mick Phillips <mick.phillips@gmail.com>
## Copyright (C) 2018 Ian Dobbie <ian.dobbie@bioch.ox.ac.uk>
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

## Copyright 2013, The Regents of University of California
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## 1. Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## 3. Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived
##   from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
## LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
## ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.


from wx.core import Position
from cockpit import events
import cockpit.gui.guiUtils
import cockpit.handlers.stagePositioner
from cockpit import interfaces
import cockpit.util.logger
import cockpit.util.threads
import cockpit.util.userConfig

from cockpit.devices.device import Device
from OpenGL.GL import *
import serial
import threading
import time
import wx
import re # to get regular expression parsing for config file

import sys
import os
#import cockpit.drivers.smaractctl.smaract.ctl.bindings
import smaract.ctl as ctl
VELOCITY = 5000000000
ACCELERATION = 100000000000
scale = 1000000



## This module is for Smaract MCS2-controlled XYZ stage.
# Sample config entry:
#  [smaractXYZ]
# incomplete
#  softlimits: ((-15000, -15000, -15000), (15000, 15000, 15000))
#

LIMITS_PAT = r"(?P<limits>\(\s*\(\s*[-]?\d*\s*,\s*[-]?\d*\s*\)\s*,\s*\(\s*[-]?\d*\s*\,\s*[-]?\d*\s*\)\))"

## TODO:  Test with hardware.
## TODO:  These parameters should be factored out to a config file.

d_handle = int
d_handle = 0
d_handle = int(d_handle)


class SmaractXYZ(Device):
    def __init__(self, name, config):
        super().__init__(name, config)
        ## Connection to the XYZ stage controller (serial.Serial instance)
        self.xyzConnection = None
        ## Lock around sending commands to the XYZ stage controller.
        self.xyzLock = threading.Lock()
        ## Cached copy of the stage's position. Initialized to an impossible
        # value; this will be modified in initialize.
        self.xyzPositionCache = (10 ** 100, 10 ** 100, 10 ** 100)
        ## Target positions for movement in X, Y and Z, or None if that ch is 
        # not moving.
        self.xyzMotionTargets = [None, None, None]

        ## If there is a config section for the smaractMCS2, grab the config and
        # subscribe to events.
        try :
            limitString = config.get('softlimits')
            parsed = re.search(LIMITS_PAT, limitString)
            if not parsed:
                # Could not parse config entry.
                raise Exception('Bad config: smaractMCS2XYZ Limits.')
                # No transform tuple
            else:
                lstr = parsed.groupdict()['limits']
                self.softlimits=eval(lstr)
        except:
            print ("No softlimits section setting default limits")
            self.softlimits = ((-15000, -15000, -15000), (15000, 15000, 15000))

        events.subscribe(events.USER_ABORT, self.onAbort)
        #events.subscribe('macro stage xyz draw', self.onMacroStagePaint)
        #events.subscribe('cockpit initialization complete', self.promptExerciseStage)


    def initialize(self):
        try:
            buffer = ctl.FindDevices("")
            if len(buffer) == 0:
                print("MCS2 no devices found.")
                sys.exit(1)
            locators = buffer.split("\n")
            for locator in locators:
                print("MCS2 available devices: {}".format(locator))
        except:
            print("MCS2 failed to find devices. Exit.")
            input()
            sys.exit(1)

        try:
            # Open the first MCS2 device from the list
            global d_handle
            d_handle = ctl.Open(locators[0])

            print("MCS2 opened {}.".format(locators[0]))
                # Get the proper initial position.
            self.getXYZPosition()
            for ch in range(2):
                move_mode = ctl.MoveMode.CL_ABSOLUTE
            if (self.findReference() != True):
                sys.exit(1)


        except ctl.Error as e:
        # Passing an error code to "GetResultInfo" returns a human readable string
        # specifying the error.
            print("MCS2 {}: {}, error: {} (0x{:04X}) in line: {}.".format(e.func, ctl.GetResultInfo(e.code), ctl.ErrorCode(e.code).name, e.code, (sys.exc_info()[-1].tb_lineno)))

        except Exception as ex:
            print("Unexpected error: {}, {} in line: {}".format(ex, type(ex), (sys.exc_info()[-1].tb_lineno)))
            raise




    ## Home the motors.
    def findReference(self):
        msg_OK = "Successfully referenced ch "
        msg_FAIL = "There was a problem homing ch "
        for ch in reversed(range(3)):
            r_id = ctl.RequestReadProperty(d_handle, ch, ctl.Property.CHANNEL_STATE, 0)
            state = ctl.ReadProperty_i32(d_handle, r_id)
            if (state & ctl.ChannelState.IS_CALIBRATED) == 0: #not calibrated (please do this by manually)
                cockpit.gui.guiUtils.showHelpDialog(None, 'Channel '+ch+' is NOT CALIBRATED)')
                sys.exit(1)
            if (state & ctl.ChannelState.IS_REFERENCED) == 0:
                busy_box = wx.ProgressDialog(parent = None, title = 'Busy...', message = 'Homing stage ch '+str(ch))
                busy_box.Show()
                ctl.Reference(d_handle, ch)
                while (ctl.GetProperty_i32(d_handle, ch, ctl.Property.CHANNEL_STATE) & ctl.ChannelState.ACTIVELY_MOVING != 0):
                    time.sleep(0.1)
                busy_box.Hide()
                busy_box.Destroy()
                # Was homing successful?
                r_id = ctl.RequestReadProperty(d_handle, ch, ctl.Property.CHANNEL_STATE, 0)
                state = ctl.ReadProperty_i32(d_handle, r_id)
                if (state & ctl.ChannelState.IS_REFERENCED) != 0:
                    msg_OK += str(ch) + ' '
                    self.sendXYZPositionUpdates()
                else:
                    msg_FAIL += str(ch)
                    cockpit.gui.guiUtils.showHelpDialog(None, msg_FAIL)
                    return(False)

        cockpit.gui.guiUtils.showHelpDialog(None, msg_OK)
        return(True)


    ## When the user logs out, switch to open-loop mode.
    def onExit(self):
        # Switch to open loop?
        self.xyzConnection.close()


    ## Halt XYZ motion when the user aborts. Note we can't control Z motion
    # here because the piezo is under the DSP's control.
    def onAbort(self, *args):
        ctl.Stop(d_handle, 2)
        ctl.Stop(d_handle, 1)
        ctl.Stop(d_handle, 0)
        ctl.Stop(d_handle, 2)
        ctl.Stop(d_handle, 1)
        ctl.Stop(d_handle, 0)


    def getHandlers(self):
        result = []
        # NB these motion limits are more restrictive than the stage's true
        # range of motion, but they are needed to keep the stage from colliding
        # with the objective. 
        for ch, minPos, maxPos in [(0, self.softlimits[0][0],self.softlimits[1][0]),
                    (1, self.softlimits[0][1],self.softlimits[1][1]), (2, self.softlimits[0][2],self.softlimits[1][2])]:
            result.append(cockpit.handlers.stagePositioner.PositionerHandler(
                    "%d Smaract mover" % ch, "%d stage motion" % ch, False,
                    {'moveAbsolute': self.moveXYZAbsolute,
                         'moveRelative': self.moveXYZRelative,
                         'getPosition': self.getXYZPosition},
                    ch, (minPos, maxPos), (minPos, maxPos)))
        return result


    def moveXYZAbsolute(self, ch, pos):
        #print("moveXYZAbsolute " + str(ch) + " " + str(pos))
        with self.xyzLock:
            if self.xyzMotionTargets[ch] is not None:
                # Don't stack motion commands for the same ch
                return
        self.xyzMotionTargets[ch] = pos
        #this should be set every time, as it may help with controling the stage via the handheld module and cockpit at the same time (according to manufacturer)
        ctl.SetProperty_i32(d_handle, ch, ctl.Property.MOVE_MODE, ctl.MoveMode.CL_ABSOLUTE)
        ctl.SetProperty_i64(d_handle, ch, ctl.Property.MOVE_VELOCITY, VELOCITY)
        ctl.SetProperty_i64(d_handle, ch, ctl.Property.MOVE_ACCELERATION, ACCELERATION)
        # The factor of 10000000 converts from µm to pm.
        #print('MOVING ' + str(ch) + ' ' + str(self.xyzMotionTargets[ch]))
        ctl.Move(d_handle, ch, int(pos * scale))
        self.sendXYZPositionUpdates()


    def moveXYZRelative(self, ch, delta):
        if not delta:
            # Received a bogus motion request.
            return
        #print("moveXYZRelative " + str(ch) + " " + str(delta))
        curPos = self.xyzPositionCache[ch]
        self.moveXYZAbsolute(ch, curPos + delta)


    ## Send updates on the XYZ stage's position, until it stops moving.
    ## TODO: Query stage to see if it still actively moving, don't rely on a change in the positions!
    @cockpit.util.threads.callInNewThread
    def sendXYZPositionUpdates(self):
        while True:
            prevX, prevY, prevZ = self.getXYZPosition()
            time.sleep(0.001)
            x, y, z = self.getXYZPosition()
            delta = abs(x - prevX) + abs(y - prevY) + abs(z-prevZ) + abs(z - prevZ)
            if delta < 1.:
                print(time.time_ns(), "no movement")
                # No movement since last time; done moving.
                for axis in [0, 1, 2]:
                    events.publish(events.STAGE_STOPPED, '%d Smaract mover' % axis)
                with self.xyzLock:
                    self.xyzMotionTargets = [None, None, None]
                return

            for axis in [0, 1, 2]:
                events.publish(events.STAGE_MOVER, axis)
            time.sleep(.01)



    ## Get the position of the specified ch, or both axes by default.
    def getXYZPosition(self, ch = None):
        # Positions are in pm, and we need µm.
        x = ctl.GetProperty_i64(d_handle, 0, ctl.Property.POSITION) /1000000
        y = ctl.GetProperty_i64(d_handle, 1, ctl.Property.POSITION) /1000000
        z = ctl.GetProperty_i64(d_handle, 2, ctl.Property.POSITION) /1000000
        self.xyzPositionCache = (x, y, z)
        #print(self.xyzPositionCache)
        if ch is None:
            return self.xyzPositionCache
        return self.xyzPositionCache[ch]


    def makeInitialPublications(self):
        self.sendXYZPositionUpdates()
