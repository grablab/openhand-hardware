#!/usr/bin/python
#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## Controlling Robotis Dynamixel RX-28 & RX-64 servos from python
## using the USB2Dynamixel adaptor.

## Authors: Travis Deyle, Advait Jain & Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)

##MOD VERSION: 2013, some additional modifications made to make appropriate for both MX/RX series (Raymond R. Ma)
##MOD VERSION: 2018,2019, additions for python3, searching for dynamixels, and readdressing now available (Andrew Morgan)
##                   Additional Changes for Robotis_Servo_X for all readings of new dynamixels.
##MOD VERSION: 2020, additions for XL series servos (Walter G. Bircher)

from __future__ import print_function
import serial
import time
import _thread
import sys, optparse
import math
import random
import ctypes
import numpy as np
import IPython

from registerDict import *
import registerDict


class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57142):
        try:
            self.dev_name = string.atoi( dev_name ) # stores the serial port as 0-based integer for Windows
        except:
            self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self.mutex = _thread.allocate_lock()	#helps ensure that only a single port is used?
        self.servo_dev = None

        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def send_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.flushInput()		#added to remove extra bytes from input buffer
        byte_msg = bytearray(msg)
        sent = self.servo_dev.write(byte_msg)


    def read_serial(self, nBytes=1):
        # It is up to the caller to acquire / release mutex 

        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):
        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors on WinXP
            # (Conversations with Brian Wu @ MIT on 6/23/2010)
            self.servo_dev.close()
            self.servo_dev.parity = serial.PARITY_NONE
            self.servo_dev.stopbits = serial.STOPBITS_ONE
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException) as e:
            raise RuntimeError('lib_robotis: Serial port not found!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_robotis: Serial port not found!\n')

##################################################################################################

#Class for Model RX and MX Servos

##################################################################################################
class Robotis_Servo():
    ''' Class to use a robotis RX-28 or RX-64 servo.
    '''
    def __init__(self, USB2Dynamixel, servo_id, series = None ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
            series - Just a convenience for defining "good" defaults on MX series.
                     When set to "MX" it uses these values, otherwise it uses values
                     better for AX / RX series.  Any of the defaults can be overloaded
                     on a servo-by-servo bases in servo_config.py.
        '''

        self.series = series;	#record dynamixel series
        self.return_delay = 250 * 2e-6	#default return delay
        # To change the defaults, load some or all changes into servo_config.py
        if series == 'MX':			#MX series generally has 4x the travel and maximums of RX
            defaults = {
                'home_encoder': 0x7FF,
                'max_encoder': 0xFFF,
                'rad_per_enc': math.radians(360.0) / 0xFFF,
                'max_ang': math.radians(180),
                'min_ang': math.radians(-180),
                'flipped': False,
                'max_speed': math.radians(100)
                }
        else: # Common settings for RX-series.  Can overload in servo_config.py
            defaults = {
                'home_encoder': 0x200,
                'max_encoder': 0x3FF,  # Assumes 0 is min.
                'rad_per_enc': math.radians(300.0) / 1024.0,
                'max_ang': math.radians(148),
                'min_ang': math.radians(-148),
                'flipped': False,
                'max_speed': math.radians(100)
                }
        self.ADDR_DICT = registerDict.RXMX_Series

        # Error Checking
        if USB2Dynamixel == None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel
            self.dyn.servo_dev.flush()

        # ID exists on bus?
        self.servo_id = servo_id

        try:
            self.read_address(self.ADDR_DICT["ADDR_ID"])
        except Exception as inst:
            raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel 3-way switch in wrong position.\n' %
                               ( servo_id, self.dyn.dev_name ))

        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( self.ADDR_DICT["ADDR_RETURN_DELAY_TIME"], 1)
        self.return_delay = data[0] * 3e-4

        self.settings = {}
        #removed external servo settings file to simplify things (we usually only deal with MX or RX, and both settings are included above)
        # Set to default any parameter not specified in servo_config
        for key in defaults.keys():
            if key in self.settings: ##CHANGED self.settings.has_key( key ) THIS SHOULD ALSO WORK IN PYTHON2
                pass
            else:
                self.settings[ key ] = defaults[ key ]	#defaults dict moved into settings component


    def init_cont_turn(self):
        '''sets CCW angle limit to zero and allows continuous turning (good for wheels).
        After calling this method, simply use 'set_angvel' to command rotation.  This
        rotation is proportional to torque according to Robotis documentation.
        '''
        self.write_address(self.ADDR_DICT["ADDR_CCW_ANGLE_LIMIT_L"], [0,0])

    #UPDATED: set limits based on servo max encoder, reset speed to 0
        #should work properly for both RX/MX series
    def kill_cont_turn(self):
        '''resets CCW angle limits to allow commands through 'move_angle' again
        '''
        max_encoder = self.settings['max_encoder']
        hi,lo = int(max_encoder / 256), int(max_encoder % 256)	#addition made to reset encoder appropriately for both series
        self.write_address(self.ADDR_DICT["ADDR_CCW_ANGLE_LIMIT_L"], [lo,hi])

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_MOVING"], 1 )
        return data[0] != 0

    def read_voltage(self):
        ''' returns voltage (Volts)
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_VOLTAGE"], 1 )
        return data[0] / 10.

    def read_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_TEMPERATURE"], 1 )
        return data[0]

    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_LOAD_L"], 2 )
        load = data[0] + data[1] * 256
        if load>1024:
            return 1024-load
        else:
            return load

    #ADDED: current functionality only available for MX-64 series:
    def read_current(self):
        if self.series=='MX':
            data = self.read_address( self.ADDR_DICT["ADDR_CURRENT_L"], 2 )	#current spans addresses 0x44 and 0x45
            curr = data[0] + data[1] * 256
            return 4.5*(curr-2048)		#in mA
        else:
            return 0.

    #ADDED: speed address same for both MX/RX series
    def read_speed(self):
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_SPEED_L"], 2 )
        speed = data[0] + data[1] * 256
        if speed>1024:
            return float(1024-speed)/1024.0
        else:
            return float(speed)/1024.0

    #both moving speed in joint mode (between designated positions) as well as wheel mode (which only sets direction in operation)
    def apply_speed(self,amnt):
        amnt = max(0.,min(abs(amnt),1.0))
        speed_val = int(amnt*1023)
        if speed_val < 0:
            speed_val = speed_val+1024
        hi,lo = int(speed_val / 256), int(speed_val % 256)
        self.write_address(self.ADDR_DICT["ADDR_MOVING_SPEED_L"],[lo,hi])

    def read_encoder(self):
        ''' returns position in encoder ticks
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_POSITION_L"], 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_target_encoder(self):
        data = self.read_address( self.ADDR_DICT["ADDR_GOAL_POSITION_L"], 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_angle(self):	#based entirely off of the encoder setting
        ''' returns the current servo angle (radians)
        '''
        ang = (self.read_encoder() - self.settings['home_encoder']) * self.settings['rad_per_enc']
        if self.settings['flipped']:
            ang = ang * -1.0
        return ang

    #ADDED: reading/setting the max torque output (at different addresses)
    def read_max_torque(self):	#set twice, but we'll read the first instance
        data = self.read_address( self.ADDR_DICT["ADDR_MAX_TORQUE_L"], 2 )
        torque = data[0] + data[1] * 256
        return torque

    def apply_max_torque(self,val):	#unlike speed, 0 is not max torque; no mode dependencies to use/set
        amnt = max(0.,min(abs(val),1.0))
        n = int(amnt*1023)
        n = min(max(n,0), 1023)		#no scaling issues between MX and RX like with position
        hi,lo = int(n / 256), int(n % 256)
        self.write_address( 0x22, [lo,hi])
        return self.write_address( self.ADDR_DICT["ADDR_MAX_TORQUE_L"], [lo,hi])

    #alternative approach to move_to_encoder, except w/ arbitrary angle setting
    def move_angle(self, ang, angvel=None, blocking=True):
        ''' move to angle (radians)
        '''
        if angvel == None:
            angvel = self.settings['max_speed']

        if angvel > self.settings['max_speed']:
            print( 'lib_robotis.move_angle: angvel too high - %.2f deg/s' % (math.degrees(angvel)))
            print( 'lib_robotis.ignoring move command.')
            return

        if ang > self.settings['max_ang'] or ang < self.settings['min_ang']:
            print( 'lib_robotis.move_angle: angle out of range- ', math.degrees(ang))
            print( 'lib_robotis.ignoring move command.')
            return

        self.set_angvel(angvel)

        if self.settings['flipped']:
            ang = ang * -1.0
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        self.move_to_encoder( enc_tics )

        if blocking == True:
            while(self.is_moving()):
                continue

    def move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1
        n = min( max( n, 0 ), self.settings['max_encoder'] )
        hi,lo = int(n / 256), int(n % 256)
        return self.write_address( self.ADDR_DICT["ADDR_GOAL_POSITION_L"], [lo,hi] )

    #ADDED: reading the goal encoder position that the user has specified
        #useful for slow motions or torque-limited motions to find distance to goal
    def read_goal(self):
        data = self.read_address( self.ADDR_DICT["ADDR_GOAL_POSITION_L"], 2)
        enc_val = data[0] + data[1] * 256
        return enc_val

    #ADDED: enabling torque control mode for the MX-64 series and above ONLY:
    def enable_torque_mode(self):
        if self.series=='MX':
            return self.write_address(self.ADDR_DICT["ADDR_TORQUE_CONTROL_MODE_ENABLE"], [1])
        else:
            return 0
    def disable_torque_mode(self):
        if self.series=='MX':
            return self.write_address(self.ADDR_DICT["ADDR_TORQUE_CONTROL_MODE_ENABLE"], [0])
        else:
            return 0
    def apply_torque(self,amnt):
        if self.series=='MX':
            amnt = max(0.,min(abs(amnt),1.0))
            torque_val = int(amnt*1023)
            if torque_val < 0:
                torque_val = torque_val+1024
            hi,lo = int(torque_val / 256), int(torque_val % 256)
            return self.write_address(self.ADDR_DICT["ADDR_GOAL_TORQUE_L"],[lo,hi])
        else:
            return 0

    #disabling/enabling address 18 effectively shuts down motor output
        #different from torque mode that's available in the MX models
        #USE ONLY AS ON/OFF quick shutdown control
    def enable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [1])

    def disable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [0])

    #same as apply_speed, except set rad/sec as opposed to single value scalar
    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc<0:
            hi,lo = int(abs(angvel_enc) / 256 + 4), int(abs(angvel_enc) % 256)
        else:
            hi,lo = int(angvel_enc / 256),int( angvel_enc % 256)

        return self.write_address( self.ADDR_DICT["ADDR_MOVING_SPEED_L"], [lo,hi] )

    def write_id(self, id):
        ''' changes the servo id
        '''
        return self.write_address( self.ADDR_DICT["ADDR_ID"], [id] )

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def ping(self):
        return self.read_address(self,self.ADDR_DICT["ADDR_MODEL_NUMBER_L"],nBytes=1)

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, self.servo_id )


    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        time.sleep(self.return_delay)	#helps w/ communication consistency?

        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]
        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except Exception as inst:
            self.dyn.rel_mutex()
            raise RuntimeError(repr(str(inst)))
        self.dyn.rel_mutex()

        if err != 0:
            self.process_err( err )

        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )	#from pydynamixel: possible that these contain empty bytes
        servo_id = self.dyn.read_serial( 1 )

        while servo_id=='\xff':
            servo_id = self.dyn.read_serial( 1 )	#on Sparkfun USB-to-RS485 chip, more than 3 header bytes are sometimes set - apparently not an issue with the USB2Dynamixel

        if len(servo_id)!=1:
            raise RuntimeError('lib_robotis: Invalid message headers, got servo id of type: '+repr(type(servo_id))+' and length: '+repr(len(servo_id)))
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received')

        data_len = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )

        data=[]
        for i in range(ord(data_len)-2):
            input = self.dyn.read_serial( 1 )
            data.append(ord(input))

        checksum = self.dyn.read_serial( 1 ) 		#checksum is read but never compared...(by design, according to original lib_robotis.py)
        return data, ord(err)

    def send_serial(self, msg):
        """ sends the command to the servo
        """
        self.dyn.send_serial( msg )

##################################################################################################

# Class for Model XL Servos

##################################################################################################

class Robotis_Servo_XL():
    ''' Class to use a robotis XL-320 servo.
    '''
    def __init__(self, USB2Dynamixel, servo_id, series = 'XL' ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
            series - We have defined another set of "good" defaults for the X
                     series motors.
        '''

        self.series = series;   #record dynamixel series
        self.return_delay = 250 * 2e-6  #default return delay
        self.initialized = 0
        # To change the defaults, load some or all changes into servo_config.py
        try:
            if series == 'XL':
                defaults = {
                    'home_encoder': 0x64, # 100
                    'max_encoder': 0x3FF, # 1023
                    'rad_per_enc': math.radians(300.0) / 0x3FF, # 0 ~ 300deg valid
                    'max_ang': math.radians(180),
                    'min_ang': math.radians(-180),
                    'flipped': False,
                    'max_speed': math.radians(100)
                    }
            self.ADDR_DICT = registerDict.XL_Series

        except Exception as inst:
            print( "Could not intialize XL series series servo. Please make sure you have your series set to ''XL'' ")

        # Error Checking
        if USB2Dynamixel == None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel
            self.dyn.servo_dev.flush()

        # ID exists on bus?
        self.servo_id = servo_id

        try:
            self.read_address(self.ADDR_DICT["ADDR_ID"])
        except Exception as inst:
            raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel 3-way switch in wrong position.\n' %
                               ( servo_id, self.dyn.dev_name ))

        
        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( self.ADDR_DICT["ADDR_RETURN_DELAY_TIME"], 1)
        self.return_delay = data[0] * 3e-6

        self.settings = {}
        #removed external servo settings file to simplify things (we usually only deal with MX or RX, and both settings are included above)

        # Set to default any parameter not specified in servo_config
        for key in defaults.keys():
            if key in self.settings: ##CHANGED self.settings.has_key( key ) THIS SHOULD ALSO WORK IN PYTHON2
                pass
            else:
                self.settings[ key ] = defaults[ key ]  #defaults dict moved into settings component

        #We will do this initially so that we will enable torque
        #self.enable_position_mode()
        #self.in_extended_position_control_mode = False

    def read_encoder(self):
        ''' returns position in encoder ticks
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_POSITION"], 2 )
        enc_val = data[0] + data[1] * 256 # + data[2]*256*256 + data[3]*256*256*256
        return float(twos_comp_backward(enc_val,16))

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [self.servo_id, 7, 0, 0x02, DXL_LOBYTE(address), DXL_HIBYTE(address), DXL_LOBYTE(nBytes), DXL_HIBYTE(nBytes)]  #0x02 is the read request command
        
        return self.send_instruction( msg, self.servo_id )        

    def send_instruction(self, instruction, id):
        time.sleep(self.return_delay)   #helps w/ communication consistency?

        #Find total packet length first
        total_packet_length = registerDict.DXL_MAKEWORD(instruction[1], instruction[2]) + 7
        msg = [ 0xFF, 0xFF, 0xFD, 0x00 ] + instruction

        # add CRC16
        crc = registerDict.updateCRC(0, msg, total_packet_length - 2)  # 2: CRC16
        msg = msg + [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

        self.dyn.acq_mutex()

        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except Exception as inst:
            self.dyn.rel_mutex()
            raise RuntimeError(repr(str(inst)))
        self.dyn.rel_mutex()

        if err != 0 and err !=128: #the 128 error is common
            self.process_err( err )
        return data

    def send_serial(self, msg):
        """ sends the command to the servo
        """
        self.dyn.send_serial( msg )        

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )   #from pydynamixel: possible that these contain empty bytes

        servo_id = self.dyn.read_serial( 1 )

        while str(servo_id)=='\xfd' or str(servo_id)=='\xff' or str(servo_id)=='\x00' or str(servo_id)=="b'\\xfd'" or str(servo_id)=="b'\\xff'" or str(servo_id)=="b'\\x00'":
            servo_id = self.dyn.read_serial( 1 )    #on Sparkfun USB-to-RS485 chip, more than 3 header bytes are sometimes set - apparently not an issue with the USB2Dynamixel 

        if len(servo_id)!=1:
            raise RuntimeError('lib_robotis: Invalid message headers, got servo id of type: '+repr(type(servo_id))+' and length: '+repr(len(servo_id)))
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received')
        message_length_low = self.dyn.read_serial( 1 )
        message_length_high = self.dyn.read_serial( 1 )
        INST = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data=[]
        for i in range(ord(message_length_low)-4):
            input = self.dyn.read_serial( 1 )
            data.append(ord(input))

        crc16_low = self.dyn.read_serial( 1 )
        crc16_high = self.dyn.read_serial( 1 )

        return data, ord(err)

    #This can be reimplemented if we desire to go past one full rotation, we will have to use the twos comp tricks
    def move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1
        n = twos_comp_forward(int(n),16)
        hi,lo = int((n >>8) & 0xff), int(n & 0xff)
        return self.write_address( self.ADDR_DICT["ADDR_GOAL_POSITION"], [lo,hi] )
        ## return self.write_address( self.ADDR_DICT["ADDR_GOAL_POSITION"], [lo,hi,0,0] )        ### Adding the 0,0 at the end is not required and also doesn't allow moving speed to change

    def write_speed(self, n, dir=1):        # direction only used in Wheel mode (1: CW, -1: CCW)
        n = max(0.,min(abs(n),1.0))
        speed_val = int(n*1023)
        if dir < 0:
            speed_val = speed_val+1024
        hi,lo = int(speed_val / 256), int(speed_val % 256)
        self.write_address(self.ADDR_DICT["ADDR_MOVING_SPEED"],[lo,hi])

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg_len=len(data)
        msg = [self.servo_id, DXL_LOBYTE(msg_len + 5), DXL_HIBYTE(msg_len + 5), 0x03, DXL_LOBYTE(address), DXL_HIBYTE(address)] + data #0x03 is the write command
        return self.send_instruction( msg, self.servo_id )

    def read_id(self):
        ''' reads the servo id
        '''
        #Must disable torque to read id properly
        id =self.read_address( self.ADDR_DICT["ADDR_ID"], 1 ) #request just 1 byte
        return

    def write_id(self, id):
        ''' changes the servo id
        '''
        #Must disable torque to write id (at all)
        #Note, you should only do this individually
        #print( self.disable_torque())
        print( self.write_address( self.ADDR_DICT["ADDR_ID"], [id] ))
        return

    def write_LED(self, color):
        ''' changes the LED color
        0: off
        1: red
        2: green
        4: blue
        3: yellow
        6: cyan
        5: purple
        7: white
        '''
        #Must disable torque to write id (at all)
        #Note, you should only do this individually
        print( self.write_address( self.ADDR_DICT["ADDR_LED"], [color] ))
        return

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def apply_max_torque(self,val): #unlike speed, 0 is not max torque; no mode dependencies to use/set
        amnt = max(0.,min(abs(val),1.0))
        n = int(amnt*648)
        n = min(max(n,0), 648)      #no scaling issues between MX and RX like with position
        hi,lo = int(n / 256), int(n % 256)
        #Disble the torque first in order to change
        self.disable_torque()
        self.write_address( self.ADDR_DICT["ADDR_TORQUE_LIMIT"], [lo,hi])
        return self.enable_torque()

    def enable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [1])

    def disable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [0])

    def enable_wheel(self):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_CONTROL_MODE"], [1])
        self.enable_torque()

    def enable_joint(self):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_CONTROL_MODE"], [2])
        self.enable_torque()

    def read_control_mode(self):
        data = self.read_address( self.ADDR_DICT["ADDR_CONTROL_MODE"], 1 )
        return data      

    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_LOAD"], 2 )
        load = data[0] + data[1] * 256
        return float(twos_comp_backward(load,16))

##################################################################################################

# Class for Model X Servos

##################################################################################################

class Robotis_Servo_X():
    ''' Class to use a robotis XH servo.
    '''
    def __init__(self, USB2Dynamixel, servo_id, series = 'X' ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
            series - We have defined another set of "good" defaults for the X
                     series motors.
        '''

        self.series = series;	#record dynamixel series
        self.return_delay = 250 * 2e-6	#default return delay
        self.initialized = 0
        # To change the defaults, load some or all changes into servo_config.py
        try:
            if series == 'X' or series == 'XM' or series == 'XR':
                defaults = {
                    'home_encoder': 0x7FF,
                    'max_encoder': 0xFFF,
                    'rad_per_enc': math.radians(360.0) / 0xFFF,
                    'max_ang': math.radians(180),
                    'min_ang': math.radians(-180),
                    'flipped': False,
                    'max_speed': math.radians(100)
                    }
            self.ADDR_DICT = registerDict.X_Series

        except Exception as inst:
            print( "Could not intialize X series series servo. Please make sure you have your series set to ''X'' ")

        # Error Checking
        if USB2Dynamixel == None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel
            self.dyn.servo_dev.flush()

        # ID exists on bus?
        self.servo_id = servo_id

        try:
            self.read_address(self.ADDR_DICT["ADDR_ID"])
        except Exception as inst:
            raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel 3-way switch in wrong position.\n' %
                               ( servo_id, self.dyn.dev_name ))

        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( self.ADDR_DICT["ADDR_RETURN_DELAY_TIME"], 1)
        self.return_delay = data[0] * 3e-6

        self.settings = {}
        #removed external servo settings file to simplify things (we usually only deal with MX or RX, and both settings are included above)

        # Set to default any parameter not specified in servo_config
        for key in defaults.keys():
            if key in self.settings: ##CHANGED self.settings.has_key( key ) THIS SHOULD ALSO WORK IN PYTHON2
                pass
            else:
                self.settings[ key ] = defaults[ key ]	#defaults dict moved into settings component


        #We will do this initially so that we will enable torque
        self.enable_position_mode()
        self.in_extended_position_control_mode = False

    #Move motor then with .set_angvel
    def init_cont_turn(self):
        '''Set the motor into velocity control mode'''
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [self.ADDR_DICT["VELOCITY_CONTROL_MODE"]]) #The value for 1 sets the it into vel control
        self.enable_torque()

    def kill_cont_turn(self):
        '''resets CCW angle limits to allow commands through 'move_angle' again
        '''
        self.enable_position_mode()

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_MOVING"], 1 )
        return data[0] != 0

    def read_voltage(self):
        ''' returns voltage (Volts)
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_INPUT_VOLTAGE"], 2 )
        return data[0] / 10.

    def read_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_TEMPERATURE"], 1 )
        return data[0]

    #This was replaced with present_current in the x series since load is not available
    #Note that this is an arbritary load. Weopt.baud cannot infer a total force on the object unless testing for stall torque
    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_CURRENT"], 2 )
        load = data[0] + data[1] * 256
        return float(twos_comp_backward(load,16))


    #ADDED: Works for the x series, but does not have a load
    def read_current(self):
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_CURRENT"], 2 )	#current spans addresses 0x44 and 0x45
        curr = data[0] + data[1] * 256
        return 2.69 * float(twos_comp_backward(curr,16))		#in mA


    #Velocity in RPMs
    def read_speed(self):
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_VELOCITY"], 4 )
        speed = data[0] + data[1] * 256 + data[2]*256*256 + data[3]*256*256*256
        return 0.229 * float(twos_comp_backward(speed,32))		#in RPM

    #both moving speed in joint mode (between designated positions) as well as wheel mode (which only sets direction in operation)
    def apply_speed(self,amnt):
        amnt = max(-1.,min(amnt,1.0))
        apply_speed = int(amnt*1023.) #note that 648 is the upper limit on the what we can set
        apply_speed = twos_comp_forward(apply_speed,32)
        hhi,mhi,mlo,lo = (apply_speed >>24) & 0xff, (apply_speed >>16) & 0xff, (apply_speed >>8) & 0xff, apply_speed & 0xff
        self.write_address(self.ADDR_DICT["ADDR_GOAL_VELOCITY"],[lo,mlo,mhi,hhi])

    def read_encoder(self):
        ''' returns position in encoder ticks
        '''
        data = self.read_address( self.ADDR_DICT["ADDR_PRESENT_POSITION"], 4 )
        enc_val = data[0] + data[1] * 256 + data[2]*256*256 + data[3]*256*256*256
        return float(twos_comp_backward(enc_val,32))

    def read_target_encoder(self):
        data = self.read_address(  self.ADDR_DICT["ADDR_GOAL_POSITION"],4 )
        enc_val = data[0] + data[1] * 256 + data[2]*256*256 + data[3]*256*256*256
        return float(twos_comp_backward(enc_val,32))

    def read_angle(self):	#based entirely off of the encoder setting
        ''' returns the current servo angle (radians)
        '''
        ang = (self.read_encoder() - self.settings['home_encoder']) * self.settings['rad_per_enc']
        if self.settings['flipped']:
            ang = ang * -1.0
        return ang

    #ADDED: reading/setting the max torque output (at different addresses)
    #This is reading the current limit as the max_torque
    #This comes in with a range of 0-648 and has a unit of ~2.69mA
    def read_max_torque(self):	#set twice, but we'll read the first instance
        data = self.read_address( self.ADDR_DICT["ADDR_CURRENT_LIMIT"], 2 )
        torque = data[0] + data[1] * 256
        return torque

    def apply_max_torque(self,val):	#unlike speed, 0 is not max torque; no mode dependencies to use/set
        amnt = max(0.,min(abs(val),1.0))
        n = int(amnt*648)
        n = min(max(n,0), 648)		#no scaling issues between MX and RX like with position
        hi,lo = int(n / 256), int(n % 256)
        #Disble the torque first in order to change
        self.disable_torque()
        self.write_address( self.ADDR_DICT["ADDR_CURRENT_LIMIT"], [lo,hi])
        return self.enable_torque()

    #alternative approach to move_to_encoder, except w/ arbitrary angle setting
    def move_angle(self, ang, angvel=None, blocking=True):
        ''' move to angle (radians)
        '''
        if angvel == None:
            angvel = self.settings['max_speed']

        if angvel > self.settings['max_speed']:
            print( 'lib_robotis.move_angle: angvel too high - %.2f deg/s' % (math.degrees(angvel)))
            print( 'lib_robotis.ignoring move command.')
            return

        if ang > self.settings['max_ang'] or ang < self.settings['min_ang']:
            print( 'lib_robotis.move_angle: angle out of range- ', math.degrees(ang))
            print( 'lib_robotis.ignoring move command.')
            return

        self.set_angvel(angvel)

        if self.settings['flipped']:
            ang = ang * -1.0
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        self.move_to_encoder( enc_tics )

        if blocking == True:
            while(self.is_moving()):
                continue

    #This can be reimplemented if we desire to go past one full rotation, we will have to use the twos comp tricks
    def move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1
        if self.in_extended_position_control_mode == False:
            n = min( max( n, 0 ), self.settings['max_encoder'] )
            hi,lo = int(n / 256), int(n % 256)
        else:
            n = twos_comp_forward(n,16)
            hi,lo = int((n >>8) & 0xff), int(n & 0xff)
        return self.write_address( self.ADDR_DICT["ADDR_GOAL_POSITION"], [lo,hi,0,0] )

    #ADDED: reading the goal encoder position that the user has specified
        #useful for slow motions or torque-limited motions to find distance to goal
    def read_goal(self):
        data = self.read_address( self.ADDR_DICT["ADDR_GOAL_POSITION"], 4)
        enc_val = data[0] + data[1] * 256 + data[2]*256*256 + data[3]*256*256*256
        return float(twos_comp_backward(enc_val,32))

    #ADDED: Enables torque control mode for x-series:
    def enable_torque_mode(self):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [self.ADDR_DICT["CURRENT_CONTROL_MODE"]])
        return self.enable_torque()

    def disable_torque_mode(self): #returns back to position control
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"],[self.ADDR_DICT["POSITION_CONTROL_MODE"]] )
        return self.enable_torque()

    #This makes more sense as a naming comvention, but will leave the original for backwars compatability
    def enable_position_mode(self): #returns back to position control
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [self.ADDR_DICT["POSITION_CONTROL_MODE"]])
        return self.enable_torque()

    def enable_extended_position_control_mode(self):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [4])
        self.in_extended_position_control_mode = True
        return self.enable_torque()

    def disable_extended_position_control_mode(self):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [self.ADDR_DICT["POSITION_CONTROL_MODE"]])
        self.in_extended_position_control_mode = False
        return self.enable_torque()

    def enable_current_position_control_mode(self, torque_val = 0.1):
        self.disable_torque()
        self.write_address(self.ADDR_DICT["ADDR_OPERATING_MODE"], [5])
        self.apply_max_torque(torque_val)
        return self.enable_torque()

    #Functionality for negative torque is now different and uses 2's complement
    #for storing register values. This is the same for velocity
    def apply_torque(self,amnt=0.1):
        amnt = max(-1.,min(amnt,1.0))
        torque_val = int(amnt*648.) #note that 648 is the upper limit on the what we can set
        torque_val = twos_comp_forward(torque_val,16)
        hi,lo = int((torque_val >>8) & 0xff), int(torque_val & 0xff)
        return self.write_address(self.ADDR_DICT["ADDR_GOAL_CURRENT"],[lo,hi])

    #disabling/enabling address 18 effectively shuts down motor output
        #different from torque mode that's available in the MX models
        #USE ONLY AS ON/OFF quick shutdown control
    def enable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [self.ADDR_DICT["TORQUE_ENABLE"]])

    def disable_torque(self):
        return self.write_address(self.ADDR_DICT["ADDR_TORQUE_ENABLE"], [self.ADDR_DICT["TORQUE_DISABLE"]])

    #same as apply_speed, except set rad/sec as opposed to single value scalar
    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''
        rpm = angvel / (2 * math.pi) * 60.0
        apply_speed = int(round( rpm / 0.229 ))
        apply_speed = twos_comp_forward(apply_speed,32)
        hhi,mhi,mlo,lo = (apply_speed >>24) & 0xff, (apply_speed >>16) & 0xff, (apply_speed >>8) & 0xff, apply_speed & 0xff
        self.write_address(self.ADDR_DICT["ADDR_GOAL_VELOCITY"],[lo,mlo,mhi,hhi])
        return

    def set_velprofile(self, amnt):         # ADDED 09/22/21: setting velocity limit in joint modes
        ''' amnt - range [0,1], note: 0 resets to max speed
        '''
        amnt = abs(amnt)
        amnt = min(amnt,1.0)
        amnt = int(round( amnt * 0.229 * 1023))
        amnt = twos_comp_forward(amnt,32)
        hhi,mhi,mlo,lo = (amnt >>24) & 0xff, (amnt >>16) & 0xff, (amnt >>8) & 0xff, amnt & 0xff
        self.write_address(self.ADDR_DICT["ADDR_PROFILE_VELOCITY"],[lo,mlo,mhi,hhi])
        return

    def read_velprofile(self):              # ADDED 09/22/21: read velocity limit in joint modes
        data = self.read_address( self.ADDR_DICT["ADDR_PROFILE_VELOCITY"], 4)
        velprof_val = data[0] + data[1] * 256 + data[2]*256*256 + data[3]*256*256*256
        return float(twos_comp_backward(velprof_val,32))

    def write_id(self, id):
        ''' changes the servo id
        '''
        #Must disable torque to write id (at all)
        #Note, you should only do this individually
        print( self.disable_torque())
        print( self.write_address( self.ADDR_DICT["ADDR_ID"], [id] ))
        return

    def read_id(self):
        ''' reads the servo id
        '''
        #Must disable torque to read id properly
        self.disable_torque()
        id =self.read_address( self.ADDR_DICT["ADDR_ID"], 1 ) #request just 1 byte
        self.enable_torque()
        return

    def ping(self):
        msg = [self.servo_id, 3, 0, 0x1]
        resp = self.send_instruction(msg,self.servo_id)
        if len(resp) >0:
            print( "PING to actuator successful!")
        else:
            print( "PING to actuator failed")

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [self.servo_id, 7, 0, 0x02, DXL_LOBYTE(address), DXL_HIBYTE(address),DXL_LOBYTE(nBytes), DXL_HIBYTE(nBytes)]  #0x02 is the read request command
        return self.send_instruction( msg, self.servo_id )

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg_len=len(data)
        msg = [self.servo_id, DXL_LOBYTE(msg_len + 5), DXL_HIBYTE(msg_len + 5), 0x03, DXL_LOBYTE(address), DXL_HIBYTE(address)] + data #0x03 is the write command
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        time.sleep(self.return_delay)	#helps w/ communication consistency?

        #Find total packet length first
        total_packet_length = registerDict.DXL_MAKEWORD(instruction[1], instruction[2]) + 7
        msg = [ 0xFF, 0xFF, 0xFD, 0x00 ] + instruction

        # add CRC16
        crc = registerDict.updateCRC(0, msg, total_packet_length - 2)  # 2: CRC16
        msg = msg + [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except Exception as inst:
            self.dyn.rel_mutex()
            raise RuntimeError(repr(str(inst)))
        self.dyn.rel_mutex()

        if err != 0:
            self.process_err( err )
        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )	#from pydynamixel: possible that these contain empty bytes

        servo_id = self.dyn.read_serial( 1 )

        while str(servo_id)=='\xfd' or str(servo_id)=='\xff' or str(servo_id)=='\x00' or str(servo_id)=="b'\\xfd'" or str(servo_id)=="b'\\xff'" or str(servo_id)=="b'\\x00'":
            servo_id = self.dyn.read_serial( 1 )	#on Sparkfun USB-to-RS485 chip, more than 3 header bytes are sometimes set - apparently not an issue with the USB2Dynamixel

        if len(servo_id)!=1:
            raise RuntimeError('lib_robotis: Invalid message headers, got servo id of type: '+repr(type(servo_id))+' and length: '+repr(len(servo_id)))
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received')
        message_length_low = self.dyn.read_serial( 1 )
        message_length_high = self.dyn.read_serial( 1 )
        INST = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data=[]
        for i in range(ord(message_length_low)-4):
            input = self.dyn.read_serial( 1 )
            data.append(ord(input))

        crc16_low = self.dyn.read_serial( 1 )
        crc16_high = self.dyn.read_serial( 1 )

        return data, ord(err)


    def send_serial(self, msg):
        """ sends the command to the servo
        """
        self.dyn.send_serial( msg )



##################################################################################################

#Methods used as options on startup

##################################################################################################

#print(s out servo IDs connected to the U2D2 Device. Looks for both protocols
def find_servos(dyn, search_range = 255):
    ''' Finds all servo IDs on the USB2Dynamixel '''
    print( 'Scanning for Servos.')
    servos = []
    dyn.servo_dev.timeout = 0.03  # To make the scan faster
    for i in range(search_range): #default 249
        try:
            s = Robotis_Servo( dyn, i )
            print( '\n FOUND A MX/RX SERVO @ ID %d\n' % i)
            servos.append( i )
        except:
            try:
                s = Robotis_Servo_X( dyn, i )
                print( '\n FOUND AN X_SERVO @ ID %d\n' % i)
                servos.append( i )
            except:
                try:
                    s = Robotis_Servo_XL( dyn, i )
                    print( '\n FOUND AN XL_SERVO @ ID %d\n' % i)
                    servos.append( i )    
                except:
                    pass
    dyn.servo_dev.timeout =  1.0  # Restore to original
    return servos

#Changes ID of the specified motor in the daisy chain. Motor will need
#to be reset after ID change
def change_servo_id(dyn, prev_id, new_id):
    print( 'Changing servo id...')
    try:
        s=Robotis_Servo(dyn, prev_id) #ensure we are changing the right one
        notlowByte = ~((254+4+3+3+new_id)%256)
        vals = [255, 255, 254, 4, 3, 3, new_id, notlowByte%256]
        vals_char= [chr(i) for i in vals]
        for i in vals_char:
        	dyn.send_serial(i)
        s_new=Robotis_Servo(dyn, new_id)
        print( 'Servo ID for RX/MX changed successfully')
    except:
        try:
            s=Robotis_Servo_X(dyn, prev_id) #ensure we are changing the right one
            s.write_id(new_id)
            s_new = s=Robotis_Servo_X(dyn, new_id)
            print( 'Servo ID for X series changed successfully')
        except:
            try:
                s=Robotis_Servo_XL(dyn, prev_id) #added this new try block for XL
                s.write_id(new_id)
                s_new = s=Robotis_Servo_XL(dyn, new_id)
                print( 'Servo ID for XL series changed successfully')
            except:
                print( 'Servo ID change failed. Please check to ensure --setID <prev_id> <new_id>')
                pass
    return

#Moves the motors to a random encoder position, specified by a string of
#actuator ids, e.g. --moveTest "1 4 5 6"
def move_servos_test(dyn, ids, encoder_pos = None): #ids is a list of integers
    ''' Move the servos specified in the function call '''
    print( 'Moving servos to a different (random) position...')
    servos = []
    dyn.servo_dev.timeout = 0.03  # To make the scan faster
    if encoder_pos == None:
        encoder_pos = random.randint(50,1000)

    for i in ids:
        try:
            s = Robotis_Servo( dyn, i )
            s.move_to_encoder(encoder_pos)
            print( '\n MOVED MOTOR TO ENCODER POSITION ' ,encoder_pos, '@ ID ',i)
            servos.append( i )
            time.sleep(0.03)
        except:
            try:
                s = Robotis_Servo_X( dyn, i )
                s.move_to_encoder(encoder_pos)
                print( '\n MOVED MOTOR TO ENCODER POSITION ' ,encoder_pos, '@ ID ',i)
                servos.append( i )
                time.sleep(0.03)
            except:
                print( 'DID NOT CONNECT AND MOVE MOTOR TO ENCODER POSITION',encoder_pos,' @ ID ',i)
    dyn.servo_dev.timeout =  1.0  # Restore to original
    return servos

##Function used for recovering a bricked protocol 2.0 servo
#This will reset all in the chain, so make sure only one is connected
def recover_protocol2_servo(dyn):
    instruction = [254, 4, 0, 6, 255] #http://emanual.robotis.com/docs/en/dxl/protocol2/
    total_packet_length = registerDict.DXL_MAKEWORD(instruction[1], instruction[2]) + 7
    msg = [ 0xFF, 0xFF, 0xFD, 0x00 ] + instruction
    # add CRC16
    crc = registerDict.updateCRC( 0, msg, total_packet_length - 2)  # 2: CRC16
    msg = msg + [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]
    out = ''
    for m in msg:
        out += chr(m)
    dyn.send_serial( out )

##Function used for recovering a bricked protocol 1.0 servo
#This will reset all in the chain, so make sure only one is connected
def recover_protocol1_servo(dyn):
    msg = [ 254, 2, 6] # instruction includes the command (1 byte + parameters. length = parameters+2) http://emanual.robotis.com/docs/en/dxl/protocol1/
    chksum = 0
    for m in msg:
        chksum += m
    chksum = ( ~chksum ) % 256
    msg = [ 0xff, 0xff ] + msg + [chksum]
    out = ''
    for m in msg:
        out += chr(m)
    dyn.send_serial( out )
    time.sleep(0.003)

#This works for sure on the protocol 2 models
def recover_servo(dyn):
    ''' Recovers a bricked servo by sending out a global reset to all servos in the chain '''
    raw_input('Make sure only one servo connected to USB2Dynamixel Device [ENTER]')
    raw_input('Connect power to the servo. [ENTER]')
    input = raw_input('Type in the protocol version, either a 1 or a 2 (RX,MX = 1, XM =2) [ENTER]:  ')
    if input == '1':
        recover_protocol1_servo(dyn);
        print( ('... completed for protocol 1'))
    elif input == '2':
        recover_protocol2_servo(dyn);
        time.sleep(0.003)
        print( ('... completed for protocol 2'))
    else:
        print( '[ERR] You did not input a 1 or a 2')



def hard_recover_servo(dev_name):
    ''' Hard recovery of a bricked servo by sending out a global reset to all servos in the chain at all baud rates '''
    raw_input('Make sure only one servo connected to USB2Dynamixel Device [ENTER]')
    raw_input('Connect power to the servo. [ENTER]')
    print( ('This may take a while...'))

    bauds = np.arange(8000, 1000000, 200)
    for i in range(len(list(bauds))):
        print ("Percentage Complete: ", float(i)/float(len(list(bauds))))
        dyn = USB2Dynamixel_Device(dev_name, list(bauds)[i])
        recover_protocol1_servo(dyn);
        time.sleep(0.05)
        recover_protocol2_servo(dyn);
        time.sleep(0.05)
        del dyn



if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='dev_name',default='/dev/ttyUSB0',
                 help='Required: Device string for USB2Dynamixel. [i.e. /dev/ttyUSB0 for Linux, \'0\' (for COM1) on Windows]')
    p.add_option('--scan', action='store_true', dest='scan', default=False,
                 help='Scan the device for servo IDs attached.')
    p.add_option('--recover', action='store_true', dest='recover', default=False,
                 help='Recover from a bricked servo (restores to factory defaults).')
    p.add_option('--hardRecover', action='store_true', dest='hardRecover', default=False,
                 help='Hard recover from a bricked servo (restores to factory defaults - hopefully).')
    p.add_option('--ang', action='store', type='float', dest='ang',
                 help='Angle to move the servo to (degrees).')
    p.add_option('--ang_vel', action='store', type='float', dest='ang_vel',
                 help='angular velocity. (degrees/sec) [default = 50]', default=50)
    p.add_option('--id', action='store', type='int', dest='id',
                 help='id of servo to connect to, [default = 1]', default=1)
    p.add_option('--baud', action='store', type='int', dest='baud',
                 help='baudrate for USB2Dynamixel connection [default = 57600]', default=57600)
    p.add_option('--setID', action='store', type='int', dest='ids', nargs =2,
                help='changing servo ids - usage "--setID <prev_id>, <new_id>" ')
    p.add_option('--moveTest', action='store', type='string', dest='mot_ids', nargs = 1,
                help='moves all motors to a random location to ensure each are connected and working "--moveTest ''id1, id2, id3, ....'' ')


    opt, args = p.parse_args()

    if opt.dev_name == None:
        p.print(_help())
        sys.exit(0)

    dyn = USB2Dynamixel_Device(opt.dev_name, opt.baud)

    if opt.scan:
        find_servos( dyn )
    if opt.recover:
        recover_servo( dyn )
    if opt.hardRecover:
        del dyn
        hard_recover_servo(opt.dev_name)
    if opt.ids:
        change_servo_id(dyn, opt.ids[0],opt.ids[1])
    if opt.mot_ids:
        des_ids = map(int, opt.mot_ids.split(' '))
        move_servos_test(dyn, des_ids) #python3 list(map(int, results))
    if opt.ang != None:
        servo = Robotis_Servo( dyn, opt.id )
        servo.move_angle( math.radians(opt.ang), math.radians(opt.ang_vel) )

##################################################################################################

#Helper functions used for X-series

##################################################################################################

def twos_comp_forward(val, bits): #takes pythons version of signed numbers and converts to twos comp
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def twos_comp_backward(number, bitLength): #returns the signed number
    mask = (2 ** bitLength) - 1
    if number & (1 << (bitLength - 1)):
        return number | ~mask
    else:
        return number & mask

