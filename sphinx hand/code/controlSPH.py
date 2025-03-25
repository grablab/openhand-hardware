
## Control Code for Sphinx Hand

from time import sleep
import lib_robotis_mod as lib

import numpy as np
from random import randint
import math

import sympy as sym

import matlab.engine
eng = matlab.engine.start_matlab()


class SPH:
    def __init__(self):
        # Design Parameters
        self.CC = np.zeros((1,3))               # Fixed common center [mm]
        self.L1 = 70                            # Link 1 length [mm]
        self.L2 = 70                            # Link 2 length [mm]
        self.R1ang = 55*np.pi/180               # Angle of R1 axis from vertical [rad]          
        self.Rsph = 100                         # Spherical motion radius [mm]

        self.L_R2 = np.sqrt(self.Rsph**2 - self.L2**2)    # Distance from center to R2 joint [mm]
        self.L_R1 = np.sqrt(self.L_R2**2 - self.L1**2)    # Distance from center to R1 joint [mm]
        
        self.beta = None                    # active R2 joint position [rad, 3x1]
        self.alpha = None                   # active R1 joint position [rad, 3x1]

        self.initial_alpha = [90*np.pi/180, 90*np.pi/180, 90*np.pi/180]
        self.initial_beta = [-110*np.pi/180, -110*np.pi/180, -110*np.pi/180]

        self.baseAngles = [90*np.pi/180, 210*np.pi/180, 330*np.pi/180]

        self.R = None                               # Object rotation matrix
        self.PP = np.zeros((3,3))                   # R1 joint locations
        self.NN = np.zeros((3,3))                   # Contact locations
        self.NN_grasp = np.zeros((3,3))             # Contact locations at grasp

        self.motTOang = [0, 0, 0, 0, 0, 0]
        self.motTOang[0] = 98*np.pi/180             # Offset from X motor to alpha angle [rad] (keep motors in range +/-180)
        self.motTOang[1] = 78*np.pi/180
        self.motTOang[2] = 70*np.pi/180
        self.motTOang[3] = 0*-180*np.pi/180            # Offset from XL motor to beta angle [rad] (keep motors in range +/-150)
        self.motTOang[4] = 0*-180*np.pi/180
        self.motTOang[5] = 0*-180*np.pi/180

        dyn1 = lib.USB2Dynamixel_Device('COM7')      # '/dev/ttyUSB0'
        dyn2 = lib.USB2Dynamixel_Device('COM8')

        self.XM1 = lib.Robotis_Servo_X(dyn1, 1)     # Initializing XM motors for alpha joints
        self.XM2 = lib.Robotis_Servo_X(dyn1, 2)
        self.XM3 = lib.Robotis_Servo_X(dyn1, 3)

        self.XL1 = lib.Robotis_Servo_X(dyn2, 4)    # Initializing XL-430 motors for beta joints
        self.XL2 = lib.Robotis_Servo_X(dyn2, 5)
        self.XL3 = lib.Robotis_Servo_X(dyn2, 6)

        print('SPH object created.')
        

    # FUNCTION DEFINITIONS

    def initHand(self):
        for f in range(3):
            self.PP[:,f] = np.transpose([self.L_R1*np.sin(self.R1ang)*np.cos(self.baseAngles[f]), 
                                    self.L_R1*np.sin(self.R1ang)*np.sin(self.baseAngles[f]),
                                     -self.L_R1*np.cos(self.R1ang)])

        self.homeHand()
        print('Hand initiated.')


    def homeHand(self):
        alpha_home = self.initial_alpha
        beta_home = self.initial_beta

        self.moveMotors(alpha_home, beta_home, 2.5)

    def openHand(self):                         # different from homeHand() to be able to control speed_a and speed_b
        vals = np.zeros(6)
        for i in range(3):
            vals[i] = (- self.initial_alpha[i] + self.motTOang[i])
            vals[i+3] = (- self.initial_beta[i] + self.motTOang[i+3])

        speed_b = 0.3                       # speed of beta motors: range [0,1]
        self.XL1.set_velprofile(speed_b)
        self.XL2.set_velprofile(speed_b)
        self.XL3.set_velprofile(speed_b)
        
        self.XL1.move_angle(vals[3], blocking=False)
        self.XL2.move_angle(vals[4], blocking=False)
        self.XL3.move_angle(vals[5], blocking=False)

        speed_a = 0.04
        self.XM1.set_velprofile(speed_a)
        self.XM2.set_velprofile(speed_a)
        self.XM3.set_velprofile(speed_a)

        self.XM1.move_angle(vals[0], blocking=False)
        self.XM2.move_angle(vals[1], blocking=False)
        self.XM3.move_angle(vals[2], blocking=False)

        

    def graspHand(self):

        graspTorque = [0.3, 1]      # Range:0-1.0, Increment by max 0.05, Ok up to 0.2-0.3 for XM
        maxCloseAng = [0, 0, 0, 0, 0, 0]                            # Maximum angle motors can close
        alpha0 = [28.1, 34.1,   38.4,   41.5,   63.1,   66.6,   81.5,   87.3]    # for R0.2 to R0.9 objects
        beta0 = [-38.1,-49.8,  -60.3,  -70.5,  -72.5,  -84.3,  -85.7,  -79.5]    # for R0.2 to R0.9 objects
        
        squeezeR = 0.7      # To change approx how hard to squeeze object (radius of "squeezed" object)
        
        squeezeR = int((squeezeR - 0.19)*10)     # convert to index of alpha0 and beta0
        print(squeezeR)
        for i in range(3):
            maxCloseAng[i] = - alpha0[squeezeR]*np.pi/180 + self.motTOang[i]
            maxCloseAng[i+3] = - (beta0[squeezeR]*np.pi/180) + self.motTOang[i+3]

        # setting XM torque limit for grasping, then commanding motors to end of range
        self.XM1.enable_current_position_control_mode(torque_val = graspTorque[0])
        self.XM2.enable_current_position_control_mode(torque_val = graspTorque[0])
        self.XM3.enable_current_position_control_mode(torque_val = graspTorque[0])

        # setting max velocity XM motors can reach
        self.XM1.set_velprofile(0.1)
        self.XM2.set_velprofile(0.1)
        self.XM3.set_velprofile(0.1)
        self.XM1.move_angle(maxCloseAng[0], blocking=False)
        self.XM2.move_angle(maxCloseAng[1], blocking=False)
        self.XM3.move_angle(maxCloseAng[2], blocking=False)

        
        self.XL1.set_velprofile(0.2)
        self.XL2.set_velprofile(0.2)
        self.XL3.set_velprofile(0.2)
        
        
        # Move XL motors only while XM is still moving
        while ((self.XM1.is_moving() or self.XM2.is_moving()) or self.XM3.is_moving()):
            if self.XM1.is_moving():
                self.XL1.move_angle(maxCloseAng[3], blocking=False)
            else:
                self.XL1.move_to_encoder(self.XL1.read_encoder())
            if self.XM2.is_moving():
                self.XL2.move_angle(maxCloseAng[4], blocking=False)
            else:
                self.XL2.move_to_encoder(self.XL2.read_encoder())
            if self.XM3.is_moving():
                self.XL3.move_angle(maxCloseAng[5], blocking=False)
            else:
                self.XL3.move_to_encoder(self.XL3.read_encoder())
            # print("grasping")


        self.R = np.identity(3)
        self.NN_grasp = self.calcNN()

        return True

    def calcNN(self):
        # Calculate the contact locations from alpha and beta
        self.alpha = self.readEnc()[0:3]
        self.beta = self.readEnc()[3:6]
        
        RR = np.zeros((3,3))
        NNcalc = np.zeros((3,3))
        
        for f in range(3):
            ey = np.transpose([np.sin(self.baseAngles[f]), -np.cos(self.baseAngles[f]) , 0])    # normal to CC-PP-Zaxis plane

            ex = np.cross(ey, self.PP[:,f])                                                     # vector along L1 link at alpha=0
            ex = ex/np.linalg.norm(ex)

            RR[:,f] = self.PP[:,f] + self.L1*np.cos(self.alpha[f])*ex + self.L1*np.sin(self.alpha[f])*ey

            ey2 = np.cross(self.PP[:,f], RR[:,f])               # normal to CC-PP-RR plane
            ey2 = ey2/np.linalg.norm(ey2)

            ex2 = np.cross(ey2, RR[:,f])                        # vector along L2 link at beta=0
            ex2 = ex2/np.linalg.norm(ex2)

            NNcalc[:,f] = RR[:,f] + self.L2*np.cos(self.beta[f])*ex2 + self.L2*np.sin(self.beta[f])*ey2

        return NNcalc


    def readEnc(self):
        # Return current alpha & beta for the 3 fingers
        angles = np.zeros(6)
        angles[0] = -self.XM1.read_angle() + self.motTOang[0]
        angles[1] = -self.XM2.read_angle() + self.motTOang[1]
        angles[2] = -self.XM3.read_angle() + self.motTOang[2]

        angles[3] = -self.XL1.read_angle() + self.motTOang[3]
        angles[4] = -self.XL2.read_angle() + self.motTOang[4]
        angles[5] = -self.XL3.read_angle() + self.motTOang[5]

        return angles

    def readMarkerPose(self, connection):

        connection.send('report')
        x_actual = None
        R_actual = None
        for i in range(20):             # make 20 attempts to read pose, if none initially found
            pose = connection.recv()
            if pose is not None:
                x_actual = pose[0]
                R_actual = pose[1]
                break

        if x_actual is None or R_actual is None:
            print("Error encountered, pose could not be measured, press any key to quit")
            getch()

        return x_actual, R_actual


    def moveRelRot_pose(self, R, connection):
        # Move object by rotation matrix R from current pose
        # Calls moveMotor() to desired alphas, betas
        # Record ArUco pose from camera

        self.NN = self.calcNN()         # Current contact locations

        NN_des = np.matmul(R, self.NN)  # Goal contact locations for rotation matrix R

        # print('Start invKin -->')
        alpha, beta, dist = self.invKin(NN_des)
        # print('--> invKin End.')

        if (dist > 1):
            print('ERROR: Point out of range!')
        else:
            self.moveMotors(alpha, beta)

        sleep(1)

        connection.send('report')
        x_actual = None
        R_actual = None
        for i in range(20):             # make 20 attempts to read pose, if none initially found
            pose = connection.recv()
            if pose is not None:
                x_actual = pose[0]
                R_actual = pose[1]
                break

        if x_actual is None or R_actual is None:
            print("Error encountered, pose could not be measured, press any key to quit")
            getch()

        return x_actual, R_actual


    def moveAbsRot_pose(self, R, connection):
        # Move object by rotation matrix R from pose at Grasp
        # Calls moveMotor() to desired alphas, betas
        # Record ArUco pose from camera

        NN_des = np.matmul(R, self.NN_grasp)  # Goal contact locations for rotation matrix R

        # print('Start invKin -->')
        alpha, beta, dist = self.invKin(NN_des)
        # print('--> invKin End.')

        if (dist > 1):
            print('ERROR: Point out of range!')
        else:
            self.moveMotors(alpha, beta)

        sleep(1)

        connection.send('report')
        x_actual = None
        R_actual = None
        for i in range(20):             # make 20 attempts to read pose, if none initially found
            pose = connection.recv()
            if pose is not None:
                x_actual = pose[0]
                R_actual = pose[1]
                break

        if x_actual is None or R_actual is None:
            print("Error encountered, pose could not be measured, press any key to quit")
            getch()

        return x_actual, R_actual


    def moveRelRot(self, R):
        # Move object by rotation matrix R from current pose
        # Calls moveMotor() to desired alphas, betas

        self.NN = self.calcNN()     # Current contact locations
        
        NN_des = np.matmul(R, self.NN)  # Goal contact locations for rotation matrix R

        # print('Start invKin -->')
        alpha, beta, dist = self.invKin(NN_des)
        # print('--> invKin End.')

        if (dist > 1):
            print('ERROR: Point out of range!')
        else:
            self.moveMotors(alpha, beta)

        # sleep(0.5)


    def moveAbsRot(self, R):
        # Move object by rotation matrix R from pose at Grasp
        # Calls moveMotor() to desired alphas, betas

        NN_des = np.matmul(R, self.NN_grasp)  # Goal contact locations for rotation matrix R

        # print('Start invKin -->')
        alpha, beta, dist = self.invKin(NN_des)
        # print('--> invKin End.')

        if (dist > 1):
            print('ERROR: Point out of range!')
        else:
            self.moveMotors(alpha, beta)

        # sleep(0.5)


    def moveMotors(self, alpha, beta, speed_factor = 1):
        # Alpha joint limits
        LOWER_a = 0*np.pi/180
        UPPER_a = 90*np.pi/180

        # Beta joint limits
        LOWER_b = -115*np.pi/180
        UPPER_b = 40*np.pi/180

        vals = np.zeros(6)
        flag = 0
        for i in range(3):
            if ((alpha[i] <= UPPER_a and alpha[i] >= LOWER_a) and (beta[i] <= UPPER_b and beta[i] >= LOWER_b)):
                vals[i] = (- alpha[i] + self.motTOang[i])
                vals[i+3] = (- beta[i] + self.motTOang[i+3])

                if (vals[i+3] < -np.pi):
                    vals[i+3] = 2*np.pi + vals[i+3]

                if (vals[i+3] > np.pi):
                    vals[i+3] = - 2*np.pi + vals[i+3]

            else:
                print('ERROR: Motor '+ str(i+1) + 'or' + str(i+4) +' out of range!')
                print(alpha[i]*180/np.pi)
                print(beta[i]*180/np.pi)
                flag = flag + 1


        if (flag == 0):
            # setting max velocity XM motors can reach
            speed_a = 0.03*speed_factor
            self.XM1.set_velprofile(speed_a)
            self.XM2.set_velprofile(speed_a)
            self.XM3.set_velprofile(speed_a)

            speed_b = 0.1*speed_factor          # speed of beta motors: range [0,1]
            self.XL1.set_velprofile(speed_b)
            self.XL2.set_velprofile(speed_b)
            self.XL3.set_velprofile(speed_b)

            self.XM1.move_angle(vals[0], blocking=False)
            self.XL1.move_angle(vals[3], blocking=False)

            self.XM2.move_angle(vals[1], blocking=False)
            self.XL2.move_angle(vals[4], blocking=False)

            self.XM3.move_angle(vals[2], blocking=False)
            self.XL3.move_angle(vals[5], blocking=False)
    


    def invKin(self, NN_des):
        # Returns Motor Positions (alpha & beta)

        ### USING KD TREE SEARCH
        # Converting to Matlab datatype
        mNN = matlab.double(NN_des.tolist())
        mNN = eng.reshape(mNN, 3, 3)

        alpha_read = self.readEnc()[0:3]
        malpha_i = matlab.double(alpha_read.tolist())
        malpha_i = eng.reshape(malpha_i, 1, 3)
        beta_read = self.readEnc()[3:6]
        mbeta_i = matlab.double(beta_read.tolist())
        mbeta_i = eng.reshape(mbeta_i, 1, 3)

        alpha, beta, dist = eng.searchKDTree(mNN, malpha_i, mbeta_i, nargout=3)     # first time only - run genKDTree in Matlab to generate kd-data.mat

        alpha = np.asarray(alpha).flatten()
        beta  = np.asarray(beta).flatten()
        dist = np.asarray(dist).flatten()

        return alpha, beta, max(dist)