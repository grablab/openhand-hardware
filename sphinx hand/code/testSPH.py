from controlSPH import SPH
import numpy as np
import math
from time import sleep

from random import randint
import scipy.io

import os

import msvcrt
def getch():
    return msvcrt.getch().decode()

def genEulerR(angle):  # XYZ Euler Angles
    roll, pitch, yaw = angle[0], angle[1], angle[2]
    roll = roll*np.pi/180
    pitch = pitch*np.pi/180
    yaw = yaw*np.pi/180 
    c1 = np.cos(roll)
    s1 = np.sin(roll)
    c2 = np.cos(pitch)
    s2 = np.sin(pitch)
    c3 = np.cos(yaw)
    s3 = np.sin(yaw)
    R = np.matrix([[c2*c3, -c2*s3, s2],
                   [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                   [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    return R


if __name__ == '__main__':

    sph = SPH()
    sph.initHand()

    sleep(2)

    print('Press any character to close the hand (ESC to skip)')
    if getch() != chr(0x1b):
        sph.graspHand()

        getch()
        print('Angles:')
        print(sph.readEnc()*180/np.pi)

    print('Press any character to Abs X-rot 5deg (ESC to skip)')
    if getch() != chr(0x1b):
        R = genEulerR([5, 0, 0])
        sph.moveAbsRot(R)

        getch()
        print('Angles:')
        print(sph.readEnc()*180/np.pi)

    getch()

    sph.homeHand()


