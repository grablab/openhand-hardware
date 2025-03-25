import numpy as np

import sys
sys.path.insert(1,'/opt/ros/kinetic/lib/python2.7/dist-packages') # hack workaround to load proper version of open-cv

import cv2
import cv2.aruco as aruco
import os
import time
# import cPickle as pickle
import pickle
from multiprocessing import Process,Pipe

def monitorArucoPose(connection):
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)     # added cv2.CAP_DSHOW to overcome warning ''[ WARN:0] terminating async callback'' (Vatsal)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    # os.system("v4l2-ctl -c focus_absolute=0") # make system call to set camera focus # removed for Windows (Vatsal)

    calibrationError,cameraMatrix,distCoeffs = pickle.load(open('aruco/cameraCalibration.p','rb'))

    aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)

    # generate marker board with one marker on it, offset from center of cylinder by half the cylinder height
    b = 0.030353/2  # half side length of marker
    h = 0.055499/2 # half height of cylinder
    # h = 0.0243205 # height to center of ball
    board_corners = [np.array([[-b,b,h],[b,b,h],[b,-b,h],[-b,-b,h]],dtype=np.float32)]
    board_ids = np.array( [[2]], dtype=np.int32)
    board = aruco.Board_create( board_corners,aruco_dict,board_ids )

    parameters =  aruco.DetectorParameters_create()

    R_cal = np.identity(3)
    R_init = np.identity(3)
    x_init = np.transpose(np.array([0,0,0]))

    while 1:
        ret, frame = cap.read()
        new_image = frame # store current frame to modify by overlaying marker poses

        R_cam = None
        obj_pose = None
        corners, ids, rejectedImgPoints = aruco.detectMarkers(new_image, aruco_dict, parameters=parameters)
        
        # method 1: estimating as board
        _,rvec,x_cam = aruco.estimatePoseBoard( corners, ids, board, cameraMatrix, distCoeffs, rvec=None, tvec=None )

        new_image = aruco.drawDetectedMarkers(new_image, corners)
        if rvec is not None and x_cam is not None:
            R_cam = axisAngleRot(rvec)
            R = np.transpose(R_cal)*R_cam*np.transpose(R_init)*R_cal
            x = np.asarray(np.transpose(R_cal*(x_cam-x_init)))[0]
            obj_pose = [x,R]
            aruco.drawAxis(new_image,cameraMatrix,distCoeffs,rvec,x_cam,0.05)

        # display modified image
        cv2.imshow('frame',new_image)
        cv2.waitKey(1)

        msg = None
        while connection.poll(): # get most recent message
            msg = connection.recv() # get instruction from main program

        if msg == 'calibrate camera':
            pose = cv2.aruco.estimatePoseSingleMarkers(corners,0.0635,cameraMatrix,distCoeffs)
            rvec = pose[0][0][0] # don't know why there are two extra levels, but this works
            x_cal = pose[1][0][0]
            R_cal = axisAngleRot(rvec)
            connection.send('Camera calibrated')
        elif msg == 'calibrate object':
            R_init = R_cam
            x_init = x_cam
            connection.send('Object calibrated')
            continue
        elif msg == 'report':
            connection.send(obj_pose)
        elif msg == 'stop':
            break

        time.sleep(0.1)

    # close camera, screen window, and pipe connection
    cap.release()
    cv2.destroyAllWindows()
    connection.close()

def axisAngleRot(rvec):
    # generate rotation matrix from rotation vector
    angle = np.linalg.norm(rvec)
    axis = [float(r/angle) for r in rvec]
    x,y,z = axis[0], axis[1], axis[2]
    c,s = np.cos(angle), np.sin(angle)

    result = np.matrix([[c+x**2*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y**2*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z**2*(1-c)]])
    return result