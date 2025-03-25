import time
import cv2
import cv2.aruco as A
import numpy as np
import os
# import cPickle as pickle
import pickle

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
# os.system("v4l2-ctl -c focus_absolute=0") # make system call to set camera focus # removed for Windows (Vatsal)

allCorners = []
allIds = []

# decimator = 1
i = 0
# for i in range(100):
while 1:
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)
    
    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if cv2.waitKey(1) & 0xFF == ord('r'):
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3:
                allCorners.append(res2[1])
                allIds.append(res2[2])
                print("Photo # {} successfully taken.".format(i))
                i = i + 1
        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if i == 30:
        break

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
# allCorners_float = allCorners #[item.astype(float) for item in allCorners]
# print type(allCorners_float)
# print type(allCorners_float[1])
# print type(allCorners_float[1][1])
# print type(allCorners_float[1][1][0])


#print allCorners

cameraMatrix = np.array([])
distCoeffs = np.array([])
cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,cameraMatrix,distCoeffs)
print('calibrated!!')

calibrationError = cal[0]
cameraMatrix = cal[1]
distCoeffs = cal[2]
pickle.dump([calibrationError,cameraMatrix,distCoeffs],open('cameraCalibration.p','wb'))

# cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
# try:  
#     print 'Attempting Calibration'
#     allCorners_float = [float(item) for item in allCorners]
#     cal = cv2.aruco.calibrateCameraCharuco(allCorners_float,allIds,board,imsize,None,None)
#     print 'Calibration Complete'
# except:
#     print "Calibration Failed, exiting ..."
#     cap.release()

cap.release()
cv2.destroyAllWindows()