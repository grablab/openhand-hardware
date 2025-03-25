import cv2
import cv2.aruco as aruco
 
 
'''
    drawMarker(...)
        drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
'''
 
aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
print(aruco_dict)
# second parameter is id number
# last parameter is total image size
id_number = 4
img = aruco.drawMarker(aruco_dict, id_number, 700)
cv2.imwrite("test_marker_{}.jpg".format(id_number), img)
 
cv2.imshow('frame',img)
cv2.waitKey(0)
cv2.destroyAllWindows()