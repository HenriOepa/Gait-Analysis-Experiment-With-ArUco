#python posEst.py -v test_vid3.mp4 -m calMtx.npy -s 10
#python posEst.py -v [videoPath] -m [calibrationMatrix] -s [sizeOfMarker]
#python posEst.py -v cam1.tts -m labMtx.npy -s 100

from asyncore import write
from queue import Empty
from tracemalloc import start
import numpy as np
import cv2
import argparse
import cv2.aruco as aruco
import time
import math
import ctypes
import os
import sys
import matplotlib.pyplot as mpl, array as arr

cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

#Parameters for usage
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=False, help="Path to the video file")
ap.add_argument("-m", "--calMatrix", required=True, help="Path to the calibration matrix of the camera")
ap.add_argument("-s", "--size", required=True, type=int, help="Size of the aruco marker in mm")
args = vars(ap.parse_args())


#Load distortion coeffs and camera matrix from file
with open(args["calMatrix"], 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

#Aruco dictionary that is used for detection
#aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)

#For getting information about the system that the program is ran on
user32 = ctypes.windll.user32

#To help know whether the program uses video or camera
#Set below, value will be either "", "video" or "camera"
vidOrCam = ""

#Check if video parameter is gives, assume camera if can't get length
try:
    len(args["video"])
    vidOrCam = "video"
except:
    vidOrCam = "camera"

    
if vidOrCam == "video":
    #Open video from path provided
    cap = cv2.VideoCapture(args["video"])
    #Get video resolution
    vid_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vid_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    longestSide = 0

    #Auto resize window, systemMetric(1) is the height of the screen, 0 is the width
    #If width is smaller than height, take height as the basis by which to resize
    if vid_width < vid_height:
        longestSide = vid_height
        #Find the ratio between height and width
        ratio = vid_height / vid_width
        #Multiplied by 0.75 so the screen would be a bit smaller
        screensizeWidth = int((user32.GetSystemMetrics(1) / ratio) * 0.75)
        cv2.moveWindow("frame", int((user32.GetSystemMetrics(0) - screensizeWidth) / 2), int((user32.GetSystemMetrics(1) - (user32.GetSystemMetrics(1) * 0.75))/ 2))
        cv2.resizeWindow('frame', screensizeWidth, int(user32.GetSystemMetrics(1) * 0.75))

    #If width is bigger or equal to height, take width as the basis by which to resize
    elif vid_width >= vid_height:
        longestSide = vid_width
        ratio = vid_width / vid_height
        screensizeHeight = int((user32.GetSystemMetrics(0) / ratio) * 0.75)
        cv2.moveWindow('frame', int((user32.GetSystemMetrics(0) - (user32.GetSystemMetrics(0) * 0.75))/ 2), int((user32.GetSystemMetrics(1) - screensizeHeight) / 2))
        cv2.resizeWindow('frame', int(user32.GetSystemMetrics(0) * 0.75), screensizeHeight)

else:
    #If video is not found, try to open web cam
    try:
        cap = cv2.VideoCapture(0)
    except:
        #Quit the program if nothing webcam nor video could be opened
        print("ERROR: Camera could not be opened")
        quit()

#For getting information about the system
user32 = ctypes.windll.user32

#Get screen resolution, where 0 is width and 1 is height (pixels)
screensize = user32.GetSystemMetrics(0) - 100, user32.GetSystemMetrics(1) - 100

#Create folder(s) for measurements if it does not already exist
timeOfUse = time.asctime().replace(" ", "").replace(":","")
file_path = os.path.dirname(os.path.realpath(sys.argv[0]))
folderName = os.path.join(file_path, 'Measurements\Measurement_' + str(timeOfUse))

if not os.path.exists(folderName):
    os.makedirs(folderName)

prev_time = 0
timePassed = 0
prevCoords = []
indexes = []
#Start coords is purely for testing
startCoords = []
prevData = ""

#======================================================================================#
#From learnopencv: https://learnopencv.com/rotation-matrix-to-euler-angles/

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])
#==========================================================================================#


#Write the data from markers inot a file
def writeToFile(id, data):
    filePath = str(folderName) + '/' + str(id) + '_measurement.txt'

    #Checks if file already exists, if not, creates it 
    if not os.path.exists(file_path):
        f = open(filePath, 'x')
        f.close()

    #Write data to file
    #Format will be aaa,bbb,ccc,ddd etc
    dataToWrite = ','.join(data)
    f = open(filePath, 'a')
    #\n in the end so the data would not be written in a single row
    f.write(dataToWrite + '\n')
    f.close()


def createGraph(id):
    #Holds previous time
    previousTime = 0
    #Holds previous vector length
    previousLen = 0

    for index in id:

        plotTime = []
        plotVector = []
        values = []
        maxY = 0
        minY = 9.814

        #Path to the file where measurements are gotten from
        filePath = str(folderName) + '/' + str(index) + '_measurement.txt'

        #Read a line from file, get the data and store it. Find maximum y value
        with open(filePath) as f:

            while (line := f.readline().rstrip()):
                #Clears old entries
                values.clear()
                #Values are gotten from the file, split from comma
                values.extend(line.split(","))
                plotTime.append(float(values[0]))

                try:
                    #Because length is in mm, have to divide by 1000 to get value in meters
                    #Time is already represented in seconds
                    #Formula for acceleration is (current distance - previous distance) / time * time
                    #values[5] is vector length, taken from file
                    acceleration = (((float(values[5]) - previousLen) / 1000) / math.pow((float(values[0]) - previousTime), 2))

                except:
                    acceleration = 0

                plotVector.append(acceleration)
                previousTime = float(values[0])
                previousLen = float(values[5])

                if acceleration > maxY:
                    maxY = acceleration

                if acceleration < minY:
                    minY = acceleration


        f.close()

        #To stop weird graphs from showing
        if maxY == 0:
            maxY = 1
        if minY == 0:
            minY = 1


        titleName = str(index) + ' tag measurement'
        

        #Create graphs
        mpl.title(titleName)
        mpl.plot(plotTime, plotVector, scaley=True)
        mpl.xticks(np.arange(0, float(values[0]), step = (float(values[0]) / 10)))
        mpl.yticks(np.arange(minY, maxY, step = ((maxY - minY + 0.001)/ 10)))
        mpl.xlabel("Time (s)")
        mpl.ylabel("Acceleration (m^2)")
        mpl.grid()
        mpl.show()

#Checks if data is a repeat of itself
#Used to avoid having constant duplicate datasets
#TO DO: Get rid of duplicate data by not having it in the first place
def checkData(dataWrite, coordX, coordY, dist, rot):
    global prevData
    checkData = ("{:.2f}".format(rot), "{:.2f}".format(coordX),
    "{:.2f}".format(coordY), "{:.2f}".format(dist))

    currentData = "".join(checkData)

    if currentData != prevData:
        writeToFile(ids[index], dataWrite)

    prevData = "".join(checkData)


#Get fps rate of the source, if the source is a video
if vidOrCam == "video":
    fps = cap.get(cv2.CAP_PROP_FPS)
elif vidOrCam == "camera":
    fps = 960


#Start time of the program
startTime = time.time()
count = 0

while True:
    current_time = time.time()

    #FPS control
    if((current_time - prev_time) >= (1 / fps)): 

        #Get frame from source
        ret, frame = cap.read()

        #Break if no frame available
        if not ret:
            break
        
        #Time passed is the time between two frames when using a video
        #Avoids graphs being weird due to slowdown from processing the frames
        timePassed += 1 / fps

        #Create a mask of the frame captured
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #Detector parameters to improve tag detection
        parameters = aruco.DetectorParameters_create()
        #value 1 is subpixel corner refinement
        parameters.cornerRefinementMethod = 1
        parameters.cornerRefinementMaxIterations = 60
        parameters.cornerRefinementWinSize = 10
        parameters.cornerRefinementMinAccuracy = 0.03
        parameters.polygonalApproxAccuracyRate = 0.02
        parameters.perspectiveRemovePixelPerCell = 8
        parameters.maxErroneousBitsInBorderRate = 0.4
        parameters.errorCorrectionRate = 0.6
        parameters.markerBorderBits = 1
        #parameters.adaptiveThreshWinSizeStep = 5
        #parameters.adaptiveThreshWinSizeMax = 40
        #parameters.adaptiveThreshWinSizeMin = 8
        #parameters.maxMarkerPerimeterRate = 6.0

        #Detect markers on the frame, get corners, ids and rejected areas
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, camera_matrix, camera_distortion, parameters=parameters)

        #outline rejected areas with red lines, for debugging purposes
        for rejected in rejected_img_points:
            rejected = rejected.reshape((4, 2))
            cv2.polylines(frame, [rejected.astype(np.int32)], True, (0, 0, 255), 2, cv2.LINE_AA)
        
        if corners:
            aruco.drawDetectedMarkers(frame, corners)

            eparams = aruco.EstimateParameters_create()

            #Estimate marker pose, get translation and rotation vectors
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, args["size"], camera_matrix, camera_distortion, estimateParameters=eparams)
            marker_num = range(0, ids.size)

            for markerIds, markerCorn, index in zip(ids, corners, marker_num):

                rvec = rvecs[index][0]
                tvec = tvecs[index][0]
                
                #Witchcraft with rotations/rotation matrixes
                #Get camera's position in relation to the marker
                #May not be totally useful
                #For vectors, marker's own coordinates in relation to camera are more accurate
                #And less prone to 
                rvec_Temp = rvec * -1
                tvec_Temp = tvec * -1
                rotation_matrix, jacobian = cv2.Rodrigues(rvec_Temp)
                real_tvec = np.dot(rotation_matrix, tvec_Temp)
                pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)

                #Outline detected markers with yellow lines
                cv2.polylines(frame, [markerCorn.astype(np.int32)], True, (0, 255, 255), 2, cv2.LINE_AA)
                
                #Tag positions(where to place text on screen)
                markerCorn = markerCorn.reshape(4, 2)
                markerCorn = markerCorn.astype(int)
                top_right = markerCorn[0].ravel()
                top_left = markerCorn[1].ravel()
                bottom_right = markerCorn[2].ravel()
                bottom_left = markerCorn[3].ravel()

                #Draw axes and put text "ID: [NUM]"
                cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 30, 2)
                cv2.putText(frame, f"ID:%3.0f"%(markerIds[0]), top_right, cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
                
                #Time between now and start
                if vidOrCam == "camera":
                    timeBetween = (current_time - startTime)
                elif vidOrCam == "video":
                    timeBetween = timePassed

                #If list is empty, set values
                try:
                    _ = prevCoords[index][0]
                except:
                    #prevCoords.insert(index, [ids[index], real_tvec[0], real_tvec[1], tvecs[index][0][2]])
                    prevCoords.insert(index, [ids[index], tvec[0], tvec[1], tvecs[index][0][2]])

                #Set starting coordinates, check how much marker has moved from a position
                #Not really useful
                try:
                    _ = startCoords[index][0]
                except:
                    startCoords.insert(index, [markerIds[0], tvec[0], tvec[1], tvecs[index][0][2]])

                #Get indexes that have been detected
                #Needs some extra work
                try:
                    _ = indexes[index]
                except:
                    indexes.insert(0,ids[index])

                #Find correct id from list
                #Prevents the occasional data being wrongly placed
                for count in range(0, len(prevCoords)):
                    
                    if prevCoords[count][0] == markerIds[0]:
                        idNum = count
                        break

                    count += 1

                #If id has not appreared previously, add to list
                if ids[index] not in indexes:
                    indexes.append(ids[index])

                #Calculate vector length based on the coordinates

                moveVector = math.sqrt(math.pow(tvec[0] + (-prevCoords[idNum][1]), 2) + math.pow(tvec[1] + (-prevCoords[idNum][2]), 2) +
                math.pow(tvecs[index][0][2] + (-prevCoords[idNum][3]), 2))

                #Format of the data to be written to the file
                data2Write = ("{:.5f}".format(timeBetween), "{:.2f}".format(math.degrees(yaw)), "{:.4f}".format(tvec[0]),
                "{:.4f}".format(tvec[1]), "{:.4f}".format(tvecs[index][0][2]), "{:.8f}".format(moveVector)) #, str(rvecs[index])

                #Write measurement data into a file
                checkData(data2Write, tvec[0], tvec[1], tvecs[index][0][2], math.degrees(yaw))

                #Show the values on the marker
                marker_info = " x=%3.0fmm, y=%3.0fmm, z=%2.2fmm"%( tvec[0], tvec[1], tvecs[index][0][2])

                cv2.putText(frame, marker_info, bottom_left, cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 250, 0), 1, cv2.LINE_8)

                #Save the coordinates so the vector could be calculated
                prevCoords.pop(idNum)
                prevCoords.insert(idNum, [markerIds[0], tvec[0], tvec[1], tvecs[index][0][2]])

                if cv2.getWindowProperty("frame", cv2.WND_PROP_VISIBLE) < 1:
                    break

        cv2.imshow("frame", frame)
        prev_time = current_time

        #If window is closed, close recording and display graph(s)
        if cv2.getWindowProperty("frame", cv2.WND_PROP_VISIBLE) < 1:
            break
        
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        #Close the video / end measurement
        break
    elif key == ord('p'):
        #Pause the video until a key is pressed
        key = cv2.waitKey(-1)
        startTime = time.time() - timeBetween

        
    #If window is closed, stop the program
    if cv2.getWindowProperty("frame", cv2.WND_PROP_VISIBLE) < 1:
        break

#Release cap and destroy windows
cap.release()
cv2.destroyAllWindows()

#Create a graph on the data
createGraph(indexes)
