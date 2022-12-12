#Checkerboard can be generated using https://calib.io/pages/camera-calibration-pattern-generator
#  python cameraCalib.py -v [vid path] -cb 14X6 -cbs 50 
#python cameraCalib.py -v [video path] -cb [checkerboard width]X[checkerboard height] -cbs [checkerboard square side length in mm]

from re import T
import numpy as np
import cv2
import time
import argparse
import sys
import glob
import os
import ctypes, subprocess, win32gui, win32con

#Make named window
cv2.namedWindow("Calibration Video", cv2.WINDOW_NORMAL)
cv2.moveWindow("Calibration Video", 50, 0)

#For start command
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=True, help="Path to the video file")
ap.add_argument("-cb", "--checkerboard", required=True, type=str, help="Checkerboard corner numbers and square size. Example 6X4 widthXheight")
ap.add_argument("-cbs", "--checkerboardSize", required=True, type=float, help="Checkerboard individual square size in mm")
ap.add_argument("-r", "--rate", required=False, type=int, default=20, help="The rate at which images are captured, default every 20 frames")
args = vars(ap.parse_args())

#Split the given string
cb_width2, cb_height2 = args["checkerboard"].split("X")

#Checkerboard width and height
cb_width = int(cb_width2) - 1
cb_height = int(cb_height2) - 1

#Open the video
cap = cv2.VideoCapture(args["video"])

#Get fps of the video
fps = cap.get(cv2.CAP_PROP_FPS)

#Get screen size, 0 is width, 1 is height in pixels
user32 = ctypes.windll.user32
screensize = user32.GetSystemMetrics(0) - 100, user32.GetSystemMetrics(1) - 100

#Resize the window
cv2.resizeWindow("Calibration Video", screensize)

prev_frame_time = time.time()
cal_image_count = 0
frame_count = 0

#Time of calibration for use in naming folders
timeOfCal = time.asctime().replace(" ", "").replace(":","")
#Get current path where program is launched
file_path = os.path.dirname(os.path.realpath(sys.argv[0]))
#Path to the folder that will be created
folderName = os.path.join(file_path, 'Calibrations\cal_' + str(timeOfCal))

#For calibration
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cb_3d_points = np.zeros((cb_width * cb_height, 3), np.float32)
cb_3d_points[:,:2] = np.mgrid[0:cb_width, 0:cb_height].T.reshape(-1,2) * args["checkerboardSize"]
cb_2d_img_points = []
cb3dpoints = []

#Check is window with name is open
def foreach_window(winName):
    #Find the window with the name
    window = win32gui.FindWindow(None, winName)
    if window:
        #Get the placement of the window
        tup = win32gui.GetWindowPlacement(window)
        #If window is maximized, minimized or normal, return true. Otherwise, return false
        if tup[1] == win32con.SW_SHOWMAXIMIZED or tup[1] == win32con.SW_SHOWMINIMIZED or tup[1] == win32con.SW_SHOWNORMAL:
            return True
        else:
            return False

#Allow user to shift through the pictures
def clearImages(path):
    #Open explorer.exe, so a folder can be opened
    FILEBROWSER_PATH = os.path.join(os.getenv('WINDIR'), 'explorer.exe')

    subprocess.Popen([FILEBROWSER_PATH, path])

    #Wait a little
    time.sleep(2)

    #Name of the folder which to check
    nameOfFolder = str('cal_' + str(timeOfCal))

    #While window with the nameOfFolder is open, sleep
    while foreach_window(nameOfFolder) is True:
        time.sleep(1)


#Check if folder exists, if not then make it
if not os.path.exists(folderName):
    os.makedirs(folderName)


#Main loop for getting pictures
while True:
    #Get current time
    new_frame_time = time.time()

    #At higher image numbers, the calibration tends to crash
    if frame_count >= 90:
        break

    #Slows down the displaying of images to the rate of the video
    if (new_frame_time - prev_frame_time) >= (1 / fps):
        #Get images
        ret, frame = cap.read()
        #If image was not gotten, break
        if not ret:
            break
        #Add 1 to frame count
        frame_count += 1

        #Capture pictures after every X amount of frames
        #As described with rate
        #Save them to the folder in img_path as .png files
        if frame_count == args["rate"]:
            img_path = os.path.join(folderName, 'cal_image' + str(cal_image_count) + '.png')
            cv2.imwrite(img_path, frame)
            cal_image_count += 1
            #Reset frame_count so after "rate" amount another image can be captured
            frame_count = 0

        #Previous time = new time so check the last time the next cycle
        prev_frame_time = new_frame_time
        #Show how many images have been taken
        cv2.putText(frame, "cal_imgs:" + str(int(cal_image_count)), (10, 60), cv2.FONT_HERSHEY_DUPLEX, 2, (100, 200, 50), 2, cv2.LINE_AA)
        #Show image
        cv2.imshow("Calibration Video", frame)
    
    #Break if q is pressed or window is closed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
         break

    if cv2.getWindowProperty("Calibration Video", cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()

#If folder with the folder name is open, the program is halted
#So that the user can shift through the pictures
#And remove the blurry ones
clearImages(folderName)

#Create window where to display captured images
cv2.namedWindow("Calibration Images", cv2.WINDOW_NORMAL)
cv2.moveWindow("Calibration Images", 50, 0)

#Get all images with .png extension from folder
list_images = glob.glob(str(folderName) + '/*.png')

#Display all images from the file with 3d points
for frame_name in list_images:
    #Get the frame
    img = cv2.imread(frame_name)
    #Mask of the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #Find the corners of the used chessboard
    ret, corners = cv2.findChessboardCorners(gray, (cb_width, cb_height), None)

    if ret == True:
        #Get the required points to display them on image
        cb3dpoints.append(cb_3d_points)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        cb_2d_img_points.append(corners2)

        #Draw the dots on the chessboard on image
        cv2.drawChessboardCorners(img, (cb_width, cb_height), corners2, ret)
        cv2.imshow('Calibration Images', img)
        #Delay between each picture
        cv2.waitKey(30)

cv2.destroyAllWindows()

#Get the calibration matrix
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(cb3dpoints, cb_2d_img_points, gray.shape[::-1], None, None)

#Create file path for the matrix
mtxFilePath = str(folderName) + '/' + str(timeOfCal) + '_calMtx.npy'

#Open file and write matrix information in it
with open(mtxFilePath, 'wb') as f:
    np.save(f, mtx)
    np.save(f, dist)
f.close()
