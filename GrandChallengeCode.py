'''
Program Obejctive: Grand Challenge - ENPM809T - Autonomous Systems
Author: Arshad S.
Version History: 
Version  -      Date         -   Changes
1.0      -   04/18/2023  -          All
2.0      -  07/07/2023   -          All

'''
# import necessary packages
import cv2
import os
import glob
import numpy as np
import RPi.GPIO as gpio
from time import sleep
import serial
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime
import matplotlib.pyplot as plt
import csv
import imutils
import sys
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

###################################################################
# Function definitions
###################################################################
# Ref: https://www.raspberrypi.com/documentation/accessories/camera.html
class cameraCalculations:

    def __init__(self, IMG_WIDTH, IMG_HEIGHT, IMG_COLOR):
        self.IMG_WD = IMG_WIDTH
        self.IMG_HT = IMG_HEIGHT
        self.IMG_CLR = IMG_COLOR
        self.FOCAL_LENGTH = 3.04
        self.REAL_OBJ_HEIGHT = 57 # in mm
        self.REAL_OBJ_WIDTH = 38 # in mm        
        self.SNSR_HT_PX = IMG_HEIGHT # in px
        self.SNSR_HT = 2.7657 # in mm
        self.SNSR_WD = 3.68 # in mm
        self.HFOV = 62.2
        self.VFOV = 48.8
        self.CAMERA_TO_IMU_DIST = 95 # in mm
        self.CAM_FRM_RATE = 25
        self.img_dir_name = 'image_data'
        self.img_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.img_dir_name)
        self.HSV_RED_MIN = [152, 128, 100] # ideal from myhome ambience light : [170, 80, 0]
        self.HSV_RED_MAX = [179, 255, 255]
        self.HSV_BLU_MIN = [100, 96, 97] # ideal from myhome ambience light : [170, 80, 0]
        self.HSV_BLU_MAX = [125, 255, 255]
        self.HSV_GRN_MIN = [40, 10, 16] # ideal from myhome ambience light : [170, 80, 0]
        self.HSV_GRN_MAX = [96, 255, 230]

        self.prevValue = 0
        self.DIST_BAND = 10
        self.CAMERA_TO_GRPR_DIST = 140
        self.CAMERA_TO_GRPROPENING_DIS =  140 # ideal 80 
        self.blockPickupConfirmRadius = 175
        self.blockPickedupRadius = 300


        self.K = round((self.REAL_OBJ_WIDTH / self.REAL_OBJ_HEIGHT),3) # ratio of width to height
        # print("\nObject Sides Ratio: ", self.K)

    def clrImgDir(self):
        try:
            files = glob.glob(os.path.join(self.img_dir_path, "*.jpg"))
            for f in files:
                os.remove(f)
        except Exception as e:
            print("Exception: ", e)
  
    # Function to take a snapshot from RPi
    def takeImgRPi(self):
        self.clrImgDir()
        print("Taking a picture..")
        #Define time stamp & record an image
        pic_time = datetime.now().strftime("%Y%m%d%H%M%S")
        name = pic_time + '.jpg'
        command = 'raspistill -w ' + str(self.IMG_WD) + ' -h ' + str(self.IMG_HT) +' -vf -o ' + str(self.img_dir_path) + str('/') + name
        # print(command)
        os.system(command)        
        image = cv2.imread(name)                   
        # cv2.imshow(name, image)
        # cv2.waitKey(1)
        return image

    def bandFilter(self, newValue, band):        
        if newValue >= (self.prevValue + (band / 2)) or newValue <= (self.prevValue - (band / 2)):
            output = newValue
        else:            
            output = self.prevValue
        
        self.prevValue = output

        return output


    def distanceToObject(self, image, x, y, objHtPx):
        # Ref: https://www.scantips.com/lights/subjectdistance.html    
        objHeightOnSensor = (objHtPx / self.SNSR_HT_PX) * self.SNSR_HT
        distanceToObject = (self.REAL_OBJ_HEIGHT / objHeightOnSensor) * self.FOCAL_LENGTH
        offset = distanceToObject * 0.25
        distanceToObject += offset
        # distanceToObject = self.bandFilter(distanceToObject, self.DIST_BAND)
        cv2.line(image, (int(x), int(y)), (int(self.IMG_WD / 2), int(self.IMG_HT)),(0,0,255),3)
        cv2.circle(image, (int(self.IMG_WD/2), int(self.IMG_HT-10)), int(10), (255, 255, 0), -1)
        imgText = "Distance: " + str(int(distanceToObject)) + " mm"
        fontScale = 1
        cv2.putText(image,imgText, (int((self.IMG_WD / 2) - 130), int(self.IMG_HT - 150)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0,255,0), 3)
        # print("\nDistance to the object: ", distanceToObject)

        return image, distanceToObject
    
    def angleToObject(self, image, x, y, radius, distanceToObject):         
        # Drawing center lines 
        # cv2.line(image, (int(self.IMG_WD / 2) - 50, int(self.IMG_HT / 2)), (int(self.IMG_WD / 2) + 50, int(self.IMG_HT / 2)),(100,255,100),3)
        # cv2.line(image, (int(self.IMG_WD / 2), int(self.IMG_HT / 2) - 50), (int(self.IMG_WD / 2), int(self.IMG_HT / 2) + 50),(100,255,100),3)
        cv2.line(image, (int(self.IMG_WD / 2), int(y)), (int(self.IMG_WD / 2), int(self.IMG_HT)),(255, 0, 0),3)        

        degPerPx = self.HFOV / self.IMG_WD
        angleCamera = (int(x) - (self.IMG_WD / 2)) * degPerPx

        # mmPerPx = self.SNSR_WD / self.IMG_WD
        # opp_side = (abs((self.IMG_WD / 2) - x) * mmPerPx)
        # d = opp_side / math.sin(math.radians(angleCamera))
        # d = abs(d)
        # print("\nd: ", d)

        # denominator1 = (d * math.cos(math.radians(angleCamera))) + self.FOCAL_LENGTH + self.CAMERA_TO_IMU_DIST
        # numerator1 = d * math.sin(math.radians(angleCamera))
        
        denominator = (distanceToObject * math.cos(math.radians(angleCamera))) + self.FOCAL_LENGTH + self.CAMERA_TO_IMU_DIST
        numerator = distanceToObject * math.sin(math.radians(angleCamera))
        angleImu = math.atan((numerator / denominator)) 
        angleToObject = round(math.degrees(angleImu), 2)


        if angleToObject < 0:
            direction = 'Cam Angle: Left '
            xCoor = x + radius/2
        elif angleToObject > 0:
            direction = 'Cam Angle: Right '
            xCoor = x - radius/2
        else:
            direction = 'Cam Angle: Straight '
            xCoor = x

        # object center to cenetr line
        cv2.line(image, (int(xCoor), int(y)), (int(self.IMG_WD / 2), int(y)), (255, 0, 0), 3)
    
        # Angle of the object detected
        imgText1 = direction + str(abs(int(angleCamera))) + " deg "
        imgText2 = "IMU Angle: "+ str(abs(int(angleToObject))) + " deg"
        cv2.putText(image,imgText1, (int((self.IMG_WD / 2) - 130), int(self.IMG_HT - 50)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        cv2.putText(image,imgText2, (int((self.IMG_WD / 2) - 130), int(self.IMG_HT - 100)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        # print("\n Angle: ", angleToObject, "°")

        return angleToObject * 0.75
    
    def detectObject(self, camera, rawCapture):
        counterIm = 0
        counterNoIm = 0
        list_angleToObject = []
        list_distanceToObject = []
        # Video looping - for each image frame from the video, do the following:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):   

            x = 0
            y = 0
            radius = 0
            objHtPx = 0         
            # Read the image frame
            image = frame.array            
            
            # Convert to BGR image to HSV image
            hsvIm = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define the threshold values for HSV mask (Ref: colorpicker.py)
            if self.IMG_CLR == "Green":
                minHSV = np.array(self.HSV_GRN_MIN)
                maxHSV = np.array(self.HSV_GRN_MAX) 
                # print("\nMin & Max HSV values: ", minHSV, maxHSV)
            elif self.IMG_CLR == "Blue":
                minHSV = np.array(self.HSV_BLU_MIN)
                maxHSV = np.array(self.HSV_BLU_MAX) 
                # print("\nMin & Max HSV values: ", minHSV, maxHSV)
            elif self.IMG_CLR == "Red":
                minHSV = np.array(self.HSV_RED_MIN)
                maxHSV = np.array(self.HSV_RED_MAX) 
                # print("\nMin & Max HSV values: ", minHSV, maxHSV)
            else:
                minHSV = np.array(self.HSV_RED_MIN)
                maxHSV = np.array(self.HSV_RED_MAX) 
                # print("\nMin & Max HSV values: ", minHSV, maxHSV)


            # Create a mask
            maskHSV = cv2.inRange(hsvIm, minHSV, maxHSV)

            # Blur the masked image before detecting the corners
            blurIm_unflt = cv2.GaussianBlur(maskHSV,(3,3),0)

            # Apply erosion to remove white noise and dilate to remove black noise
            kernel = np.ones((5,5), np.uint8)
            blurIm = cv2.erode(blurIm_unflt, kernel, iterations=1)
            blurIm = cv2.erode(blurIm_unflt, kernel, iterations=1)
            # blurIm = cv2.dilate(blurIm, kernel, iterations=1)

            # Detect the top 5 corners using the cv2.goodFeaturesToTrack()
            # The top 2 corners will always be two most narrowed corners of the arrow head
            quality = 0.1 # varies from 0 to 1; close to 0 implies that even a slight corners are detected
            corners = cv2.goodFeaturesToTrack(blurIm,2,quality,10)
            len_corners = 0 
            # Display all types of processed images from Camera Feed - [Original - HSV - Masked - Blurred]
            cv2.namedWindow("Camera Feed: Mask - Processed Frame", cv2.WINDOW_NORMAL)
            # print('\nShape: maskHSV, blurIm', np.shape(maskHSV), np.shape(blurIm))
            cv2.imshow("Camera Feed: Mask - Processed Frame", np.hstack([maskHSV, blurIm]))
            cv2.resizeWindow("Camera Feed: Mask - Processed Frame", 640, 480)

            #Creating a opencv window - RGB video
            cv2.namedWindow('RGB Capture', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('RGB Capture', 640, 480)

            if isinstance(corners, type(None)):
                # print("\n Corners detected: ", len_corners)         
                
                # Check key to break from the loop
                key = cv2.waitKey(1) & 0xFF

                # press the 'q' key to exit
                if key == ord("q"):
                    break
            else:
                # print("\n Corners detected: ", len(corners))
                len_corners = len(corners)

            # Get the corners date to find the mid point of the arrow head
            if len_corners > 0 :                             
                #find the contour of the arrow
                (cont, _) = cv2.findContours(blurIm.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # Sort the contours from contours with larger areas to smaller areas
                cont = sorted(cont, key = lambda x: cv2.contourArea(x), reverse=True)
                # when the contour is detected, draw a enclosing circle, line from center of the circle to 
                # calculate orinetation of the arraow, display it on the image
                if (len(cont[0]) > 10): # Contour with larger area, is our contour-of-interest (observed values: > 200, especially when near 30 cm)
                    # print('\n Length of the contour1: ', len(cont[0]))
                    # Find the minimum enclosing circle for the  biggest contour
                    (x,y), radius = cv2.minEnclosingCircle(cont[0])
                    # print("\n [inside]Origin: ", (x,y), "radius:", radius)
                    # Draw a larger circle - minimum enclosing circle of the contour
                    cv2.circle(image, (int(x), int(y)), int(radius), (0, 0, 255), 3)
                    cv2.circle(image, (int(x), int(y)), int(10), (255, 255, 0), -1)
                    # Drawing lines - Minimum Enclosing Circle height
                    # cv2.line(image, (int(x),int(y-radius)), (int(x),int(y+radius)),(255,255,0),3)
                    
                    objHtPx = (2 * radius) / math.sqrt((1 + self.K ** 2))
                    
                    # Drawing lines - Object height
                    # cv2.line(image, (int(x+ (radius/2)),int(y-(objHtPx/2))), (int(x+(radius/2)),int(y+(objHtPx/2))),(255,255,0),3)
                    cv2.line(image, (int(x),int(y-(objHtPx/2))), (int(x),int(y+(objHtPx/2))), (255,255,0),3)
                    cv2.line(image, (int(x - (radius/2)),int(y)), (int(x + (radius/2)), int(y)), (255,255,0),3)
                    # print("\n Diameter & Object Height (px): ", (2 * radius), objHtPx)

                    image, distanceToObject = self.distanceToObject(image, int(x), int(y), objHtPx)
                    angleToObject = self.angleToObject(image, int(x), int(y), radius, distanceToObject)
                    list_distanceToObject.append(distanceToObject)
                    list_angleToObject.append(angleToObject)

                    counterIm += 1
                    # print("\n counterIm", counterIm)
                    cv2.imshow('RGB Capture', image) 
            else:
                print("\n\tNo Block Found!")  
                counterNoIm += 1               

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            
            # Check key to break from the loop
            cv2.waitKey(1)

            # press the 'q' key to exit
            if counterIm >= 3:
                distanceToObject = np.mean(np.array(list_distanceToObject))
                angleToObject = np.mean(np.array(list_angleToObject))
                break

            # press the 'q' key to exit
            if counterNoIm >= 3:
                distanceToObject = 0
                angleToObject = 0
                x = 0
                y = 0
                radius = 0
                break

       
                
        return image, int(x), int(y), radius, objHtPx, round((distanceToObject), 2), angleToObject
###################################################################
###################################################################
class moveNgrab:
    def __init__(self):
        self.HOP_DIST = 300
        self.LAST_MILE_DIST = 300
        self.WH_RADIUS = 32.25
               
    
    # def init(self):
        #### Initialize gpio pins ####

    ###################################################################

    def gameover(self):
        gpio.output(31, False)
        gpio.output(33, False)
        gpio.output(35, False)
        gpio.output(37, False)

        gpio.cleanup()
    ###################################################################
    # Function to rotate servo to the required position (takes 2 seconds)
    def ServoControl(self, pos):
        if pos == "full_closed":
            duty_cycle = 3.5
            pwmS.ChangeDutyCycle(duty_cycle)        
            time.sleep(2)
        elif pos == "pickup":
            duty_cycle = 3.8
            pwmS.ChangeDutyCycle(duty_cycle)
            time.sleep(2)
        elif pos == "partial_open":
            duty_cycle = 5.5
            pwmS.ChangeDutyCycle(duty_cycle)
            time.sleep(2)
        elif pos == "full_open":
            duty_cycle = 7.5
            pwmS.ChangeDutyCycle(duty_cycle)
            time.sleep(2)
        else:
            duty_cycle = 3.3
            pwmS.ChangeDutyCycle(duty_cycle)
            time.sleep(2)
        
        return duty_cycle
    ###################################################################
    def ReadIMUx(self, ser):
        
        while True:
            if (ser.in_waiting > 0):
                line = ser.readline()
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                line = line.strip("X: ")
                imu_x = float(line)
                break

        return imu_x
    ##################################################################

    ###################################################################
    def forward(self, distance, ser):
        list_enFL = []
        list_enBR = []
        list_dist = []

        wh_rev = distance * ( 1 / (2 * math.pi * (self.WH_RADIUS)) ) # prev ideal value = 0.03625
        wh_stop = int(960 * wh_rev)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        # Initialize pwmm signal to control motor
        # pwm3 = gpio.PWM(33, 50) # to control right wheels
        # pwm2 = gpio.PWM(35, 50) # to control left wheels

        val = 60

        pwm3.start(val)
        pwm2.start(val)

        time.sleep(0.1)

        while True:        
            # counting the encoder pulses BR
            if int(gpio.input(12)) != int(buttonBR):
                buttonBR = int(gpio.input(12))
                counterBR += 1

            # counting the encoder pulses
            if int(gpio.input(7)) != int(buttonFL):
                buttonFL = int(gpio.input(7))
                counterFL += 1
            
            #PROPORTIONAL CONTROLLER
            err = counterFL - counterBR
            # print("Error", err)
            kp = 2.0
            if err < 0:
                if (val + (-err*kp)) > 10 and (val + (-err*kp)) < 90:
                    pwm2.ChangeDutyCycle(val + (-err*kp))
                    #time.sleep(0.1)
                else:
                    print("Max dutycycle reached!")

            elif err>=0:
                if (val - (err*kp)) > 10 and (val - (err*kp)) < 90:
                    pwm2.ChangeDutyCycle(val - (err*kp))                
                    #time.sleep(0.1)
                else:
                    print("Min dutycycle reached!")                  
            
            # stopping the counter when the value reaches some point
            if counterFL >= wh_stop:
                pwm3.stop()
                pwm2.stop()
                break

            # stopping the counter when the value reaches some point
            if counterBR >= wh_stop:
                pwm2.stop()
                pwm3.stop()
                break

        list_enFL.append(counterFL)
        list_enBR.append(counterBR)

        return (counterFL, counterBR)
       
    ###################################################################

    def reverse(self, distance, ser):
        list_enFL = []
        list_enBR = []
        list_dist = []

        wh_rev = distance * ( 1 / (2 * math.pi * (self.WH_RADIUS)) ) # prev ideal value = 0.03625
        wh_stop = int(960 * wh_rev)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        # Initialize pwmm signal to control motor
        val = 60

        pwm1.start(val)
        pwm4.start(val)

        time.sleep(0.1)

        while True:        
            # counting the encoder pulses BR
            if int(gpio.input(12)) != int(buttonBR):
                buttonBR = int(gpio.input(12))
                counterBR += 1

            # counting the encoder pulses
            if int(gpio.input(7)) != int(buttonFL):
                buttonFL = int(gpio.input(7))
                counterFL += 1
            
            #PROPORTIONAL CONTROLLER
            err = counterFL - counterBR
            kp = 2.0
            if err < 0:
                if (val + (-err*kp)) > 10 and (val + (-err*kp)) < 90:
                    pwm1.ChangeDutyCycle(val + (-err*kp))
                    #time.sleep(0.1)
                else:
                    print("Max dutycycle reached!")

            elif err>=0:
                if (val - (err*kp)) > 10 and (val - (err*kp)) < 90:
                    pwm1.ChangeDutyCycle(val - (err*kp))                
                    #time.sleep(0.1)
                else:
                    print("Min dutycycle reached!")                  
            
            # stopping the counter when the value reaches some point
            if counterFL >= wh_stop:
                pwm1.stop()
                pwm4.stop()
                break

            # stopping the counter when the value reaches some point
            if counterBR >= wh_stop:
                pwm1.stop()
                pwm4.stop()
                break

        list_enFL.append(counterFL)
        list_enBR.append(counterBR)

        return (counterFL, counterBR)
       
    ###################################################################

    def left(self, val):
        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        # Initialize pwmm signal to control motor
        pwm3.start(val)
        pwm1.start(val)

        time.sleep(0.1)
 
    ###################################################################
    def right(self, val):
        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        # Initialize pwmm signal to control motor
        pwm2.start(val)
        pwm4.start(val)

        time.sleep(0.1)
  
    ###################################################################
    def anglePerioidicity(self, angle):# converts the angles from 0 to 360 to -180 to 180
        angle = angle % 360

        if angle > 180:
            angle = angle - 360
        else:
            angle = angle

        return angle

    ###################################################################
class obstacleAvoidance():
    def __init__(self):
        self.RBT_TO_SNR_OFST  = 20
        self.SNR_TO_GRPR_OFST = 150
        self.SAFE_DIST = (304 + 100 + 50 + self.SNR_TO_GRPR_OFST) # 1 Ft + gripper distance
            
    # Function to calculate instantaneous distance
    def distance(self):
        # Generate trigger pulse
        gpio.output(trig, True)
        time.sleep(0.00001)
        gpio.output(trig, False)

        # Generate echo time signal
        while gpio.input(echo) == 0:
            pulse_start = time.time()
        
        while gpio.input(echo) == 1:
            pulse_end = time.time()
        
        pulse_duration = pulse_end - pulse_start

        # Convert time to distance
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        return ((distance * 10) + self.RBT_TO_SNR_OFST) # returning in mm

    # Function to calculate average distance (takes 2 seconds)
    def AvgDistance(self, num_of_readings):
        obstSafeDist = False
        average_dist = np.array([])
        distances = []

        for i in range(num_of_readings):
            inst_val = self.distance()
            #print("Distance: ", inst_val, "mm")
            distances.append(inst_val)
            time.sleep(0.01)
            
        avg_dist = round(np.average(distances),2)
        # print("Average distance to the obstacle: ",avg_dist," mm")

        if avg_dist <= self.SAFE_DIST:
            obstSafeDist = True
        return avg_dist, obstSafeDist
###################################################################
class email_smtp:

    def __init__(self) -> None:
        #Email information
        self.smtpUser = 'enpm809ts23arsh@gmail.com'
        self.smtpPass = 'ksveygscusrwlfld'

        self.toAdd = 'ENPM809TS19@gmail.com'
        self.cc = 'rpatil10@umd.edu'
       
        self.fromAdd = self.smtpUser 
        self.mail_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'image_data')
           

    def clear_mail_dir(self):
        try:
            files = glob.glob(os.path.join(self.mail_directory, "*.jpg"))
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)


    def mail_snapshot(self, subject, pic_time):
        try:

            #Destination email information
            msg = MIMEMultipart()
            msg['Subject'] = subject
            msg['From'] = self.fromAdd
            msg['To'] = self.toAdd
            msg['cc'] = self.cc
           
            
            msg.preamble = "Attached Image recorded at "+ pic_time

            #Email text
            body = MIMEText('Attached Image recorded at '+ pic_time)
            msg.attach(body)
                
            #Attach Image
            fp = open(str(self.mail_directory) + str('/')+ pic_time + '.jpg', 'rb')

            img = MIMEImage(fp.read())
            img.add_header('Content-Disposition', "attachment; filename= %s" % "snapshot.jpg")
            fp.close()
            msg.attach(img)

            #Send email
            s = smtplib.SMTP('smtp.gmail.com', 587)

            s.ehlo()
            s.starttls()
            s.ehlo()

            s.login(self.smtpUser, self.smtpPass)
            s.sendmail(self.fromAdd, [self.toAdd, self.cc], msg.as_string())
            s.quit()

        except Exception as e:
            print("Email delivery failed!")
            print(e)

################## Define Class Ojects ################################
cr = cameraCalculations(IMG_WIDTH=1280, IMG_HEIGHT=720, IMG_COLOR="Red")
cg = cameraCalculations(IMG_WIDTH=1280, IMG_HEIGHT=720, IMG_COLOR="Green")
cb = cameraCalculations(IMG_WIDTH=1280, IMG_HEIGHT=720, IMG_COLOR="Blue")
ca = obstacleAvoidance()
mg = moveNgrab()
em = email_smtp()

############################ Start of Initializationns #####################################
pwrSwitch = bool(int(input("\nPower Switch is ON?(0/1)")))

# initialization: Motors & Encoders   
gpio.setmode(gpio.BOARD)

gpio.setup(31, gpio.OUT) # IN4
gpio.setup(33, gpio.OUT) # IN3
gpio.setup(35, gpio.OUT) # IN2
gpio.setup(37, gpio.OUT) # IN1

gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)  

# Servo Control - Define pin allocations
gpio.setup(36, gpio.OUT)

# Distance Measurement - Define pin allocations
trig = 16
echo = 18
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

######################################################
# Motor Control - Initialize PWM 
pwm3 = gpio.PWM(33, 50) # to control right wheels
pwm1 = gpio.PWM(37, 50) # to control left wheels
pwm2 = gpio.PWM(35, 50) # to control left wheels
pwm4 = gpio.PWM(31, 50) # to control right wheels

# Servo Control - Initialize PWM
pwmS = gpio.PWM(36, 50)
pwmS.start(5)

# Sonar - Initialize - Ensure output has no value
gpio.output(trig, False)
time.sleep(0.01)

# Camera - Initialize  #############
# initialize Camera
camera = PiCamera()
camera.resolution = (cr.IMG_WD, cr.IMG_HT)
camera.framerate = cr.CAM_FRM_RATE
camera.vflip = True
camera.hflip = True
rawCapture = PiRGBArray(camera, size=(cr.IMG_WD, cr.IMG_HT))
# allow the camera to warmup
time.sleep(0.1)
#######
# img, x, y, radius, objHtPx, distanceToObject, angleToObject = cr.detectObject(camera, rawCapture)
# print("\n Origin: ", (x,y), "\n radius:", radius, "\n Object Height (px): ", objHtPx, "\n Distance to Object: ", distanceToObject, "\n Angle to Object: ", angleToObject)
# cameraOK = input("Camera OK? (Y/N) ")
# print("\n[test] Camera: ", cameraOK)

# %% IMU - Initialize - Identify serial connection    
ser = serial.Serial('/dev/ttyUSB0', 9600)
countImu = 0
imu_x = 0
while True:    
    if (ser.in_waiting > 0):
        countImu += 1
        line = ser.readline()
        
        if countImu > 10:
            # Read serial stream and remove unnecessary characters
            line = line.rstrip().lstrip()
            line = str(line)
            line = line.strip("'")
            line = line.strip("b'")
            line = line.strip("X: ")
            imu_x = float(line)

            # print(imu_x, "\n")
            break
time.sleep(0.5)
print("\n[test] IMU: ", imu_x)  

# Calculate the 1st avg. obstacle distance of an object (4 samples - lessthan 1 seconds)
avgObsDist, safeDistFlg = ca.AvgDistance(4)
print("\n[test] Sonar: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)


# Move the gripper to Full- Closed position (2 sec)
print("Moving the gripper to full-closed position")
duty = mg.ServoControl("full_closed")
############################ End of Initializationns #####################################  

list_x = []
list_y = []
list_angle = []
global cur_x
cur_x = 0
countercheckNdrop = 0
imgNum = 1

f = open('encodertrajectory.txt','a')
em.clear_mail_dir()

##################### End of main initializations #######################################
def blockTransportEmail(objClr):
    print("\n########## Starting to send the email ##########")
    global imgNum
    img, _, _, _, radius, _, _ = objClr.detectObject(camera, rawCapture)
    #
    # objClr.clrImgDir()
    # Define time stamp & record an image
    pic_time = datetime.now().strftime("%Y%m%d%H%M%S")
    name = pic_time + '.jpg'
    imgText = "Block-" + str(imgNum) + "/6"
    cv2.putText(img,imgText, (int((objClr.IMG_WD) - 200), int(40)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)       

    cv2.imwrite(str(objClr.img_dir_path) + str('/') + name, img)
    time.sleep(2)
    em.mail_snapshot("Block" + str(imgNum) +  " Pickedup Message!", pic_time)
    imgNum += 1
    time.sleep(1)
    print("\n\tSent the email")

##################### End of blockTransportEmail function #######################################

def turnAngleRequired(forNearDistance, objClr):
    print("\n\tturnAngleRequired(forNearDistance, objClr)")
    noBlockFound = False

    if forNearDistance == True:
        AngleTolerance = 2
    else:
        AngleTolerance = 2

    _, _, _, radius, _, _, angleToObject = objClr.detectObject(camera, rawCapture)  
    angle_desired = angleToObject
    cur_x_mod = mg.anglePerioidicity(mg.ReadIMUx(ser))
    new_x = cur_x_mod
    val = 70   
    if angle_desired < -2:            
        print("\n\tLeft turn inititated!")
        mg.left(val)
        while True:
            new_x = mg.anglePerioidicity(mg.ReadIMUx(ser))
            diff = abs(cur_x_mod - new_x)                    
            if (abs(cur_x_mod - new_x) >= abs(angle_desired) - AngleTolerance): # -2 is for inertia
                print("\n\tThe difference between current angle and new angle: ", diff)
                pwm3.stop(val)
                pwm1.stop(val)
                time.sleep(1)
                break              
    elif angle_desired > 2:
        print("\n\tRight turn inititated!")
        mg.right(val)
        while True:                
            new_x = mg.anglePerioidicity(mg.ReadIMUx(ser))
            diff = abs(cur_x_mod - new_x)                    
            if (abs(cur_x_mod - new_x) >= abs(angle_desired) - AngleTolerance): # -2 is for inertia
                print("\n\tThe difference between current angle and new angle: ", diff)
                pwm2.stop(val)
                pwm4.stop(val)
                time.sleep(1)
                # gameover()
                break
    else:
        pass
        # print("No turn left or right!")
    time.sleep(1)

    print("\n\tradius detected: ", radius)

    # cv2.waitKey(0)
    if radius < 2:
        print("\n\tradius detected: ", radius)
        print("\n\tBlock not found (may be toppled/ missed/not visible), coming out of descentAction!")
        noBlockFound = True
    
    # print("\n\tcoming out of turnAngleRequired")
    return noBlockFound
############################ End of turnAngleRequired function #####################################

def headingTurn(angle_tobeRotated):
    print("\n########## headingTurn(angle_tobeRotated) ##########")

    AngleTolerance = 2
    headingTurnFlag = False

    cur_x = mg.ReadIMUx(ser)
    # calcualte the smaller of the angle difference and find out which side is that smaller angle
    angle_desired = angle_tobeRotated - cur_x
    angle_desired = angle_desired - 360*math.floor(0.5 + (angle_desired/ 360))
    print("\n\tangle_desired: ", angle_desired)
    val = 60 

    if angle_desired < 0:            
        print("\n\tLeft turn inititated!")
        mg.left(val)
        while True:
            # keep on sensing new IMU angle, find out difference (smaller one) 
            # with needed IMU angle, rotate until difference is within tolerance.
            #  -2 is to account for inertia of rotation
            new_x = mg.ReadIMUx(ser)

            diff = (angle_tobeRotated - new_x)  
            diff = diff - 360*math.floor(0.5 + (diff/ 360))                  
            if (abs(diff) <= AngleTolerance): # -2 is for inertia
                pwm3.stop(val)
                pwm1.stop(val)
                time.sleep(1)
                cur_x = mg.ReadIMUx(ser)
                # print("\n Current Heading: ", cur_x)
                headingTurnFlag = True
                break              
    elif angle_desired > 0:
        print("\n\tRight turn inititated!")
        mg.right(val)
        while True:                
            new_x = mg.ReadIMUx(ser)
            diff = (angle_tobeRotated - new_x)  
            diff = diff - 360*math.floor(0.5 + (diff/ 360))                  
            if (abs(diff) <= AngleTolerance): # -2 is for inertia
                pwm2.stop(val)
                pwm4.stop(val)
                time.sleep(1)
                cur_x = mg.ReadIMUx(ser)
                # print("\n\tCurrent Heading: ", cur_x)
                headingTurnFlag = True
                # gameover()
                break
    else:
        headingTurnFlag = True
        pass
        # print("No turn left or right!")
    
    return headingTurnFlag

############################ End of headingTurn function #####################################

def reachBlock(objClr):
    print("\n########## reachBlock(objClr) ##########")
    img, x, y, radius, objHtPx, distanceToObject, angleToObject = objClr.detectObject(camera, rawCapture)
    distance = distanceToObject - objClr.CAMERA_TO_GRPROPENING_DIS
    print("\n\tReachingBlock Distance: ", distance)
    ###########################################################
    if distance > (1000):
        print("\n\tReachBlock Distance found as > 1000, which is :", distance)
        
        # hops = int((distance) / mg.HOP_DIST)
        # print("\nNumber of hops: ", hops)
        countReachBlock = 1

        while distance > 1000:
            # Move forward in hops
            print("\n\tMoving until distance is < 1000 , ", "\n\tCounter: ", countReachBlock, "\n\tMoving distance: ", (distance / 2))
            flTicks, brTicks= mg.forward((distance / 2), ser)
            cur_x = mg.ReadIMUx(ser)
            dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
            x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
            y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
            list_x.append(x_coor)
            list_y.append(y_coor)
            list_angle.append(cur_x)
            
            time.sleep(1)
            #
            noBlk = turnAngleRequired(forNearDistance=False, objClr=objClr) 
            #
            img, x, y, radius, objHtPx, distanceToObject, angleToObject = objClr.detectObject(camera, rawCapture)
            distance = distanceToObject - objClr.CAMERA_TO_GRPROPENING_DIS
            print("\n\tNew distance after moving: ", distance)
            #
            countReachBlock += 1            
        print("\n\tCompleted ReachBLock distance!!")
        print("\n\tnNow the distance shall be: 501 < distance <= 1000")
        print("\n\tNew distance after completing ReachBlock: ", distance)
############################ End of reachBlock function #####################################   

def descentAction(objClr): 
    print("\n########## descentAction(objClr) ##########") 
    # Turn angle - required
    noBlk = turnAngleRequired(forNearDistance=True, objClr=objClr) 

    ############## Now calculate distance and divide it in 6 parts    
    _, _, _, radius, _, distanceToObject, angleToObject = objClr.detectObject(camera, rawCapture)
    distance = distanceToObject - objClr.CAMERA_TO_GRPROPENING_DIS
    
    print("\n\tdescentAction distance calculated is: ", distance)    

    descentDist = int(distance / 6)
    print("\n\tdescending by step-distance: ", descentDist)

    counterDescent = 1
    permissibleAngle = 2
    while radius > 5:
        print("\n\tTravelling descent - " + str(counterDescent) + "/6", descentDist)
        flTicks, brTicks= mg.forward(descentDist, ser)
        _, _, _, radius, _, _, angleToObject = objClr.detectObject(camera, rawCapture)  
        angle_desired = angleToObject
        cur_x_mod = mg.anglePerioidicity(mg.ReadIMUx(ser))
        new_x = cur_x
        val = 50
        if angle_desired < - permissibleAngle:            
            mg.left(val)
            while True:   
                new_x = mg.anglePerioidicity(mg.ReadIMUx(ser))                  
                if (abs(cur_x_mod - new_x) >= abs(angle_desired)):
                    pwm3.stop(val)
                    pwm1.stop(val)
                    time.sleep(1)
                    break                  

        elif angle_desired > permissibleAngle:
            mg.right(val)
            while True:       
                new_x = mg.ReadIMUx(ser)
                new_x = mg.anglePerioidicity(new_x)                  
                if (abs(cur_x_mod - new_x) >= abs(angle_desired)):
                    pwm2.stop(val)
                    pwm4.stop(val)
                    time.sleep(1)
                    break             
        else:
            pass
        
        # print("\nradius after step ", counterDescent, ": ", radius)

        if counterDescent == 2:
            permissibleAngle = 1
            print("\n\tPreposing gripper for picking up!")
            duty = mg.ServoControl("full_open")            
        
        if counterDescent == 5:
            print("\n\tCounter reached 5, coming out of descentAction!")
            break
        
        if radius < 10:
            print("\n\tradius detected: ", radius)
            print("\n\tBlock not found (may be toppled/ missed), coming out of descentAction!")
            break

        
        counterDescent += 1
        
    print("\n\tdescentAction complete") 
    return descentDist
############################ End of descentAction function ##################################### 
def checkNpick(descentDist, objClr):  
    print("\n########## checkNpick(descentDist, objClr) ##########") 
    lastPushOffset = 15
    # print("\n moving to last push ", int(descentDist - lastPushOffset))
    # print("\n\tTravelling descent - 6/6")
    # flTicks, brTicks= mg.forward(int(descentDist - lastPushOffset), ser)             
    _, _, _, _, radius, _, _ = objClr.detectObject(camera, rawCapture)
    # print("\n radius after last push (step 5): ", radius)
    if radius > objClr.blockPickupConfirmRadius:
        flTicks, brTicks= mg.forward(descentDist, ser)
        time.sleep(1)
        # Move the gripper to Full- Closed position (2 sec)
        duty = mg.ServoControl("pickup")
        _, _, _, _, radius, _, _ = objClr.detectObject(camera, rawCapture)
        if radius > objClr.blockPickedupRadius:
            print("\n\tPicked the Block :)")
            blockTransportEmail(objClr) 
            pickupFlag = True
            print("\n\tradius after pickup: ", radius)                         

        else:
            print("\n\tMissed the block!! :(")
            flTicks, brTicks= mg.reverse(descentDist * 2, ser)
            pickupFlag = False
    else:
        print("\n\tMissed the block!! :(")
        flTicks, brTicks= mg.reverse(descentDist * 2, ser)
        pickupFlag = False
    
    print("\n\tPickedup Flag: ", pickupFlag)
    return pickupFlag
############################ End of checkNpick function #####################################
def reachHdg180Wall():
    print("\n########## reachHdg180Wall() ##########")
    avgObsDist, safeDistFlg = ca.AvgDistance(4)
    print("\n\tSensed distance to 180 wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)

    # if avgObsDist > (1000): # if distance is more than a meter, go in hops while correcting heading
    #     approachHdg180WallDist = avgObsDist - 1000
    #     hops = 2
    #     print("\nMoving approachHdg180WallDist ", int(approachHdg180WallDist))
    #     for i in range(hops):        
    #         # Move forward in hops
    #         flTicks, brTicks= mg.forward(int(approachHdg180WallDist / 2), ser)
    #         cur_x = mg.ReadIMUx(ser)
    #         dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
    #         x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
    #         y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
    #         list_x.append(x_coor)
    #         list_y.append(y_coor)
    #         list_angle.append(cur_x)
    #         
    #         time.sleep(1)
    #         headingTurnFlag = headingTurn(180)

    # avgObsDist, safeDistFlg = ca.AvgDistance(4)
    # print("\n[test] Sonar: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)

    while True:
        if avgObsDist < ca.SAFE_DIST:
            print("\n\tNoted distance before stopping before the wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)
            # cv2.waitKey(0)
            break
        # Move forward to safe distance
        flTicks, brTicks= mg.forward(int(mg.HOP_DIST), ser)
        time.sleep(1)
        headingTurnFlag = headingTurn(180)

        # calculate distance before the wall
        avgObsDist, safeDistFlg = ca.AvgDistance(4)
        print("\n\tDistance before the wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)



############################ End of reachHdg180Wall function #####################################

def reachConstrZn():
    print("\n########## reachConstrZn() ##########")
    avgObsDist, safeDistFlg = ca.AvgDistance(4)
    print("\n\tSensed distance to 270 wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)

    # if avgObsDist > (1219 + 100): # block can be detected as obstacle
    # # which can falsely convey that wall is 1 ft ahead
    #     approachConstrZnDist = avgObsDist - 1219
    #     hops = 2
    #     print("\nMoving approachConstrZnDist ", int(approachConstrZnDist))
    #     for i in range(hops):        
    #         # Move forward in hops
    #         flTicks, brTicks= mg.forward(int(approachConstrZnDist / 2), ser)
    #         cur_x = mg.ReadIMUx(ser)
    #         dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
    #         x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
    #         y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
    #         list_x.append(x_coor)
    #         list_y.append(y_coor)
    #         list_angle.append(cur_x)
    #         
    #         time.sleep(1)
    #         headingTurnFlag = headingTurn(270)
        
    #     # Move forward  2 feet into the contruction zone
    #     flTicks, brTicks= mg.forward(int(610), ser)
    #     cur_x = mg.ReadIMUx(ser)
    #     dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
    #     x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
    #     y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
    #     list_x.append(x_coor)
    #     list_y.append(y_coor)
    #     list_angle.append(cur_x)
    #     
    #     time.sleep(1)
    #     headingTurnFlag = headingTurn(270)

    while True:
        if avgObsDist < (ca.SAFE_DIST) :
            print("\n\tNoted distance before stopping before the wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)
            # cv2.waitKey(0)
            break
        # Move forward to safe distance
        flTicks, brTicks= mg.forward(int(mg.HOP_DIST), ser)
        time.sleep(1)
        headingTurnFlag = headingTurn(270)

        # calculate distance before the wall
        avgObsDist, safeDistFlg = ca.AvgDistance(4)
        print("\n\tDistance before the wall: ", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)

############################ End of reachConstrZn function #####################################
def checkNdrop(objClr):
    print("\n########## checkNdrop(objClr) ##########")
    global countercheckNdrop
    global imgNum
    avgObsDist, safeDistFlg = ca.AvgDistance(4)
    print("\n\tDistance before the wall", avgObsDist, "\tSafe Distance Flag: ", safeDistFlg)

    # Move forward to safe distance
    if (avgObsDist - ca.SAFE_DIST) > 10:
        flTicks, brTicks= mg.forward(int(avgObsDist - ca.SAFE_DIST), ser)
        cur_x = mg.ReadIMUx(ser)
        dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
        x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
        y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
        list_x.append(x_coor)
        list_y.append(y_coor)
        list_angle.append(cur_x)
        
        time.sleep(1)

    # Turn again to 210 heading and keep every block at an angle of 10 deg separation
    hdgAngle = 225
    print("\n\tcountercheckNdrop", countercheckNdrop)
    print("\n\tHeading angle before placing block ", hdgAngle)
    headingTurnFlag = headingTurn(hdgAngle)
    # traveling a small distance to place the block
    flTicks, brTicks= mg.forward(int(60), ser)
    time.sleep(1)

    # Turn again to 210 heading and keep every block at an angle of 10 deg separation
    # hdgAngle = 210 + (countercheckNdrop * 10)
    # print("\n countercheckNdrop", countercheckNdrop)
    # print("\n Heading angle before placing block ", hdgAngle)
    # headingTurnFlag = headingTurn(hdgAngle)
    # # traveling a small distance to place the block
    # if hdgAngle <= 275:
    #     flTicks, brTicks= mg.forward((int(60) * math.cos(math.radians(countercheckNdrop * 10))), ser)
    # else:
    #     flTicks, brTicks= mg.forward((int(60) * math.cos(math.radians( (countercheckNdrop  - 6) * 10))), ser)

    # Move the gripper to Full- open position (2 sec)
    # print("\n partially opening..")
    duty = mg.ServoControl("partial_open")

    # Move the gripper to Full- open position (2 sec)
    print("\n\treleasing down..")
    duty = mg.ServoControl("full_open")
    time.sleep(2)

    # Move reverse - back off
    flTicks, brTicks= mg.reverse(60, ser)
    cur_x = mg.ReadIMUx(ser)
    dActual  = ((flTicks + brTicks) / 2) * (2 * math.pi * mg.WH_RADIUS) / 960
    x_coor = round(dActual * math.sin(math.radians(cur_x)), 3)
    y_coor = round(dActual * math.cos(math.radians(cur_x)), 3)
    list_x.append(x_coor)
    list_y.append(y_coor)
    list_angle.append(cur_x)
    
    time.sleep(1)

    # blockTransportEmail(objClr)    

    # Move the gripper to Full- Closed position (2 sec)
    time.sleep(1)
    print("\n\tclosing up..")
    duty = mg.ServoControl("full_closed")
    time.sleep(2)

    # Turn again to 0 heading
    headingTurnFlag = headingTurn(270)
    headingTurnFlag = headingTurn(0)
    headingTurnFlag = headingTurn(45)
    if headingTurnFlag == False:
        print("\n\tRetrying heading turn 45")
        headingTurnFlag = headingTurn(45)
        if headingTurnFlag == False:
            print("\n\t2nd Retrying heading turn 45 failed, aborting!!")
            # cv2.destroyAllWindows()
            # mg.gameover()
        else:
            print("\n\theading turn 45 success(2nd attempt)")
    else:
        print("\n\theading turn 45 success")
    
    countercheckNdrop += 1

############################ End of checkNdrop function #####################################
def noBlockFoundAction(noBlk):
    print("\n########## noBlockFoundAction(noBlk) ##########")
    moveToAltHdg = False
    if noBlk == True:
        global imgNum
        if imgNum == 1: # variable to find out the block number
            headingTurnFlag = headingTurn(315)

        else:
            headingTurnFlag = headingTurn(0)

############################ End of noBlockFoundAction function #####################################

def detectBlock(objClr):
    print("\n########## detectBlock() ##########")
    moveToAltHdg = False
    print("\n\tDetecting the block ....")
    img, x, y, radius, objHtPx, distanceToObject, angleToObject = objClr.detectObject(camera, rawCapture)
    
    if radius < 2:
        print("\n\tradius detected: ", radius)
        print("\n\tNo block detected at current heading!!")
        print("\n\tMoving to alternate heading!!")      
        moveToAltHdg = True  
    else:
        print("\n\tDetected the block, turning to block now!!")
        print("\n\tOrigin: ", (x,y), "\n\tradius:", radius, "\n\tObject Height (px): ", objHtPx, "\n\tDistance to Object: ", distanceToObject, "\n\tAngle to Object: ", angleToObject)
        moveToAltHdg = False
    
    return moveToAltHdg
############################ End of detectBlock function #####################################

# Red Block Function
def pursueRed():    
    print("\n************** pursueRed() *****************")
    print("\nPursuing Block number: ", imgNum, "/9 is completed!")
    ############################ Actual Operation #####################################
    global cr
    while True:
        try:
            # Initial x,y co-ordinates, angle
            cur_x = 0
            cur_x = mg.ReadIMUx(ser)
            prev_angle = cur_x
            x_coor = 0
            y_coor = 0
            list_x.append(x_coor)
            list_y.append(y_coor)
            list_angle.append(cur_x)
            # print("Initial Waypoint (x,y,theta): ", (x_coor, y_coor, cur_x))

            # Detect the block
            moveToAltHdg = detectBlock(cr)

            # if block not found, move to Alteernate heading          
            if moveToAltHdg == True:
                noBlockFoundAction(moveToAltHdg)
                moveToAltHdg = detectBlock(cr)     
                if moveToAltHdg == True:
                    print("\nBlock not detected even after moving to alternate heading!!")    
                    print("\nAborting pursuit of this block!")
                else:
                    print("\nFound the block after moving to alternate heading!")
            else:
                    print("\nFound the block in the current heading!")

            # Turn angle - required
            noBlk = turnAngleRequired(forNearDistance=False, objClr=cr)           

            # Reach the block, while correcting its heading every specified distance
            reachBlock(cr)

            ############## Before 1 Hop + last mile distance (80 cm)
            descentDist = descentAction(cr)

            ############## pick the block - Final check before 15 mm
            pickupFlag = checkNpick(descentDist, cr)
            
            ############## if block is missed, give another try
            if pickupFlag == False:
                print("\nGiving another try to pickup the block!")
                descentDist = descentAction(cr)
                pickupFlag = checkNpick(descentDist, cr)  
                if pickupFlag == False:
                    # Move the gripper to Full- Closed position (2 sec)
                    duty = mg.ServoControl("full_closed")

            ############## turn 180 heading
            headingTurnFlag = headingTurn(180)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 180")
                headingTurnFlag = headingTurn(180)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 180 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 180 success(2nd attempt)")
            else:
                print("\n heading turn 180 success")
            
            ############## reach to 180 heading wall
            reachHdg180Wall()

            ############## turn 270 heading
            headingTurnFlag = headingTurn(270)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 270")
                headingTurnFlag = headingTurn(270)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 270 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 270 success(2nd attempt)")
            else:
                print("\n heading turn 270 success")
            
            ############## reach construction Zone
            reachConstrZn()
            ############## drop the block in the right place, email, turn to 0 heading
            checkNdrop(cr)            

            break
        except Exception as e:
            print("Error Handling:", e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print("\nLine number: ", exc_tb.tb_lineno)
            # gpio.cleanup()
            break

    print("\nRed block Process Done:")
    print("\nPursuing Block number: ", (imgNum-1), "/9 is completed!")
    # Stope Servo
    # pwmS.stop()
    # Stope Servo
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()

############################ End of pursueRed function #####################################
# Green Block Function
def pursueGreen():  
    print("\n************** pursueGreen() *****************")
    print("\nPursuing Block number: ", imgNum, "/9........")
    ############################ Actual Operation #####################################
    global cg
    while True:
        try:
            # Initial x,y co-ordinates, angle
            cur_x = 0
            cur_x = mg.ReadIMUx(ser)
            prev_angle = cur_x
            x_coor = 0
            y_coor = 0
            list_x.append(x_coor)
            list_y.append(y_coor)
            list_angle.append(cur_x)
            # print("Initial Waypoint (x,y,theta): ", (x_coor, y_coor, cur_x))

            # Detect the block
            moveToAltHdg = detectBlock(cg)

            # if block not found, move to Alteernate heading          
            if moveToAltHdg == True:
                noBlockFoundAction(moveToAltHdg)
                moveToAltHdg = detectBlock(cg)     
                if moveToAltHdg == True:
                    print("\nBlock not detected even after moving to alternate heading!!")    
                    print("Aborting pursuit of this block!")
                else:
                    print("\nFound the block after moving to alternate heading!")
            else:
                    print("\nFound the block in the current heading!")

            # Turn angle - required
            noBlk = turnAngleRequired(forNearDistance=False, objClr=cg)           

            # Reach the block, while correcting its heading every specified distance
            reachBlock(cg)

            ############## Before 1 Hop + last mile distance (80 cm)
            descentDist = descentAction(cg)

            ############## pick the block - Final check before 15 mm
            pickupFlag = checkNpick(descentDist, cg)
            
            ############## if block is missed, give another try
            if pickupFlag == False:
                print("\nGiving another try to pickup the block!")
                descentDist = descentAction(cg)
                pickupFlag = checkNpick(descentDist, cg)  
                if pickupFlag == False:
                    # Move the gripper to Full- Closed position (2 sec)
                    duty = mg.ServoControl("full_closed")

            ############## turn 180 heading
            headingTurnFlag = headingTurn(180)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 180")
                headingTurnFlag = headingTurn(180)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 180 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 180 success(2nd attempt)")
            else:
                print("\n heading turn 180 success")
            
            ############## reach to 180 heading wall
            reachHdg180Wall()

            ############## turn 270 heading
            headingTurnFlag = headingTurn(270)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 270")
                headingTurnFlag = headingTurn(270)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 270 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 270 success(2nd attempt)")
            else:
                print("\n heading turn 270 success")
            
            ############## reach construction Zone
            reachConstrZn()
            ############## drop the block in the right place, email, turn to 0 heading
            checkNdrop(cg)            

            break
        except Exception as e:
            print("Error Handling:", e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print("\nLine number: ", exc_tb.tb_lineno)
            # gpio.cleanup()
            break

    print("\nGreen block Process Done:")
    print("\nPursuing Block number: ", (imgNum-1), "/9 is completed!")
    # Stope Servo
    # pwmS.stop()
    # Stope Servo
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()

############################ End of pursueGreen function #####################################
# Blue Block Function
def pursueBlue():  
    print("\n************** pursueBlue() *****************")
    print("\nPursuing Block number: ", imgNum, "/9....")

    ############################ Actual Operation #####################################
    global cb
    while True:
        try:
            # Initial x,y co-ordinates, angle
            cur_x = 0
            cur_x = mg.ReadIMUx(ser)
            prev_angle = cur_x
            x_coor = 0
            y_coor = 0
            list_x.append(x_coor)
            list_y.append(y_coor)
            list_angle.append(cur_x)
            # print("Initial Waypoint (x,y,theta): ", (x_coor, y_coor, cur_x))

            # Detect the block
            moveToAltHdg = detectBlock(cb)

            # if block not found, move to Alteernate heading          
            if moveToAltHdg == True:
                noBlockFoundAction(moveToAltHdg)
                moveToAltHdg = detectBlock(cb)     
                if moveToAltHdg == True:
                    print("\nBlock not detected even after moving to alternate heading!!")    
                    print("Aborting pursuit of this block!")
                else:
                    print("\nFound the block after moving to alternate heading!")
            else:
                    print("\nFound the block in the current heading!")

            # Turn angle - required
            noBlk = turnAngleRequired(forNearDistance=False, objClr=cb)           

            # Reach the block, while correcting its heading every specified distance
            reachBlock(cb)

            ############## Before 1 Hop + last mile distance (80 cm)
            descentDist = descentAction(cb)

            ############## pick the block - Final check before 15 mm
            pickupFlag = checkNpick(descentDist, cb)
            
            ############## if block is missed, give another try
            if pickupFlag == False:
                print("\nGiving another try to pickup the block!")
                descentDist = descentAction(cb)
                pickupFlag = checkNpick(descentDist, cb)  
                if pickupFlag == False:
                    # Move the gripper to Full- Closed position (2 sec)
                    duty = mg.ServoControl("full_closed")

            ############## turn 180 heading
            headingTurnFlag = headingTurn(180)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 180")
                headingTurnFlag = headingTurn(180)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 180 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 180 success(2nd attempt)")
            else:
                print("\n heading turn 180 success")
            
            ############## reach to 180 heading wall
            reachHdg180Wall()

            ############## turn 270 heading
            headingTurnFlag = headingTurn(270)
            if headingTurnFlag == False:
                print("\n Retrying heading turn 270")
                headingTurnFlag = headingTurn(270)
                if headingTurnFlag == False:
                    print("\n 2nd Retrying heading turn 270 failed, aborting!!")
                    break
                else:
                    print("\n heading turn 270 success(2nd attempt)")
            else:
                print("\n heading turn 270 success")
            
            ############## reach construction Zone
            reachConstrZn()
            ############## drop the block in the right place, email, turn to 0 heading
            checkNdrop(cb)            

            break
        except Exception as e:
            print("Error Handling:", e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print("\nLine number: ", exc_tb.tb_lineno)
            # gpio.cleanup()
            break

    print("\nBlue block Process Done:")
    print("\nPursuing Block number: ", (imgNum-1), "/9 is completed!")
    # Stop Servo
    # pwmS.stop()
    # Stop Servo
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()

############################ End of pursueBlue function #####################################

# Driver Program
if __name__ == "__main__":
    # autoOp = bool(int(input("\nStart Auto Op? (0/1)")))
    # print("\nautoOp", autoOp)
    while True:
        pursueRed()
        pursueGreen()
        pursueBlue()

        pursueRed()
        pursueGreen()
        pursueBlue()

        pursueRed()
        pursueGreen()
        pursueBlue()
				

        break
    
    # Closure
    pwmS.stop()
    cv2.destroyAllWindows()
    mg.gameover()
	