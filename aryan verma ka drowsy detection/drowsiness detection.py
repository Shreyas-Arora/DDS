#Importing OpenCV Library for basic image processing functions
import cv2
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
#face_utils for basic operations of conversion
from imutils import face_utils
import threading
import time
import os
from scipy.spatial import distance as dist
from imutils.video import VideoStream

def alarm(msg):
    global alarm_status
    global alarm_status2
    global saying

    while alarm_status:
        print('call')
        s = 'espeak "'+msg+'"'
        os.system(s)

    if alarm_status2:
        print('call')
        saying = True
        s = 'espeak "' + msg + '"'
        os.system(s)
        saying = False


#Initializing the camera and taking the instance
cap = cv2.VideoCapture(0)

def change_res(width, height):
    cap.set(3, width)
    cap.set(4, height)

#Initializing the face detector and landmark detector
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#status marking for current state
sleep = 0
drowsy = 0
active = 0
status=""
color=(0,0,0)
alarm_status = False
alarm_status2 = False
alarm_status3 = False
saying = False

def compute(ptA,ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a,b,c,d,e,f):
    up = compute(b,d) + compute(c,e)
    down = compute(a,f)
    ratio = up/(2.0*down)

    #Checking if it is blinked
    if(ratio>0.22):
        return 2
    elif(ratio>0.165 and ratio<=0.22):
        return 1
    else:
        return 0





change_res(400, 350)

while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    #face_frame = frame.copy()
    
    #detected face in faces array
   


    for face in faces:
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()

        
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        landmarks = predictor(gray, face)
        landmarks = face_utils.shape_to_np(landmarks)
      


        #The numbers are actually the landmarks which will show eye
        left_blink = blinked(landmarks[36],landmarks[37], 
            landmarks[38], landmarks[41], landmarks[40], landmarks[39])
        right_blink = blinked(landmarks[42],landmarks[43], 
            landmarks[44], landmarks[47], landmarks[46], landmarks[45])
        #head = [landmarks[21], landmarks[22]]

        
        #Now judge what to do for the eye blinks
        if(left_blink==0 or right_blink==0):
            sleep+=1
            drowsy=0
            active=0
            yawn=0
            if(sleep>6):
                status="SLEEPING"
                color = (255,0,0)
                if alarm_status == False:
                    alarm_status = True
                    t = threading.Thread(target=alarm, args=('wake up',))
                    t.deamon = True
                    t.start()
            else:
                alarm_status = False
                

        elif(left_blink==1 or right_blink==1):
            sleep=0
            active=0
            drowsy+=1
            yawn=0
            if(drowsy>6):
                status="DROWSY"
                color = (0,0,255)
                if alarm_status2 == False and saying == False:
                    alarm_status2 = True
                    t = threading.Thread(target=alarm, args=('take some fresh air',))
                    t.deamon = True
                    t.start()
            else:
                alarm_status3 = False


        else:
            drowsy=0
            sleep=0
            active+=1
            yawn=0
            if(active>6):
                status="AWAKE AND ALRIGHT"
                color = (0,255,0)

       
            
        cv2.putText(frame, status, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color,2)

        for n in range(0, 68):
            (x,y) = landmarks[n]
            cv2.circle(frame, (x, y), 1, (255, 255, 255), -1)

    #cv2.imshow("Frame", frame)
    cv2.imshow("Result of detector", frame)
    key = cv2.waitKey(1)
    if key == 27:
          break
