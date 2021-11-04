import cv2
import numpy as np
import time
import argparse
import imutils
from threading import Thread
from imutils.video import WebcamVideoStream
from imutils.video import FPS
from typing import ItemsView
import serial
import numpy as np
import math
from Serial_cmd import Serial_cmd



# initialize data
ser = Serial_cmd()

def pixels_to_angle(num_pixels):
    fov = 90
    width = 640
    angle = num_pixels/width*fov - fov/2
    return int(angle)




net = cv2.dnn.readNet("./yolov3.weights", "../darknet/cfg/yolov3.cfg")
classes = []
with open("../darknet/data/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
print(net.getUnconnectedOutLayers())
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

#load input video stream
cap = cv2.VideoCapture(2)
#if not cap.isOpened():
#    print("Aw, frick")
cap.set(3, 640)
cap.set(4, 480)


#instantiate a variable 'p' to keep count of persons
p = 0  
#initialize the writer
writer = None
(W, H) = (None, None)
starting_time = time.time()
frame_id = 0
while True:
    #print("frame id:" + str(frame_id))
    res, frame = cap.read()
    #print("ret:" + str(ret))
    # print("frame:" + str(frame))
    frame_id += 1
    if W is None or H is None:
        (H, W) = frame.shape[:2]
    # Detecting objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    # initialize our lists of detected bounding boxes, confidences, and class IDs, respectively
    boxes = []
    confidences = []
    class_ids = []
    # loop over each of the layer outputs
    for out in outs:
        # loop over each of the detections
        for detection in out:
            # extract the class ID and confidence 
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            # filter out weak predictions
            if confidence > 0.7:
                center_x = int(detection[0] * W)
                center_y = int(detection[1] * H)
                w = int(detection[2] * W)
                h = int(detection[3] * H)
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
    
                # update our list of bounding box coordinates, confidences, and class IDs
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    # apply non-maxima suppression to suppress weak, overlapping bounding boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.2)

    people_centers = []
    if len(indexes) > 0:
        # loop over the indexes we are keeping
        for i in indexes.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            label = str(classes[class_ids[i]])
            if label == 'person':
                people_centers.append(x+(w/2))

    #detecting persons
    if len(indexes) > 0:
        # loop over the indexes we are keeping
        for i in indexes.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            label = str(classes[class_ids[i]])
            if label == 'person':
                p=p+1
            else:
                continue
            # draw a bounding box rectangle and label on the frame
            color = [int(c) for c in colors[class_ids[i]]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "Angle of Person" + '=' + str(pixels_to_angle(people_centers[0]))
            cv2.putText(frame, text, (x, y+30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    

    if len(people_centers) != 0:
        person_angle = pixels_to_angle(people_centers[0])
        print("Person angle: " + str(person_angle))
        ser.set_angle(person_angle)



    
    if writer is None:
        # initialize our video writer
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = cv2.VideoWriter("../low_to_ground_robo.mp4", fourcc, 30,(frame.shape[1], frame.shape[0]), True)
    
    elapsed_time = time.time() - starting_time
    fps = frame_id / elapsed_time
    print("fps: " + str(round(fps, 2)))
    cv2.imshow("Frame", frame)

    writer.write(frame)
    writer.write(frame)
    writer.write(frame)
    writer.write(frame)
    writer.write(frame)
    writer.write(frame)
    if cv2.waitKey(1) == 27:
        cap.release()
        writer.release()
        break
    #print(ser.read())

cv2.destroyAllWindows()


