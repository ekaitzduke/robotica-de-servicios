import cv2 as cv
import mediapipe as mp
import numpy as np


# Gets hand_landmarks coordinates ((x,y) for each on a list) given a screen (dimensions width,height) and
# a mediapipe hand_landmark detection (hand_landmarks) and only the index desired on a list (point_lists)
def getCoordinates (width, height, points_list, hand_landmarks): 
    coordinates = [] 
    for i in points_list: 
        xthumb=int(hand_landmarks.landmark[i].x*width) 
        ythumb=int(hand_landmarks.landmark[i].y*height) 
        coordinates.append([xthumb,ythumb]) 
    
    return coordinates 

def getTriangle (coordinates_thumb): 
    p1 = np.array(coordinates_thumb[0]) 
    p2 = np.array(coordinates_thumb[1]) 
    p3 = np.array(coordinates_thumb[2]) 
    l1 = np.linalg.norm(p2-p3) 
    l2 = np.linalg.norm(p1-p3) 
    l3 = np.linalg.norm(p1-p2) 
    return p1, p2, p3, l1, l2, l3 

def palmCentroid (coordinates_list): 
    coordinates = np.array(coordinates_list) 
    centroid = np.mean(coordinates, axis=0) 
    centroid = int(centroid[0]), int(centroid[1]) 
    return centroid