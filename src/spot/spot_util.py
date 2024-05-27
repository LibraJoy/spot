import spot_spot as spot
import cv2

def show():
    img = spot.ptzImage()
    cv2.imshow('ptzCamera', img)
    cv2.waitKey()
    
