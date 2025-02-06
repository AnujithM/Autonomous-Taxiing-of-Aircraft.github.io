# -*- coding: utf-8 -*-
"""
Created on Wed Feb  8 18:15:22 2023

@author: i3d_m
"""

import numpy as np
import cv2, os

import tensorflow as tf
import pickle
from PIL import Image
import socket

def preprocess(img, height, width):
    shape = img.shape
    og_img = img
    img = cv2.resize(img, (width, height))
    img = img[None, :, :, :]
    # label = cv2.imread(label, cv2.IMREAD_GRAYSCALE)
    
    # label = cv2.resize(label, (width, height))
    # label = label.astype(np.uint8)
    return img, shape
def resizeNearest(img, shape):
    img = Image.fromarray(img)
    img = img.resize(shape, resample=Image.NEAREST)
    img = np.array(img)
    return img
def obj(img, model1, colorDict, labelColorDict, rect_width=150, rect_height=180):
    
    height = img.shape[1]
    width = img.shape[2]
   
    prediction = model1.predict(img)[0]
    prediction = np.apply_along_axis(np.argmax, 2, prediction).astype(np.uint8)
    mask = np.zeros(prediction.shape).astype(np.uint8)
    # cv2.rectangle(mask, (int((width - rect_width)/2), height - rect_height), (int((width - rect_width)/2)+rect_width, height), 255, -1)
    cv2.rectangle(mask, (40 , 0), (180,224), 255, -1)
    prediction = cv2.bitwise_and(prediction, prediction, mask=mask)
    predictionColor = cv2.bitwise_and(prediction, prediction, mask=mask)
    count_obj = np.unique(prediction)
    pixel_count = np.sum(prediction > 0)
    predictionColor = np.zeros((*prediction.shape, 3)).astype(np.uint8)
    for i in range(height):
        for j in range(width):
            predictionColor[i,j,:] = labelColorDict[prediction[i, j]][::-1]
    predictionColor = resizeNearest(predictionColor, (width, height))
    # cv2.rectangle(predictionColor, (40 , 0), (180,224), 255, 1)
    
    return predictionColor ,count_obj, pixel_count

def importObjModel():
    model1 = tf.keras.models.load_model(r"resources/weights/best_weights1.h5")
    with open(r"resources\meta_data\new_meta_data\6_classes/colorDict.pickle", "rb") as f:
        colorDict = pickle.load(f)
    
    with open(r"resources\meta_data\new_meta_data\6_classes/labelColorDict.pickle", "rb") as f:
        labelColorDict = pickle.load(f)
    return model1, colorDict, labelColorDict
if __name__ == "__main__":
    height = 1080
    width = 1920
    model1, colorDict, labelColorDict = importObjModel()
    cap = cv2.VideoCapture(1)
    cap.set(3, width)
    cap.set(4, height)
    font = cv2.FONT_HERSHEY_SIMPLEX
    while(True):
        ret, frame = cap.read()
        if not ret:
            break
        # frame = resizeNearest(frame, (1920,1080))
        # img, img_shape = preprocess(frame, 224, 224)

        # prediction = model1.predict(img)[0]
        # prediction = np.apply_along_axis(np.argmax, 2, prediction).astype(np.uint8)
        # predictionColor = np.zeros((*prediction.shape, 3)).astype(np.uint8)
        # for i in range(224):
        #     for j in range(224):
        #         predictionColor[i,j,:] = labelColorDict[prediction[i, j]]
        # predictionColor = resizeNearest(predictionColor, img_shape[:2][::-1])
        
        # result = cv2.addWeighted(frame, 0.7, predictionColor, 1, 0)
   
        img_obj, img_obj_shape = preprocess(frame, 224, 224)
        
        img_obj, count_obj, pixel_count = obj(img_obj, model1, colorDict, labelColorDict)

        img_obj = cv2.resize(img_obj, (width, height))
        cv2.putText(img_obj, 'Pixel Count:' + str(pixel_count), (int((width / 2) + 50), int((height / 2) + 200)), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        print(frame.shape, img_obj.shape)
        result = cv2.addWeighted(frame, 0.7, img_obj, 1, 0)
        cv2.imshow("frame", result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # After the loop release the cap object
    cap.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
        
        
        