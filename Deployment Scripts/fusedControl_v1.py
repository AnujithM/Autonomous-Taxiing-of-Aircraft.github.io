


# -*- coding: utf-8 -*-
"""
Created on unknown

@author: unknown
"""

from midLane_control import *
from lightControl import *
from objectControl import *
import pandas
import socket
import tensorflow as tf
import pickle
from PIL import Image
import cv2
connect=1
# with open('lane_log.csv', 'w') as f:
#     csv_f = csv.writer(f)
#     csv_f.writerow([0, 0, 0])
laneLog = pd.DataFrame(columns=["control", ["state"]])
if connect:
    UDP_IP = "192.168.0.35"#"192.168.43.69"
    UDP_PORT = 20001
    print(3)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(2)
    serverAddressPort = (UDP_IP, UDP_PORT)
    print(1)
    sock.sendto("Hi Server".encode('utf-8'), serverAddressPort)
    print('after sendto')
    data, addr = sock.recvfrom(1024)
    print(data.decode('utf-8'), "was recieved")
# def preprocess(img, height, width):
#     shape = img.shape
#     og_img = img
#     img = cv2.resize(img, (width, height))
#     img = img[None, :, :, :]
#     # label = cv2.imread(label, cv2.IMREAD_GRAYSCALE)
    
#     # label = cv2.resize(label, (width, height))
#     # label = label.astype(np.uint8)
#     return img, shape
# def resizeNearest(img, shape):
#     img = Image.fromarray(img)
#     img = img.resize(shape, resample=Image.NEAREST)
#     img = np.array(img)
#     return img
# def obj(img, model1, colorDict, labelColorDict):
    
#     height = img.shape[1]
#     width = img.shape[2]
#     prediction = model1.predict(img)[0]
#     prediction = np.apply_along_axis(np.argmax, 2, prediction).astype(np.uint8)
#     count_obj = np.unique(prediction)
#     predictionColor = np.zeros((*prediction.shape, 3)).astype(np.uint8)
#     for i in range(height):
#         for j in range(width):
#             predictionColor[i,j,:] = labelColorDict[prediction[i, j]]
#     predictionColor = resizeNearest(predictionColor, (width, height))
#     return predictionColor,count_obj

def getSwitch(frame, hsvlog):
    pass
def getGreen(img, hsvlog, rect_width=100, rect_height=100, lw=20, st_threshold=0):
    img_shape = img.shape[:2]
    rect_xpos = int((img_shape[0] - rect_width) / 2)
    rect_ypos = img_shape[1] - rect_height
    centredot_ypos = int(rect_ypos + rect_height / 2)
    mask1 = np.zeros(img_shape, dtype="uint8")
    cv2.rectangle(mask1, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, -1)
    kernel = np.ones((5, 5), np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowArray = list(hsvlog.iloc[len(hsvlog)-1, 1:4])
    highArray = list(hsvlog.iloc[len(hsvlog)-1, 4:])
    HSVLOW = np.array(lowArray).astype(np.uint8)
    HSVHIGH = np.array(highArray).astype(np.uint8)
    mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)
    
    mask = cv2.erode(mask, kernel, iterations=1)

    mask = cv2.dilate(mask, kernel, iterations=2)

    mask = cv2.bitwise_and(mask, mask, mask=mask1)
    control = int(np.sum(mask)/255)
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    # rect_centre = (int(rect_xpos + rect_width / 2), int(rect_ypos + rect_height / 2))
    # cv2.circle(mask, rect_centre, radius=1, color=(0,0,255), thickness=1)
    # cv2.rectangle(mask, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, 1)
    
    

        
    
    return mask, control
if __name__ == "__main__":
    onlyLane = 0
    cap = cv2.VideoCapture(0)
    cap.set(3, 1920)
    cap.set(4, 1080)
    hsvlog = pd.read_csv("hsvlog2.csv")
    font = cv2.FONT_HERSHEY_SIMPLEX
    width = 1920
    height = 1080
    model = importModel()
    if not onlyLane:
        model1, colorDict, labelColorDict = importObjModel()
    # model1 = tf.keras.models.load_model(r"resources/best_weights.h5")
    # with open("resources/meta_data/colorDict.pickle", "rb") as f:
    #     colorDict = pickle.load(f)

    # with open("resources/meta_data/labelColorDict.pickle", "rb") as f:
    #     labelColorDict = pickle.load(f)
    flag = 1
    stopControlThreshold = 100
    pred_height = 224
    pred_width = 224
    # out = cv2.VideoWriter("output_realtime_2.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25, (1280, 720))
    count = 0
    alpha = 0.7
    beta = 1 - alpha
    while (True):
        ret, frame = cap.read()
        img_segment = cv2.resize(frame, (pred_height, pred_width))
        # img_lane = cv2.resize(frame, (height, width))
        img_lane, img_lane_shape = preprocess(frame, pred_height, pred_width)
    
        mask, stopControl = getGreen(img_segment, hsvlog)
        if stopControl > stopControlThreshold:
            flag = 0
        if flag:
            
            lane_drawn, control, contours_no = get_midlane(model, img_lane, width, height)
            count = 0
            laneLog.loc[len(laneLog)] = [control, "Lane"]
            
        else:
            lane_drawn , control = getControl(img_segment, hsvlog, rect_width=img_segment.shape[0], rect_height=img_segment.shape[1])
            if control == "None":
                count += 1
            if count > 5:
                flag = 1
            laneLog.loc[len(laneLog)] = [control, "Light"]
        if not onlyLane:
            img_obj = obj(img_lane, model1, colorDict, labelColorDict)
            if img_obj[2] > 800:
                control = "Object"
      
        
        # img_obj, count_obj = obj(img_lane, model1, colorDict, labelColorDict)
        # # print(img_obj.shape)
        # img_obj = cv2.resize(img_obj, (1920, 1080))
       
        
        width = frame.shape[1]
        height = frame.shape[0]
        mask = cv2.resize(mask, frame.shape[:2][::-1])
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        lane_drawn = cv2.resize(lane_drawn, frame.shape[:2][::-1])
        if not onlyLane:
            obj_drawn = cv2.resize(img_obj[0], frame.shape[:2][::-1])
        if connect:
            sock.sendto(str(control).encode('utf-8'), serverAddressPort)
        
        # blanks = np.zeros_like(mask).astype(np.uint8)
        cv2.putText(mask, 'Pixel Count:' + str(img_obj[2]), (int((width / 2) + 50), int((height / 2))), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        cv2.putText(mask, 'Stop Control:' + str(stopControl), (int((width / 2) + 250), int((height / 2) + 200)), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        cv2.putText(mask, 'Control:' + str(control), (int((width / 2) + 250), int((height / 2) + 100)), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        
        result = cv2.addWeighted(frame, alpha, mask, beta, 0)
        result = cv2.addWeighted(result, alpha, lane_drawn, beta, 0)
        if not onlyLane:
            result = cv2.addWeighted(result, alpha, obj_drawn, beta, 0)
        cv2.imshow("mask", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # elif cv2.waitKey(1) & 0xFF == ord('s'):
        #     # result2 = cv2.addWeighted(lane_drawn, 0.5, obj_drawn, 0.5, 0)
        #     # result2 = cv2.addWeighted(frame, alpha, result2, beta, 0)
        #     # cv2.imwrite("fullImage.png", result2)
        #     resultLane = cv2.addWeighted(frame, alpha, lane_drawn, beta, 0)
        #     resultObj = cv2.addWeighted(frame, alpha, obj_drawn, beta, 0)
        #     cv2.imwrite("rawImage.png", frame)
        #     cv2.imwrite("laneMask.png", lane_drawn)
        #     cv2.imwrite("objMask.png", obj_drawn)
        #     cv2.imwrite("fullImage.png", cv2.addWeighted(resultLane, 0.6, resultObj, 0.6, 0))
    laneLog.to_csv("lane_log.csv")
    # After the loop release the cap object
    cap.release()
    # Destroy all the windows
    cv2.destroyAllWindows()