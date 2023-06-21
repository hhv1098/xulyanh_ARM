import time
from PyQt5.QtCore import QTimer
import cv2
import numpy as np
import imutils


class Dectect_Object:
    def __init__(self):
        self.cap = cv2.VideoCapture(1)
        self.cap.set(3, 480)
        self.cap.set(4, 640)
        self.info = []
    def Dectec(self):
        while True:
            _, self.frame = self.cap.read()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([15, 50, 20])
            upper_yellow = np.array([35, 255, 255])

            lower_blue = np.array([90, 50, 20])
            upper_blue = np.array([130, 255, 255])

            lower_red = np.array([0, 100, 20])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 20])
            upper_red2 = np.array([179, 255, 255])

            mask = cv2.threshold(hsv, 100, 250, cv2.THRESH_BINARY_INV)
            mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow, cv2.THRESH_BINARY)
            mask2 = cv2.inRange(hsv, lower_blue, upper_blue, cv2.THRESH_BINARY)
            mask3 = cv2.inRange(hsv, lower_red, upper_red, cv2.THRESH_BINARY)
            mask4 = cv2.inRange(hsv, lower_red2, upper_red2, cv2.THRESH_BINARY)
            cnts = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            i = 0
            for c in cnts:

                area = cv2.contourArea(c)
                if (20000 > area > 14000):
                    i = i + 1
                    cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                    M = cv2.moments(c)
                    cx = int(M["m10"] / (M["m00"]))
                    cy = int(M["m01"] / (M["m00"]))
                    x_cm, y_cm = self.CV_P2cm(cx, cy)
                    cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.putText(self.frame, "blue:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                    cv2.putText(self.frame, str(int(i)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                    cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)

                    cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)
                    self.info.append(['blue', str(x_cm), str(y_cm)])

            cnts = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            for c in cnts:

                area = cv2.contourArea(c)
                if (20000 > area > 14000):
                    i = i + 1
                    cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                    M = cv2.moments(c)
                    cx = int(M["m10"] / (M["m00"]))
                    cy = int(M["m01"] / (M["m00"]))
                    x_cm, y_cm = self.CV_P2cm(cx, cy)
                    cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.putText(self.frame, "yellow:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                    cv2.putText(self.frame, str(int(i)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                    cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)

                    cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)
                    self.info.append(['yellow', str(x_cm), str(y_cm)])
            cnts = cv2.findContours(mask3 + mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            for c in cnts:

                area = cv2.contourArea(c)
                if (20000 > area > 12000):
                    i = i + 1
                    cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                    M = cv2.moments(c)
                    cx = int(M["m10"] / (M["m00"]))
                    cy = int(M["m01"] / (M["m00"]))
                    x_cm, y_cm = self.CV_P2cm(cx, cy)
                    cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.putText(self.frame, "Red:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                    cv2.putText(self.frame, str(int(i)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                    cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)

                    cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                1)
                    self.info.append(['red', str(x_cm), str(y_cm)])
            cv2.line(self.frame, (0, 0), (0, 480), (255, 0, 0), 2)
            cv2.line(self.frame, (0, 0), (640, 0), (255, 0, 0), 2)
            cv2.imshow("self.frame", self.frame)
            cv2.waitKey(1)
        self.cap.release()
        cv2.destroyAllWindows()
    def CV_P2cm(self, a, b):
        ratio_x = 23 / 640
        ratio_y = 17.3 / 480
        x_cm = round(a * ratio_x, 2)
        y_cm = round(b * ratio_y, 2)
        x_robot = round(-y_cm + 24, 2)
        y_robot = round(-x_cm + 12.4, 2)
        return x_robot, y_robot
Com = Dectect_Object()
infor =Com.Dectec()
print(infor)