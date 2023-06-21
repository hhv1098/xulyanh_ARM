import math
import threading
import time
import cv2
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication, QMessageBox
import sys
from PyQt5.QtCore import QTimer

from qtpy import QtGui

from Gui import Ui_Control_Robot
import serial
import serial.tools.list_ports
import numpy as np
import cv2
import imutils
class Communication:
    def __init__(self):
        self.port = serial.Serial()
        self.uic = Ui_Control_Robot()
    def connection(self, port, baudrate):
        try:
            self.port.port = port
            self.port.baudrate = baudrate
            self.port.open()
        except:
            self.uic.Lb_Status.setText("Robot Status: Off")
            self.uic.Status_Light.setStyleSheet("background: rgb(255,0,0); border-radius:20px")
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Check your COM Port")
            msg.setWindowTitle("Error")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec()
    def send_data(self, buffer):
        try:
            self.port.write((buffer).encode("utf-8"))
            time.sleep(0.01)
            self.port.flush()
        except:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Check your COM Port")
            msg.setWindowTitle("Error")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec()

class MainWindow:
    def __init__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_Control_Robot()
        self.uic.setupUi(self.main_win)
        self.com = Communication()
        self.list_ports()
        self.info = []
        self.d1 = 12.9
        self.a2 = 14
        self.a3 = 14
        self.a4 = 5.5
        self.a5 = 2.3
        self.Px = 0
        self.Py = 0
        self.Pz = 0
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.flag = 1
        self.run = 0
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 480)
        self.cap.set(4, 640)
        self.flag_blue = False
        self.flag_yellow = False
        self.flag_red = False
        self.flag_all = False
        self.blue = 0
        self.yellow = 0
        self.red = 0

        self.uic.Btn_X_Dec.setDisabled(True)
        self.uic.Btn_Y_Dec.setDisabled(True)
        self.uic.Btn_Z_Dec.setDisabled(True)
        self.uic.Btn_X_Inc.setDisabled(True)
        self.uic.Btn_Y_Inc.setDisabled(True)
        self.uic.Btn_Z_Inc.setDisabled(True)
        self.uic.Btn_J1_Dec.setDisabled(True)
        self.uic.Btn_J2_Dec.setDisabled(True)
        self.uic.Btn_J3_Dec.setDisabled(True)
        self.uic.Btn_J1_Inc.setDisabled(True)
        self.uic.Btn_J2_Inc.setDisabled(True)
        self.uic.Btn_J3_Inc.setDisabled(True)
        self.uic.Btn_Move.setDisabled(True)
        self.uic.lb_B.setText('Blue: 0')
        self.uic.lb_Y.setText('Yellow: 0')
        self.uic.lb_R.setText('Red: 0')
        self.uic.lb_T.setText('Total: 0')

        #Pushbutton
        self.uic.Btn_Connect.clicked.connect(self.connect)
        self.uic.Btnsend.clicked.connect(self.send)
        self.uic.Bt_CalibJ1.clicked.connect(self.Calib_J1)
        self.uic.Bt_CalibJ2.clicked.connect(self.Calib_J2)
        self.uic.Bt_CalibJ3.clicked.connect(self.Calib_J3)
        self.uic.Btn_Tool.clicked.connect(self.Tool)
        self.uic.Btn_Home.clicked.connect(self.Home)
        self.uic.Btn_J1_Dec.clicked.connect(self.Btn_J1_Dec)
        self.uic.Btn_J2_Dec.clicked.connect(self.Btn_J2_Dec)
        self.uic.Btn_J3_Dec.clicked.connect(self.Btn_J3_Dec)
        self.uic.Btn_J1_Inc.clicked.connect(self.Btn_J1_Inc)
        self.uic.Btn_J2_Inc.clicked.connect(self.Btn_J2_Inc)
        self.uic.Btn_J3_Inc.clicked.connect(self.Btn_J3_Inc)
        self.uic.Btn_X_Dec.clicked.connect(self.Btn_X_Dec)
        self.uic.Btn_Y_Dec.clicked.connect(self.Btn_Y_Dec)
        self.uic.Btn_Z_Dec.clicked.connect(self.Btn_Z_Dec)
        self.uic.Btn_X_Inc.clicked.connect(self.Btn_X_Inc)
        self.uic.Btn_Y_Inc.clicked.connect(self.Btn_Y_Inc)
        self.uic.Btn_Z_Inc.clicked.connect(self.Btn_Z_Inc)
        self.uic.Btn_Move.clicked.connect(self.Btn_Move)
        self.uic.Btn_Start.clicked.connect(self.Btn_Start)
        self.uic.Btn_Stop.clicked.connect(self.Btn_Stop)
        self.uic.Btn_Close.clicked.connect(self.Btn_Close)
        self.uic.Btn_Run.clicked.connect(self.Btn_Run)

        #Slider
        self.uic.Sd_Speed.valueChanged.connect(self.Sd_Speed)
        self.uic.Sd_Acc.valueChanged.connect(self.Sd_Acc)
        self.uic.Sd_Deg.valueChanged.connect(self.Sd_Deg)
        self.uic.Sd_cm.valueChanged.connect(self.Sd_cm)

        #checkbox
        self.uic.Check_B.stateChanged.connect(lambda: self.Checkstate(self.uic.Check_B))
        self.uic.Check_Y.stateChanged.connect(lambda: self.Checkstate(self.uic.Check_Y))
        self.uic.Check_R.stateChanged.connect(lambda: self.Checkstate(self.uic.Check_R))
        self.uic.Check_A.stateChanged.connect(lambda: self.Checkstate(self.uic.Check_A))
    def Checkstate(self, type):
        if type.text() == 'Blue':
            if type.isChecked() == True:
                self.flag_blue = True
                self.flag_red = False
                self.flag_yellow = False
                self.flag_all = False
                self.uic.Check_R.setChecked(False)
                self.uic.Check_Y.setChecked(False)
                self.uic.Check_A.setChecked(False)
        elif type.text() == 'Yellow':
            if type.isChecked() == True:
                self.flag_yellow = True
                self.flag_red = False
                self.flag_blue = False
                self.flag_all = False
                self.uic.Check_R.setChecked(False)
                self.uic.Check_B.setChecked(False)
                self.uic.Check_A.setChecked(False)
        elif type.text() == 'Red':
            if type.isChecked() == True:
                self.flag_red = True
                self.flag_yellow = False
                self.flag_blue = False
                self.flag_all = False
                self.uic.Check_B.setChecked(False)
                self.uic.Check_Y.setChecked(False)
                self.uic.Check_A.setChecked(False)
        else:
            if type.isChecked() == True:
                self.flag_red = False
                self.flag_yellow = False
                self.flag_blue = False
                self.flag_all =True
                self.uic.Check_Y.setChecked(False)
                self.uic.Check_B.setChecked(False)
                self.uic.Check_R.setChecked(False)


    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            a = str(port)
            self.uic.Com.addItem(a[:5])

    def connect(self):
        if self.uic.Btn_Connect.text() == "Connect":
            try:
                port = self.uic.Com.currentText()
                baudrate = int(self.uic.Baudrate.currentText())
                self.com.connection(port, baudrate)
                self.uic.Status_Light.setStyleSheet("background: rgb(51,255,51); border-radius:20px")
                self.uic.Btn_Connect.setText("Disconnect")
                self.uic.Lb_Status.setText("Robot Status: On")
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Warning)
                msg.setText("Check your COM Port")
                msg.setWindowTitle("Error")
                msg.setStandardButtons(QMessageBox.Ok)
                msg.exec()
        else:
            self.uic.Btn_Connect.setText("Connect")
            self.uic.Lb_Status.setText("Robot Status: Off")
            self.uic.Status_Light.setStyleSheet("background: rgb(255,0,0); border-radius:20px")
            self.com.port.close()

    def Sd_Speed(self):
        v = self.uic.Sd_Speed.value()
        self.uic.Te_Speed.setText(str(v))

    def Sd_Acc(self):
        a = self.uic.Sd_Acc.value()
        self.uic.Te_Acc.setText(str(a))

    def Sd_Deg(self):
        deg = self.uic.Sd_Deg.value()
        self.uic.Te_deg.setText(str(deg))

    def Sd_cm(self):
        cm = self.uic.Sd_cm.value()
        self.uic.Tx_cm.setText(str(cm))

    def send(self):
        v = self.uic.Sd_Speed.value()
        a = self.uic.Sd_Acc.value()
        buffer = "0" + "," + str(v) + "," + str(a) + "," + "0" + ',' + '0' + ','
        self.com.send_data(buffer)

    def Calib_J1(self):
        buffer = "2,1,0,0,0,"
        self.com.send_data(buffer)

    def Calib_J2(self):
        buffer = "2,2,0,0,0,"
        self.com.send_data(buffer)

    def Calib_J3(self):
        buffer = "2,3,0,0,0,"
        self.com.send_data(buffer)

    def Tool(self):
        if(self.uic.Btn_Tool.text() == 'Gripper On'):
            self.uic.Btn_Tool.setText('Gripper Off')
            buffer = '2,4,0,0,0,'
        else:
            self.uic.Btn_Tool.setText('Gripper On')
            buffer = '2,5,0,0,0,'
        self.com.send_data(buffer)

    def Home(self):
        self.uic.Te_J1.setText("0")
        self.uic.Te_J2.setText("100")
        self.uic.Te_J3.setText("-130")
        self.uic.Btn_Move.setDisabled(False)
        self.uic.Btn_X_Dec.setDisabled(False)
        self.uic.Btn_Y_Dec.setDisabled(False)
        self.uic.Btn_Z_Dec.setDisabled(False)
        self.uic.Btn_X_Inc.setDisabled(False)
        self.uic.Btn_Y_Inc.setDisabled(False)
        self.uic.Btn_Z_Inc.setDisabled(False)
        self.uic.Btn_J1_Dec.setDisabled(False)
        self.uic.Btn_J2_Dec.setDisabled(False)
        self.uic.Btn_J3_Dec.setDisabled(False)
        self.uic.Btn_J1_Inc.setDisabled(False)
        self.uic.Btn_J2_Inc.setDisabled(False)
        self.uic.Btn_J3_Inc.setDisabled(False)
        self.Forward()
        self.uic.Te_SX.setText('0')
        self.uic.Te_SY.setText('0')
        self.uic.Te_SZ.setText('0')

    def Btn_J1_Dec(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta1 = float(self.uic.Te_J1.toPlainText()) - step
        self.uic.Te_J1.setText(str(self.theta1))
        self.Forward()

    def Btn_J2_Dec(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta2 = float(self.uic.Te_J2.toPlainText()) - step
        self.uic.Te_J2.setText(str(self.theta2))
        self.Forward()

    def Btn_J3_Dec(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta3 = float(self.uic.Te_J3.toPlainText()) - step
        self.uic.Te_J3.setText(str(self.theta3))
        self.Forward()

    def Btn_J1_Inc(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta1 = float(self.uic.Te_J1.toPlainText()) + step
        self.uic.Te_J1.setText(str(self.theta1))
        self.Forward()

    def Btn_J2_Inc(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta2 = float(self.uic.Te_J2.toPlainText()) + step
        self.uic.Te_J2.setText(str(self.theta2))
        self.Forward()

    def Btn_J3_Inc(self):
        step = int(self.uic.Sd_Deg.value())
        self.theta3 = float(self.uic.Te_J3.toPlainText()) + step
        self.uic.Te_J3.setText(str(self.theta3))
        self.Forward()

    def Btn_X_Dec(self):
        step = int(self.uic.Sd_cm.value())
        self.Px = float(self.uic.Te_X.toPlainText()) - step
        self.uic.Te_X.setText(str(round(self.Px,2)))
        self.Inverse()

    def Btn_Y_Dec(self):
        step = int(self.uic.Sd_cm.value())
        self.Py = float(self.uic.Te_Y.toPlainText()) - step
        self.uic.Te_Y.setText(str(round(self.Py,2)))
        self.Inverse()

    def Btn_Z_Dec(self):
        step = int(self.uic.Sd_cm.value())
        self.Pz = float(self.uic.Te_Z.toPlainText()) - step
        self.uic.Te_Z.setText(str(round(self.Pz,2)))
        self.Inverse()

    def Btn_X_Inc(self):
        step = int(self.uic.Sd_cm.value())
        self.Px = float(self.uic.Te_X.toPlainText()) + step
        self.uic.Te_X.setText(str(round(self.Px,2)))
        self.Inverse()

    def Btn_Y_Inc(self):
        step = int(self.uic.Sd_cm.value())
        self.Py = float(self.uic.Te_Y.toPlainText()) + step
        self.uic.Te_Y.setText(str(round(self.Py,2)))
        self.Inverse()

    def Btn_Z_Inc(self):
        step = int(self.uic.Sd_cm.value())
        self.Pz = float(self.uic.Te_Z.toPlainText()) + step
        self.uic.Te_Z.setText(str(round(self.Pz,2)))
        self.Inverse()

    def Btn_Move(self):
        Px = self.uic.Te_SX.toPlainText()
        Py = self.uic.Te_SY.toPlainText()
        Pz = self.uic.Te_SZ.toPlainText()
        self.uic.Te_X.setText(Px)
        self.uic.Te_Y.setText(Py)
        self.uic.Te_Z.setText(Pz)
        self.Px = float(self.uic.Te_X.toPlainText())
        self.Py = float(self.uic.Te_Y.toPlainText())
        self.Pz = float(self.uic.Te_Z.toPlainText())
        self.theta1 = math.atan2(self.Py, self.Px)
        self.uic.Te_CX.setText(str(self.Px))
        self.uic.Te_CY.setText(str(self.Py))
        self.uic.Te_CZ.setText(str(self.Pz))
        self.Px = self.Px - self.a4 * math.cos(self.theta1)
        self.Py = self.Py - self.a4 * math.sin(self.theta1)
        self.Pz = self.Pz + self.a5
        r = math.sqrt(self.Px ** 2 + self.Py ** 2 + (self.Pz - self.d1) ** 2)
        self.theta3 = -math.acos((r ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
        self.theta2 = math.asin((self.Pz - self.d1) / r) + math.atan(
            self.a3 * math.sin(abs(self.theta3)) / (self.a2 + self.a3 * math.cos(abs(self.theta3))))
        self.theta1 = round(self.theta1 * 180 / math.pi, 2)
        self.theta2 = round(self.theta2 * 180 / math.pi, 2)
        self.theta3 = round(self.theta3 * 180 / math.pi, 2)
        self.uic.Te_J1.setText(str(self.theta1))
        self.uic.Te_J2.setText(str(self.theta2))
        self.uic.Te_J3.setText(str(self.theta3))
        buff = "4" + "," + str(self.theta1) + "," + str(self.theta2) + "," + str(self.theta3) + ',' + '0' +','
        self.com.send_data(buff)
        self.Pitch = 90
        self.Roll = 0
        self.Yaw = round(90 - self.theta1, 2)
        self.uic.Te_CR.setText(str(self.Roll))
        self.uic.Te_CP.setText(str(self.Pitch))
        self.uic.Te_CYw.setText(str(self.Yaw))
    def Forward(self):
        self.theta1 = self.uic.Te_J1.toPlainText()
        self.theta2 = self.uic.Te_J2.toPlainText()
        self.theta3 = self.uic.Te_J3.toPlainText()
        buffer = "3" + "," + self.theta1 + "," + self.theta2 + "," + self.theta3 + ',' + '0' + ','
        self.com.send_data(buffer)
        self.theta1 = round(float(self.theta1) * math.pi / 180, 2)
        self.theta2 = round(float(self.theta2) * math.pi / 180, 2)
        self.theta3 = round(float(self.theta3) * math.pi / 180, 2)
        self.Px = round(math.cos(self.theta1) * (
                    self.a4 + self.a3 * math.cos(self.theta2 + self.theta3) + self.a2 * math.cos(self.theta2)), 2)
        self.Py = round(math.sin(self.theta1) * (
                    self.a4 + self.a3 * math.cos(self.theta2 + self.theta3) + self.a2 * math.cos(self.theta2)), 2)
        self.Pz = round(
            self.d1 - self.a5 + self.a3 * math.sin(self.theta2 + self.theta3) + self.a2 * math.sin(self.theta2), 2)
        self.Pitch = 90
        self.Roll = 0
        self.Yaw = round(90 - float(self.uic.Te_J1.toPlainText()), 2)
        self.uic.Te_CX.setText(str(self.Px))
        self.uic.Te_CY.setText(str(self.Py))
        self.uic.Te_CZ.setText(str(self.Pz))
        self.uic.Te_CR.setText(str(self.Roll))
        self.uic.Te_CP.setText(str(self.Pitch))
        self.uic.Te_CYw.setText(str(self.Yaw))
        self.uic.Te_X.setText(str(self.Px))
        self.uic.Te_Y.setText(str(self.Py))
        self.uic.Te_Z.setText(str(self.Pz))

    def Inverse(self):
        self.Px = float(self.uic.Te_X.toPlainText())
        self.Py = float(self.uic.Te_Y.toPlainText())
        self.Pz = float(self.uic.Te_Z.toPlainText())
        self.theta1 = math.atan2(self.Py, self.Px)
        self.uic.Te_CX.setText(str(self.Px))
        self.uic.Te_CY.setText(str(self.Py))
        self.uic.Te_CZ.setText(str(self.Pz))
        self.Px = self.Px - self.a4 * math.cos(self.theta1)
        self.Py = self.Py - self.a4 * math.sin(self.theta1)
        self.Pz = self.Pz + self.a5
        r = math.sqrt(self.Px ** 2 + self.Py ** 2 + (self.Pz - self.d1) ** 2)
        self.theta3 = -math.acos((r ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
        self.theta2 = math.asin((self.Pz - self.d1) / r) + math.atan(
            self.a3 * math.sin(abs(self.theta3)) / (self.a2 + self.a3 * math.cos(abs(self.theta3))))
        self.theta1 = round(self.theta1 * 180 / math.pi, 2)
        self.theta2 = round(self.theta2 * 180 / math.pi, 2)
        self.theta3 = round(self.theta3 * 180 / math.pi, 2)
        self.uic.Te_J1.setText(str(self.theta1))
        self.uic.Te_J2.setText(str(self.theta2))
        self.uic.Te_J3.setText(str(self.theta3))
        buff = "3" + "," + str(self.theta1) + "," + str(self.theta2) + "," + str(self.theta3) + ',' + '0' + ','
        self.com.send_data(buff)
        self.Pitch = 90
        self.Roll = 0
        self.Yaw = round(90 - self.theta1, 2)
        self.uic.Te_CR.setText(str(self.Roll))
        self.uic.Te_CP.setText(str(self.Pitch))
        self.uic.Te_CYw.setText(str(self.Yaw))
    def Inverse_2(self, Px, Py, Pz):
        try:
            theta1 = math.atan2(Py, Px)
            Px = Px - self.a4 * math.cos(theta1)
            Py = Py - self.a4 * math.sin(theta1)
            Pz = Pz + self.a5
            r = math.sqrt(Px ** 2 + Py ** 2 + (Pz - self.d1) ** 2)
            theta3 = -math.acos((r ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
            theta2 = math.asin((Pz - self.d1) / r) + math.atan(
                self.a3 * math.sin(abs(theta3)) / (self.a2 + self.a3 * math.cos(abs(theta3))))
            theta1 = round(theta1 * 180 / math.pi, 2)
            theta2 = round(theta2 * 180 / math.pi, 2)
            theta3 = round(theta3 * 180 / math.pi, 2)
            return theta1, theta2, theta3
        except:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Out Of Workspace")
            msg.setWindowTitle("Error")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec()

    def start(self):
        Pz = 0.5
        def Dectec():
            buffer_trans = []
            last_time = time.time()
            len_buff = 1
            while self.flag:
                _, self.frame = self.cap.read()
                self.frame = self.frame[0:240,0:640]
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
                b = 0
                for c in cnts:

                    area = cv2.contourArea(c)
                    if (2000 > area > 1000): #20000,18000
                        b = b + 1
                        cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                        M = cv2.moments(c)
                        cx = int(M["m10"] / (M["m00"]))
                        cy = int(M["m01"] / (M["m00"]))
                        x_cm, y_cm = self.CV_P2cm(cx, cy)
                        theta1, theta2, theta3 = self.Inverse_2(x_cm, y_cm, Pz)
                        data = '4' + ',' + str(theta1) + ',' + str(theta2) + ',' + str(theta3) + ',' + '0' + ','
                        if self.flag_blue or self.flag_all:
                            self.info.append(data)
                        cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(self.frame, "blue:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                        cv2.putText(self.frame, str(int(b)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                        cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)

                        cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)
                y=0
                cnts = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                for c in cnts:

                    area = cv2.contourArea(c)
                    if (2000 > area > 1000):
                        y = y + 1
                        cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                        M = cv2.moments(c)
                        cx = int(M["m10"] / (M["m00"]))
                        cy = int(M["m01"] / (M["m00"]))
                        x_cm, y_cm = self.CV_P2cm(cx, cy)
                        theta1, theta2, theta3 = self.Inverse_2(x_cm, y_cm, Pz)
                        data = '4' + ',' + str(theta1) + ',' + str(theta2) + ',' + str(theta3) + ',' + '1' + ','
                        if self.flag_yellow or self.flag_all:
                            self.info.append(data)
                        cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(self.frame, "yellow:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                        cv2.putText(self.frame, str(int(b+y)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                        cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)

                        cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)
                r=0
                cnts = cv2.findContours(mask3 + mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                for c in cnts:

                    area = cv2.contourArea(c)
                    if (2000 > area > 1000):
                        r = r + 1
                        cv2.drawContours(self.frame, [c], -1, (0, 255, 0), 3)
                        M = cv2.moments(c)
                        cx = int(M["m10"] / (M["m00"]))
                        cy = int(M["m01"] / (M["m00"]))
                        x_cm, y_cm = self.CV_P2cm(cx, cy)
                        theta1, theta2, theta3 = self.Inverse_2(x_cm, y_cm, Pz)
                        data = '4' + ',' + str(theta1) + ',' + str(theta2) + ',' + str(theta3) + ',' + '2' + ','
                        if self.flag_red or self.flag_all:
                            self.info.append(data)
                        cv2.circle(self.frame, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(self.frame, "Red:", (cx + 9, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))
                        cv2.putText(self.frame, str(int(b+y+r)), (cx + 65, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 1))

                        cv2.putText(self.frame, "X=", (cx - 22, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(x_cm)), (cx + 12, cy - 16), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)

                        cv2.putText(self.frame, "Y=", (cx - 22, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(self.frame, str(float(y_cm)), (cx + 12, cy + 23), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0),
                                    1)
                cv2.line(self.frame, (0, 0), (0, 480), (255, 0, 0), 2)
                cv2.line(self.frame, (0, 0), (640, 0), (255, 0, 0), 2)
                cv2.line(self.frame, (640, 480), (640, 0), (255, 0, 0), 2)
                cv2.line(self.frame, (640, 480), (0, 480), (255, 0, 0), 2)
                self.frame = cv2.resize(self.frame,(900,620),interpolation=cv2.INTER_LINEAR)
                image = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_BGR888)
                self.uic.lb_View.setPixmap(QtGui.QPixmap.fromImage(image))
                k = len(self.info)
                current_time = time.time()
                if (k != 0) & self.run & (current_time - last_time > len_buff * 10):
                    if self.flag_blue == True or self.flag_all == True:
                        self.blue = self.blue + b
                        self.uic.lb_B.setText('Blue: ' + str(self.blue))
                    if self.flag_yellow == True or self.flag_all == True:
                        self.yellow = self.yellow + y
                        self.uic.lb_Y.setText('Yellow: ' + str(self.yellow))
                    if self.flag_red == True or self.flag_all == True:
                        self.red = self.red + r
                        self.uic.lb_R.setText('Red: ' + str(self.red))
                    self.uic.lb_T.setText('Total: ' + str(self.blue + self.yellow + self.red))
                    len_buff = len(self.info)
                    buffer_trans = self.info
                    for i in buffer_trans:
                        self.com.send_data(i)
                        time.sleep(0.1)
                    last_time = current_time
                self.info = []
                cv2.waitKey(1)
                if self.flag == 0:
                    self.uic.lb_View.clear()
                    break
        thread = threading.Thread(target=Dectec())
        thread.start()
        # self.cap.release()
        # cv2.destroyAllWindows()
    def CV_P2cm(self, a, b):
        ratio_x = 22.2 / 640
        ratio_y = 17.3 / 480
        x_cm = round(a * ratio_x, 2)
        y_cm = round(b * ratio_y, 2)
        x_robot = round(-y_cm + 24, 2)
        y_robot = round(-x_cm + 12.4, 2)
        # x_robot = round(y_cm + 14)
        # y_robot = round(x_cm - 14)
        return x_robot, y_robot

    def Btn_Start(self):
        self.uic.Btn_Start.setEnabled(False)
        self.uic.Btn_Stop.setEnabled(True)
        self.flag = 1
        self.start()
    def Btn_Stop(self):
        self.uic.Btn_Stop.setEnabled(False)
        self.uic.Btn_Start.setEnabled(True)
        self.flag = 0
        self.run = 0
        self.start()
    def Btn_Run(self):
        if (self.uic.Btn_Run.text() == 'Run'):
            self.run = 1
            self.uic.Btn_Run.setText('Pause')
        else:
            self.run = 0
            self.uic.Btn_Run.setText('Run')



    def Btn_Close(self):
        self.main_win.close()

    def show(self):
        self.main_win.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())
