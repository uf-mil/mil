#!/usr/bin/env python
from __future__ import division
from qt_gui.plugin import Plugin
import sys
import rospy
import os
import rospkg
from python_qt_binding import QtCore, QtGui, loadUi
from python_qt_binding.QtWidgets import QApplication, QWidget
from std_msgs.msg import Float32
from roboteq_msgs.msg import Feedback


# Display voltage from battery_monitor and the four motors to a GUI


__author__ = "Joseph Brooks"
__email__ = "brooksturtle@ufl.edu"
__license__ = "MIT"


class voltageGUI(Plugin):
    def __init__(self, context):
        super(voltageGUI, self).__init__(context)
        self.setObjectName('voltage_gui')

        self.myWidget = VoltageWidget()

        if context.serial_number() > 1:
            self.myWidget.setWindowTitle(self.myWidget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.myWidget)
        self.runGUI()

    # Updates Values displayed in GUI every second
    def runGUI(self):
        app = QApplication(sys.argv)
        self.myWidget.show()
        while (0 == 0):
            rospy.sleep(1.)
            self.myWidget.updateLabel()

        sys.exit(app.exec_())


class VoltageWidget(QWidget):
    resized = QtCore.pyqtSignal()

    def __init__(self):
        super(VoltageWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('voltage_gui'), 'resource', 'voltage_gui.ui')
        loadUi(ui_file, self)

        # Whenever the screen is resized the resizeFont function is called
        self.resized.connect(self.resizeFont)

        self.height = VoltageWidget.frameGeometry(self).height()

        self.fontSize = 40

        self.warningCounter = 0
        self.paramCounter = 0

        self.initThresh()

        # Subscribing to all the data we need
        self.battery_voltage = None
        rospy.Subscriber("/battery_monitor", Float32, self.updateMain)
        rospy.Subscriber("/FL_motor/feedback", Feedback, self.update_FL)
        self.voltageFL = None
        rospy.Subscriber("/FR_motor/feedback", Feedback, self.update_FR)
        self.voltageFR = None
        rospy.Subscriber("/BL_motor/feedback", Feedback, self.update_BL)
        self.voltageBL = None
        rospy.Subscriber("/BR_motor/feedback", Feedback, self.update_BR)
        self.voltageBR = None

    # The functions that the subscribers call in order to get new data
    def updateMain(self, mainData):
        self.battery_voltage = mainData

    def update_FL(self, dataFL):
        self.voltageFL = dataFL.supply_voltage

    def update_FR(self, dataFR):
        self.voltageFR = dataFR.supply_voltage

    def update_BL(self, dataBL):
        self.voltageBL = dataBL.supply_voltage

    def update_BR(self, dataBR):
        self.voltageBR = dataBR.supply_voltage

    # Part of signal that notifies program whenever window is resized
    def resizeEvent(self, event):
        self.resized.emit()
        return super(VoltageWidget, self).resizeEvent(event)

    # Increase/decrease size of fonts based on window resize
    def resizeFont(self):
        # gets new window dimensions, the self is needed because we are referencing
        # our VoltageWidget class
        height = VoltageWidget.frameGeometry(self).height()

        # The ratio change in width and height caused by the resize
        heightRatio = height / self.height

        # update to current
        self.height = height

        # Fonts like 16, 24 are references to height of letters. So I thought it would
        # Make sense to change font size proportionally to changes in window height
        self.fontSize = self.fontSize * heightRatio
        newfont = QtGui.QFont("Times", self.fontSize, QtGui.QFont.Bold)

        self.labelMain.setFont(newfont)
        self.labelFL.setFont(newfont)
        self.labelFR.setFont(newfont)
        self.labelBL.setFont(newfont)
        self.labelBR.setFont(newfont)
        threshFont = QtGui.QFont("Times", (self.fontSize) / 3, QtGui.QFont.Bold)
        self.labelThresh.setFont(threshFont)

    # Sets the text of the thrshold info box
    def initThresh(self):
        # Low and Critical decide what colors the boxes take for
        # Good (Green), Warning (Yellow), and Critical (Red)
        # If the parameter server has not set these values then we use the DEFAULT
        # values as of Oct. 2017 which are 26 and 20
        # The GUI notifies that it is using defualt values and will continually
        # check to see if the params have been set, as symboloized by self.gotParams
        self.gotParams = True
        self.lowThreshold = 26
        self.criticalThreshold = 20

        try:
            self.lowThreshold = rospy.get_param('battery-voltage/low')
            self.criticalThreshold = rospy.get_param('battery-voltage/critical')
        except:
            print("Low and Critical Voltage not set, Low: 26 Critical:20")
            self.lowThreshold = 26
            self.criticalThreshold = 20
            self.gotParams = False

        # Thresh is a box in the top right of the GUI that displays Threshold values and box layouts
        # self.labelThresh = QLabel(self)
        # self.labelThresh.setGeometry(QtCore.QRect((150+2*self.boxWidth), (10), self.boxWidth, self.boxHeight))
        threshText = "Low Threshold: {} \nCritical: {}".format(self.lowThreshold, self.criticalThreshold)
        self.labelThresh.setText(threshText)
        self.labelThresh.setStyleSheet("QLabel { background-color : white; color : black; }")
        threshFont = QtGui.QFont("Times", (self.fontSize) / 3, QtGui.QFont.Bold)
        self.labelThresh.setFont(threshFont)

    # If self.gotParams is False, the updateLabel function calls testParams every 5 seconds
    def testParams(self):
        try:
            self.lowThreshold = rospy.get_param('battery-voltage/low')
            self.criticalThreshold = rospy.get_param('battery-voltage/critical')
            self.gotParams = True
        except:
            self.gotParams = False

        if self.gotParams:
            threshText = "Low Threshold: {} \nCritical: {}".format(self.lowThreshold, self.criticalThreshold)
        else:
            threshText = "THRESHOLDS NOT SET\nUSING DEFAULT\nLow Threshold: {} \nCritical: {}".format(
                self.lowThreshold, self.criticalThreshold)
        self.labelThresh.setText(threshText)

    # sets colors of boxes based on current values of voltages for each box
    def setColors(self, numMain):
        if numMain > self.lowThreshold:
            self.labelMain.setStyleSheet("QLabel { background-color : green; color : white; }")
        elif numMain <= self.lowThreshold and numMain > self.criticalThreshold:
            self.labelMain.setStyleSheet("QLabel { background-color : yellow; color : black; }")
        elif numMain <= self.criticalThreshold:
            self.labelMain.setStyleSheet("QLabel { background-color : red; color : white; }")

        if self.voltageFL > self.lowThreshold:
            self.labelFL.setStyleSheet("QLabel { background-color : green; color : white; }")
        elif self.voltageFL <= self.lowThreshold and self.voltageFL > self.criticalThreshold:
            self.labelFL.setStyleSheet("QLabel { background-color : yellow; color : black; }")
        elif self.voltageFL <= self.criticalThreshold:
            self.labelFL.setStyleSheet("QLabel { background-color : red; color : white; }")

        if self.voltageFR > self.lowThreshold:
            self.labelFR.setStyleSheet("QLabel { background-color : green; color : white; }")
        elif self.voltageFR <= self.lowThreshold and self.voltageFR > self.criticalThreshold:
            self.labelFR.setStyleSheet("QLabel { background-color : yellow; color : black; }")
        elif self.voltageFR <= self.criticalThreshold:
            self.labelFR.setStyleSheet("QLabel { background-color : red; color : white; }")

        if self.voltageBL > self.lowThreshold:
            self.labelBL.setStyleSheet("QLabel { background-color : green; color : white; }")
        elif self.voltageBL <= self.lowThreshold and self.voltageBL > self.criticalThreshold:
            self.labelBL.setStyleSheet("QLabel { background-color : yellow; color : black; }")
        elif self.voltageBL <= self.criticalThreshold:
            self.labelBL.setStyleSheet("QLabel { background-color : red; color : white; }")

        if self.voltageBR > self.lowThreshold:
            self.labelBR.setStyleSheet("QLabel { background-color : green; color : white; }")
        elif self.voltageBR <= self.lowThreshold and self.voltageBR > self.criticalThreshold:
            self.labelBR.setStyleSheet("QLabel { background-color : yellow; color : black; }")
        elif self.voltageBR <= self.criticalThreshold:
            self.labelBR.setStyleSheet("QLabel { background-color : red; color : white; }")

    def updateLabel(self):
        # Tries self.gotParams every 3 function calls
        self.paramCounter = self.paramCounter + 1
        if self.gotParams is False and self.paramCounter >= 3:
            self.testParams()
            self.paramCounter = 0

        # Checks if battery_monitor.py is online, if not it sets that voltage to 0
        # otherwise ite formats the voltage data into an int to be shown in GUI
        try:
            stringMain = str(self.battery_voltage)
            stringMain = stringMain[5:]
            numMain = float(stringMain)
            numMain = int(numMain * 100)
            numMain = numMain / 100.0
        except:
            if self.warningCounter == 0:
                print("battery monitor is not online!!")
                self.warningCounter = 1
            numMain = 0.0

        self.setColors(numMain)

        # Turns all the voltages into strings and sets them as text in GUI boxes
        self.labelMain.setText("{}".format(numMain))
        try:
            numFL = (int(self.voltageFL * 100)) / 100
            stringFL = str(numFL)
            self.labelFL.setText("{}".format(stringFL))
        except:
            pass

        try:
            numFR = (int(self.voltageFR * 100)) / 100
            stringFR = str(numFR)
            self.labelFR.setText("{}".format(stringFR))
        except:
            pass

        try:
            numBL = (int(self.voltageBL * 100)) / 100
            stringBL = str(numBL)
            self.labelBL.setText("{}".format(stringBL))
        except:
            pass

        try:
            stringBR = float("{0:.2f}".format(self.voltageBR))
            self.labelBR.setText("{}".format(stringBR))
        except:
            pass

        QApplication.processEvents()
