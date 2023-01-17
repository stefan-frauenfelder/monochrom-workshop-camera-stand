import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from control import *


class View(QtWidgets.QMainWindow):

    def __init__(self, controller):
        super().__init__()

        self.controller = controller

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.homing_button.clicked.connect(self.homing_button_clicked)

        # self.actionClose.triggered.connect(self.closing_app)

    def homing_button_clicked(self):
        self.controller.create_stepper_instances()

    def closeEvent(self, event):
        print("Exiting...")
        # power down the steppers
        self.controller.shutdown()
        # close the window and exit the application
        event.accept()
