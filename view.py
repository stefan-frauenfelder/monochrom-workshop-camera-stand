import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from control import *


class View(QtWidgets.QMainWindow):

    def __init__(self, fsm):
        super().__init__()

        self.fsm = fsm

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(self.fsm.initialize)

        self.homing_button.clicked.connect(self.fsm.home)

    def closeEvent(self, event):
        print("Window close command issued, passing exit event to FSM.")
        self.fsm.exit()
        # close the window and exit the application
        event.accept()
