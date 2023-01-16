import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic


class View(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.homing_button.clicked.connect(self.homing_button_clicked)

        # self.actionClose.triggered.connect(self.closing_app)

    def homing_button_clicked(self):
        print('Click!')

    def closeEvent(self, event):
        print("Exiting...")
        event.accept()
