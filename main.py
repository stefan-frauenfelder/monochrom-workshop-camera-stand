import sys
from PyQt5 import QtCore, QtGui, QtWidgets

from hmi import Hmi, Hmi

if __name__ == '__main__':

    try:
        # create an application
        app = QtWidgets.QApplication(sys.argv)
        # set default font on app-level
        default_font = QtGui.QFont('NN1050', 20)
        app.setFont(default_font)
        # create the human/machine interface which handles user inputs and the display (window of the app (QMainWindow)
        hmi = Hmi()
        # show the view (not in maximized size)
        hmi.showFullScreen()  # alt: .showMaximized()
        # launch the app
        app.exec_()

    except KeyboardInterrupt:

        # exits when you press CTRL+C
        print('Exiting upon KeyboardInterrupt.')
