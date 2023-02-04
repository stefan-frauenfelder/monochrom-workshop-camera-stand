import sys
from PyQt5 import QtCore, QtGui, QtWidgets

from hmi import View

if __name__ == '__main__':

    try:
        # create an application
        app = QtWidgets.QApplication(sys.argv)
        # create the main window of the app (QMainWindow)
        view = View()
        # show the view in maximized size
        view.showMaximized()
        # launch the app
        app.exec_()

    except KeyboardInterrupt:

        # exits when you press CTRL+C
        print('Exiting upon KeyboardInterrupt.')
