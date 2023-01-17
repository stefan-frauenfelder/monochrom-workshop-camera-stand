
from control import *
from view import *

if __name__ == '__main__':

    # create the controller which will wait for fuction calls from the view
    controller = Controller()
    # create an application
    app = QtWidgets.QApplication(sys.argv)
    # create the main window of the app (QMainWindow)
    view = View(controller)
    # show the view in maximized size
    view.showMaximized()
    # launch the app
    app.exec_()

# end of main
