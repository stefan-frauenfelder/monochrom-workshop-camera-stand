
import sys
from fsm import *
from coordinator import *
from view import *

if __name__ == '__main__':

    try:
        # create the coordinator which will wait for fuction calls from the view
        coordinator = Coordinator()
        # create the finite state machine
        fsm = CameraMotionControlFsm(coordinator)
        # create an application
        app = QtWidgets.QApplication(sys.argv)
        # create the main window of the app (QMainWindow)
        view = View(fsm)
        # hand the fsm a reference to the view
        fsm.set_view(view)
        # show the view in maximized size
        view.showMaximized()
        # launch the app
        app.exec_()

    except KeyboardInterrupt:
        # exits when you press CTRL+C
        print('Exiting upon KeyboardInterrupt.')

# end of main
