
import sys
from fsm import *
from coordinator import *
from hmi import *
from zcame2 import *

if __name__ == '__main__':

    try:
        # create the coordinator which will wait for function calls from the view
        coordinator = Coordinator()
        # create the finite state machine
        fsm = CameraMotionControlFsm(coordinator)
        # create the camera interface class to control the camera
        cam = ZCamE2()
        # create the controller which handles manual user inputs
        controller = Controller(fsm, coordinator)
        # hand the coordinator a reference to the controller
        coordinator.set_controller(controller)
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
