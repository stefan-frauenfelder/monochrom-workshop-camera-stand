import requests
import time

from datetime import datetime

E2_IP = '10.98.32.1'


class ZCamE2:

    def __init__(self):

        # status flag of the interface class
        self.ready = False
        # check connection
        if self.get_response():
            print('Connection to camera established.')
            # acquire the control session
            if self.get_control_session():
                print('Camera control session acquired.')
                self.ready = True
                self.sync_time()
                print('Camera time synchronized.')
                self.set_mode('record')
            else:
                print('Camera control session could not be obtained.')
        else:
            print('Unable to connect to camera.')

    def set_mode(self, mode):
        if mode == 'record':
            payload = {'action': 'to_rec'}
        elif mode == 'playback':
            payload = {'action': 'to_pb'}
        elif mode == 'standby':
            payload = {'action': 'to_standby'}
        else:
            raise ValueError('This is not a valid camera mode.')
        # send the command and allow for a long timeout in case the camera is in standby
        self.get_response(subdir='ctrl/mode', payload=payload, timeout=2)

    def start_recording(self):
        # verify the camera is in recording mode
        response = self.get_response(subdir='ctrl/mode', payload={'action': 'query'})
        if not response.json()['msg'] == 'rec':
            # switch to recording mode
            self.set_mode('record')
        # start recording
        response = self.get_response(subdir='ctrl/rec', payload={'action': 'start'})

    def stop_recording(self):
        # stop recording
        response = self.get_response(subdir='ctrl/rec', payload={'action': 'stop'})

    def sync_time(self):
        # get date and time
        now = datetime.now()
        date = now.strftime("%Y-%m-%d")
        time = now.strftime("%H:%M:%S")
        # fill into payload of URL
        payload = {'date': date, 'time': time}
        response = self.get_response(subdir='datetime', payload=payload)

    def get_control_session(self):
        # try to acquire the control session
        response = self.get_response('ctrl/session')
        # check if it failed
        if response.status_code == 409:
            # control session was already established
            # try to quit it
            payload = {'action': 'quit'}
            response = self.get_response('ctrl/session', payload=payload)
            time.sleep(0.1)
            response = self.get_response('ctrl/session')
            if response.status_code == 409:
                return None
        else:
            return response

    def get_response(self, subdir='info', payload=None, timeout=0.1):
        # this is a wrapper to do all the error handling
        # assemble URL
        url = 'http://' + E2_IP + '/' + subdir
        # initialize response to None
        response = None
        # send get request
        try:
            if payload:
                response = requests.get(url, params=payload, timeout=timeout)
            else:
                response = requests.get(url, timeout=timeout)
            # check for http errors
            response.raise_for_status()
        except requests.exceptions.HTTPError as error_h:
            print('Camera control http Error:', error_h)
        except requests.exceptions.ConnectionError as error_c:
            print('Camera control connection error:', error_c)
        except requests.exceptions.Timeout as error_t:
            print('Camera control connection timeout:', error_t)
        except requests.exceptions.RequestException as error:
            print('Camera control unexpected error:', error)
        # return
        return response
