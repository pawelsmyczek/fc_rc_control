import argparse
import os
import signal
import sys
import termios
import struct
import time
import tty
import serial
from scipy.interpolate import interp1d
from scipy.signal import iirfilter, lfilter
from collections import deque
import numpy as np

if os.uname().nodename == 'raspberrypi':
    import RPi.GPIO as GPIO

# from serial.tools import list_ports
# from pynput import keyboard
from threading import Thread
from threading import Lock
from evdev import InputDevice, categorize, ecodes
import select

#
# import uvc
# from getkey import getkey, keys
if os.uname().nodename == 'raspberrypi':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600


class _Getch:
    def __call__(self, char_num):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(char_num)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


'''

joystick mapping:

ABS_L_Y - PITCH
ABS_L_X - ROLL
ABS_R_Y - THROTTLE
ABS_R_X - YAW

'''
INIT = 0
V = 100

RPYT = {
    'ROLL': INIT,
    'PITCH': INIT,
    'YAW': INIT,
    'THROTTLE': INIT
}

"""
represents control state, is an unsigned char
"""
State = INIT

"""
packed frame as bytes with a beginning and end of frame signs
"""


class TX_FRAME:
    def __init__(self, mgc, hb, r, p, y, t, state):
        self.__mgc = mgc
        self.__hb = hb
        self.__r = r
        self.__p = p
        self.__y = y
        self.__t = t
        self.__state = state % 256 if state < 256 else 1

    def __bytes__(self):
        return struct.pack("<cxHHhhhhBc", b'<'
                           , self.__mgc, self.__hb, self.__r, self.__p, self.__y, self.__t, self.__state
                           , b'>')

    def __str__(self):
        return "MGC {}\tHB {}\tR {}\tP {}\tY {}\tT {}\tst {}" \
            .format(self.__mgc, self.__hb, self.__r, self.__p, self.__y, self.__t, self.__state)


class RX_FRAME:
    def __init__(self, buffer):
        self.inp, self.mgc, self.hb, self.vel, self.lat, self.lon, self.r, self.p, self.heading, self.height, self.out \
            = struct.unpack("<cHHfffffffc", buffer)

    def __str__(self):
        return "MGC {}\tHB {}\tVEL {}\tLAT {}\tLON {}\tR {}\tP {}\tHEAD {}\tH {}".format(self.mgc, self.hb, self.vel,
                                                                                         self.lat, self.lon, self.r,
                                                                                         self.p, self.heading,
                                                                                         self.height)


finish_tx = False
finish_rx = False


class E32:
    def __init__(self, p, baud):
        self.baud = baud
        self.dev = serial.Serial(port=p, baudrate=baud, parity=serial.PARITY_NONE, timeout=0.09, write_timeout=0.03)
        self.write_thread = Thread(target=self.write_, args=())
        self.read_thread = Thread(target=self.read_, args=())
        self.buffer = None
        self.write_buffer = []
        self.hb = 0
        self.MAGIC_TX = 12345
        self.serial_lock = Lock()
        self.finish_all = False

    def write(self, data):
        self.dev.write(data)
        # self.dev.flush()

    def read(self):
        self.buffer = self.dev.read(128)

    def write_settings(self, up, baud, adr, ftx, io, fec):
        if os.uname().nodename != 'raspberrypi':
            print("cannot write configuration to receiver if it's not connected to RPI")
        self.dev.baudrate = baud
        self.baud = baud
        if os.uname().nodename == 'raspberrypi':
            GPIO.output(23, 0)
        # GPIO.output(24, 1)
        time.sleep(0.1)
        print(bytearray(("AT+B" + str(baud)).encode()))
        self.write(bytearray(("AT+B" + str(baud)).encode()))
        self.read()
        print("read: {}".format(self.buffer))

        time.sleep(0.1)
        print(b'AT+U' + bytearray(up.encode()))
        self.write(b'AT+U' + bytearray(up.encode()))
        self.read()
        print("read: {}".format(self.buffer))
        time.sleep(0.1)

        print(b'AT+P' + bytearray(str(adr).encode()))
        self.write(b'AT+P' + bytearray(str(adr).encode()))
        self.read()
        print("read: {}".format(self.buffer))
        time.sleep(0.1)

        print(b'AT+C0' + bytearray(str(21).encode()))
        self.write(b'AT+C0' + bytearray(str(21).encode()))
        self.read()
        print("read: {}".format(self.buffer))
        time.sleep(0.1)
        if os.uname().nodename == 'raspberrypi':
            GPIO.output(23, 1)
        time.sleep(0.1)

    def read_config(self):
        if os.uname().nodename != 'raspberrypi':
            print("cannot read configuration from receiver if it's not connected to RPI")
            return
        self.dev.baudrate = 9600
        time.sleep(0.1)
        if os.uname().nodename == 'raspberrypi':
            GPIO.output(23, 0)
        time.sleep(0.1)
        self.write(b'AT+RX')
        self.read()
        print("RXTX CONFIG:")
        print("{}".format(self.buffer))
        time.sleep(0.1)

        if os.uname().nodename == 'raspberrypi':
            GPIO.output(23, 1)
        time.sleep(0.1)
        self.dev.baudrate = self.baud

    def read_(self):
        print("starting to read from rx/tx")
        global finish_rx
        last_update_rx_frame = 0
        rx_time_update = 0.9
        while finish_rx == False:
            time.sleep(0.008)
            now = time.time()
            try:
                if now >= last_update_rx_frame:
                    print("reading")
                    last_update_rx_frame = now + rx_time_update
                    with self.serial_lock:
                        self.buffer = self.dev.read(sys.getsizeof(RX_FRAME) + 2)
                    if len(self.buffer) < 34:
                        continue
                    for i in range(0, len(self.buffer)):
                        if self.buffer[i] != 60:
                            continue
                        print("buff[{}]={}".format(i, self.buffer[0]))
                        if len(self.buffer) < i + 33 + 1:
                            continue
                        if self.buffer[i + 33] == 62:
                            rx_frame = RX_FRAME(self.buffer[i:i + 34])
                            print("read frame {}".format(str(rx_frame)))
            except serial.SerialException as eser:
                print("serial error on read: {}".format(eser.strerror))
                break
            except TypeError:
                continue
        time.sleep(1)
        print("Finished reading rxtx")

    def write_(self):
        global finish_tx
        global RPYT
        print("starting to write to rx/tx")
        while finish_tx == False:
            time.sleep(0.008)
            try:
                self.write_buffer.clear()

                self.hb = self.hb + 1 if self.hb < 16375 else 0

                txframe = TX_FRAME(self.MAGIC_TX, self.hb
                                   , RPYT["ROLL"], RPYT["PITCH"], RPYT["YAW"], RPYT["THROTTLE"], State)
                with self.serial_lock:
                    self.write(bytes(txframe))
            except serial.SerialException as eser:
                print("serial error on write: {}".format(eser.strerror))
                break
            except TypeError:
                continue
        time.sleep(1)
        print("Finished writing to rxtx")

    def finish(self):
        global finish_tx
        global finish_rx
        finish_tx = True
        finish_rx = True
        self.read_thread.join(2)
        self.write_thread.join(2)
        self.dev.close()
        print("e32.finish()")

    def start_reading(self):
        self.read_thread.start()
        self.write_thread.start()


finish_pad = False

from collections import deque

class LiveFilter:
    """Base class for live filters.
    """
    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")
class LiveLFilter(LiveFilter):
    def __init__(self, b, a):
        """Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)

    def _process(self, x):
        """Filter incoming data with standard difference equations.
        """
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y

class PadDev:

    def __init__(self, name):
        self.pad = InputDevice(name)
        self.read_thread = Thread(target=self.read, args=())
        print("Device:\n {}\n\n Its capabilites:\n {}".format(self.pad, self.pad.capabilities(verbose=True)))
        self.xmap11 = interp1d([-32768, 32767], [-1, 1])
        self.xremap = interp1d([-1, 1], [-1000, 1000])
        bfilt, afilt = iirfilter(4, Wn=2.5, fs=30, btype="low", ftype="butter")
        self.livefilter = LiveLFilter(bfilt, afilt)

    def start(self):
        self.read_thread.start()

    def MAP(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)
    def apply_dummy_expo(self, x, expo, out_left, out_right):
        # return x
        # stupid but works, applies lower values at around 0
        # TODO still flickers at some values, maybe low pass filter?
        x_f = expo/100
        x5_f = 1 - x_f
        # # xleft = 0 if out_left == 0 else out_left / abs(out_left)
        # # xright = 0 if out_right == 0 else out_right / abs(out_right)
        # # vxmap11 = float(self.MAP(x, -32768, 32767, xleft, xright))
        # fxmap = x_f * vxmap11 + x5_f * pow(vxmap11, 5)
        # # return int(self.MAP(fxmap, -1, 1, -1000, 1000))
        return int(self.MAP(x,-32768, 32767, out_left, out_right))

    def apply_filter(self, x):
        xfilt = self.livefilter(x)
        return xfilt

    def read(self):
        global State
        global finish_pad
        global RPYT
        while finish_pad == False:
            try:
                select.select([self.pad], [], [], 0.4)
                for event in self.pad.read():
                    if event.type == ecodes.EV_KEY:
                        categorized_event = categorize(event).event
                        if categorized_event.code == 308:
                            if categorized_event.value == 1:
                                State = 255 if State == 0 else 0
                                print(State)
                    if event.type == ecodes.EV_ABS:
                        print("r:{}\tp:{}\ty:{}\tt:{}".format(RPYT["ROLL"], RPYT["PITCH"], RPYT["YAW"]
                                                              , RPYT["THROTTLE"]))
                        categorized_event = categorize(event).event
                        if categorized_event.code == 0:
                            RPYT['ROLL'] = self.apply_dummy_expo(categorized_event.value, 16, -500, 500)
                        elif categorized_event.code == 1:
                            RPYT['PITCH'] = self.apply_dummy_expo(categorized_event.value, 16, -500, 500)
                        elif categorized_event.code == 3:
                            RPYT['YAW'] = self.apply_dummy_expo(categorized_event.value, 10, -500, 500)
                        elif categorized_event.code == 4:
                            RPYT['THROTTLE'] = self.apply_dummy_expo(categorized_event.value, 38, 1000, 0)
            except BlockingIOError:
                continue
            except OSError as e:
                print("os error, message : {}, done with that".format(e.strerror))
                break
        time.sleep(2)
        print("Finished reading pad")

    def cleanup(self):
        global finish_pad
        finish_pad = True
        self.read_thread.join(2)
        self.pad.close()

        print("pad.cleanup()")


# parser = argparse.ArgumentParser(description='Issues a command for a receiver device, further doc TODO')
# parser.add_argument('-g', dest='gpio', type=int, default=17,
#                     help="GPIO pin (Default: 17)")
# parser.add_argument('-t', dest='protocol', type=int, default=None,
#                     help="Protocol (Default: 1)")
# args = parser.parse_args()
finish_all = False
commander = None
e32 = None
pad = None


def exit_handler(signal_, frame_):
    # global e32
    # global finish_all
    # global pad
    # e32.finish()
    # pad.cleanup()
    # finish_all = True
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)
