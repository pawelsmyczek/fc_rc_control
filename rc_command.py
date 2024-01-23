import argparse
import os
import signal
import sys
import termios
import struct
import time
import tty
import serial

if os.uname().nodename == 'raspberrypi':
    import RPi.GPIO as GPIO

# from serial.tools import list_ports
# from pynput import keyboard
from threading import Thread
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


class E32:
    def __init__(self, p, baud):
        self.baud = baud
        self.dev = serial.Serial(port=p, baudrate=baud, parity=serial.PARITY_NONE, timeout=0.1, write_timeout=0.1)
        self.read_thread = Thread(target=self.write_, args=())
        self.buffer = None
        self.write_buffer = []
        self.hb = 0
        self.MAGIC_TX = 12345
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

    def write_(self):
        print("starting to read/write")
        last_update_rx_frame = 0
        rx_time_update = 0.8
        while not self.finish_all:
            time.sleep(0.008)
            now = time.time()
            try:
                self.write_buffer.clear()
                self.hb += 1
                if self.hb >= 16000:
                    self.hb = 0
                txframe = TX_FRAME(self.MAGIC_TX, self.hb
                                   , RPYT["ROLL"], RPYT["PITCH"], RPYT["YAW"], RPYT["THROTTLE"], State)

                self.write(bytes(txframe))
            except serial.SerialException as eser:
                print(eser.strerror)
            except TypeError:
                print("type error")
            if now >= last_update_rx_frame:
                last_update_rx_frame = now + rx_time_update
                self.read()
                # if len(self.buffer) >= 34:
                #     for i in range(0, len(self.buffer)):
                #         if self.buffer[i] == 60:
                #             print("buff[{}]={}".format(i, self.buffer[0]))
                #             if len(self.buffer) >= i + 33 + 1:
                #                 if self.buffer[i + 33] == 62:
                #                     rx_frame = RX_FRAME(self.buffer[i:i + 34])
                #                     print("read frame {}".format(str(rx_frame)))
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

    def finish(self):
        self.dev.close()
        self.finish_all = True
        self.read_thread.join()

    def start_reading(self):
        self.read_thread.start()


class PadDev:

    def __init__(self, name):
        self.pad = InputDevice(name)
        self.read_thread = Thread(target=self.read, args=())
        self.finish = False
        print("Device:\n {}\n\n Its capabilites:\n {}".format(self.pad, self.pad.capabilities(verbose=True)))

    def start(self):
        self.read_thread.start()

    def read(self):
        global State
        while not self.finish:
            # r,w,x = select.select([self.pad], [], [])
            select.select([self.pad], [], [])
            for event in self.pad.read():
                if event.type == ecodes.EV_KEY:
                    categorized_event = categorize(event).event
                    if categorized_event.code == 308:
                        if categorized_event.value == 1:
                            State = 255 if State == 0 else 0
                            print(State)
                if event.type == ecodes.EV_ABS:
                    categorized_event = categorize(event).event
                    if categorized_event.code == 0:
                        RPYT['ROLL'] = categorized_event.value
                    elif categorized_event.code == 1:
                        RPYT['PITCH'] = categorized_event.value
                    elif categorized_event.code == 3:
                        RPYT['YAW'] = categorized_event.value
                    elif categorized_event.code == 4:
                        RPYT['THROTTLE'] = categorized_event.value

    def cleanup(self):
        self.pad.close()
        self.finish = True


parser = argparse.ArgumentParser(description='Issues a command for a receiver device, further doc TODO')
parser.add_argument('-g', dest='gpio', type=int, default=17,
                    help="GPIO pin (Default: 17)")
parser.add_argument('-t', dest='protocol', type=int, default=None,
                    help="Protocol (Default: 1)")
args = parser.parse_args()
finish_all = False
commander = None
e32 = None
pad = None


def exit_handler(signal_, frame_):
    global e32
    global finish_all
    global pad
    e32.finish()
    pad.cleanup()
    finish_all = True
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)
