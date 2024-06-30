from uvc.uvc_bindings import CameraMode

from rc_command import E32, PadDev
from threading import Lock
import threading
import uvc
import time
import io
import tkinter as tk
from tkinter import messagebox
from tkinter import filedialog
import sys
import signal
from PIL import ImageTk, Image

finish_all = False


def open_available_devices_list(title, initial_dir):
    return filedialog.askopenfilename(initialdir=initial_dir,
                                      title="Select a " + title,
                                      filetypes=[("All files",
                                                  "*")])


class FPVWindow:
    def __init__(self, xsize, ysize, rx_frame):
        self.tkimg = None
        self.capture_device = None
        self.captured_frame = None
        self.uvc_device = None
        self.dev_list = []
        self.e32 = None
        self.pad = None
        self.xsize = 640 if xsize < 640 else xsize
        self.ysize = 480 if ysize < 480 else ysize
        self.rx_frame = rx_frame
        self.finish_all = False

        self.video_stream_thread = threading.Thread(target=self.__frame_to_tk)
        self.uvc_video_panel_update_thread = threading.Thread(target=self.__uvc_video_panel_update)

        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.menu = tk.Menu(self.root)
        self.frame = tk.Frame(self.root, bg="white")

        self.device_menu = tk.Menu(self.menu)
        self.menu.add_cascade(label="Devices", menu=self.device_menu)
        self.device_menu.add_command(label="Choose receiver...", command=self.__try_start_receiver)
        self.device_menu.add_command(label="Choose controller...", command=self.__try_start_controller)
        self.device_menu.add_command(label="Choose VTX...", command=self.__try_to_spawn_uvc_widget)
        self.widgets = {
            "UVC_VIDEO_PANEL": [tk.Label(self.root), 1, 3]
        }

    def prepare_window(self):
        self.frame.grid()
        for key, value in self.widgets.items():
            value[0].grid(column=value[1],
                          row=value[2])

        # menu add
        self.root.config(menu=self.menu)

        self.root.title("FPV")
        self.root.geometry("{}x{}".format(self.xsize, self.ysize))

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.finish_all = True
            if self.e32 is not None:
                print("Closing rxtx")
                self.e32.finish()
                time.sleep(1)
            if self.pad is not None:
                print("Closing pad")
                self.pad.cleanup()
                time.sleep(1)
            if self.video_stream_thread.is_alive():
                print("Closing vid stream")
                self.video_stream_thread.join(1)
                time.sleep(1)
            if self.uvc_video_panel_update_thread.is_alive():
                print("Closing vid update panel")
                self.uvc_video_panel_update_thread.join(1)
                time.sleep(1)
            print("Closing window")
            self.root.destroy()


    def __try_to_spawn_uvc_widget(self):

        vtxname = open_available_devices_list("VTX device", "/dev")
        print(vtxname)
        while len(self.dev_list) == 0:
            print("trying to capture UVC device, first from the list will be taken!")
            time.sleep(1)
            self.dev_list = uvc.device_list()

        print("found: ", self.dev_list[0])
        if self.capture_device is not None:
            print("VTX already chosen")
            return
        self.capture_device = uvc.Capture(self.dev_list[0]['uid'])
        if len(self.capture_device.available_modes) == 0:
            print("No available UVC camera modes, returning")
            return

        print("Available UVC camera modes: ")
        for mode in self.capture_device.available_modes:
            print(f"\t {self.capture_device.name} mode: {mode}")
            if mode.width == 640 and mode.height == 480 and mode.fps == 60:
                self.capture_device.frame_mode = mode
                print(f"\tChosen above one")

        self.capture_device.bandwidth_factor = 4
        self.video_stream_thread.start()
        self.uvc_video_panel_update_thread.start()

    def __uvc_video_panel_update(self):
        if not self.finish_all:
            self.widgets["UVC_VIDEO_PANEL"][0].imgtk = self.tkimg
            self.widgets["UVC_VIDEO_PANEL"][0].configure(image=self.tkimg)
            self.root.after(17, self.__uvc_video_panel_update)  # TODO cant be ran in a loop ?

    def __frame_to_tk(self):
        while not self.finish_all:
            try:
                self.captured_frame = self.capture_device.get_frame_robust()
                self.tkimg = ImageTk.PhotoImage(Image.open(io.BytesIO(self.captured_frame.jpeg_buffer)))
            except TypeError:
                pass
            except AttributeError:
                pass
            except Exception:
                pass
            time.sleep(0.017)

    def __try_start_receiver(self):
        rxtxname = open_available_devices_list("Receiver device", "/dev")
        if rxtxname == "":
            print("Didn't choose any file, returning")
            return
        if self.e32 is not None:
            print("Receiver already chosen")
            return
        self.e32 = E32(rxtxname, 9600)
        self.e32.write_settings("8N1", 38400, 6, 0, 1, 1)
        # self.e32.read_config()
        time.sleep(0.5)
        self.e32.start_reading()

    def __try_start_controller(self):
        try:
            ctrlrname = open_available_devices_list("Controller device", "/dev/input")
            if ctrlrname == "":
                print("Didn't choose any file, returning")
                return
            if self.pad is not None:
                print("Controller already chosen")
                return
            self.pad = PadDev(ctrlrname)
            self.pad.start()
        except PermissionError:
            print("Not enough permission to open file")

    def start(self):
        self.root.mainloop()


# def signal_handler(signal, frame):
#     global finish_all
#     finish_all = True
#     cap = None
#     # frame_thread.join()
#     sys.exit(0)
#
#
# signal.signal(signal.SIGINT, signal_handler)


def main():
    # global commander
    # global e32
    # global finish_all
    # global pad
    main_window = FPVWindow(xsize=640, ysize=480, rx_frame=None)
    main_window.prepare_window()
    main_window.start()

    # while not finish_all:
    #     time.sleep(0.05)


if __name__ == '__main__':
    main()
