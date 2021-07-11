#!/usr/bin/python3
#
# Author    Kalvin
#
# A simple utility for reading and plotting the LTDZ ADC sample buffer.
# The ADC sample data can be saved into a MATLAB ascii data file for later
# analysis.
#
# Example:
#
#   python3 tkadc_read.py -p /dev/ttyUSB0 --start 90m --end 100m --count 10
#
# After a sweep, use the navigation buttons for navigating the sweep samples.
#
# File | New        Start a new sweep
# File | Save       Save the current sweep sample in MATLAB ascii data format
# File | Save All   Save all sweep samples in MATLAB ascii data format
# File | Exit       Quit
#

APP_VERSION     = "Ver 0.0"
APP_TITLE       = "LTDZ ADC Reader " + APP_VERSION
APP_GEOMETRY    = '1400x800'

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter.filedialog import asksaveasfile

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from threading import Thread
from queue import Queue
import serial, struct

from enum import Enum

import argparse

SERIAL_PORT     = "/dev/ttyUSB0"
SERIAL_BAUD     = 57600
SERIAL_TIMEOUT  = 10

serial_port     = SERIAL_PORT
serial_baud     = SERIAL_BAUD
serial_timeout  = SERIAL_TIMEOUT

MIN_Hz          = 35e6
MAX_Hz          = 4400e6
MIN_STEP_Hz     = 1
MAX_STEP_Hz     = 1000e3
MIN_STEP_COUNT  = 3
MAX_STEP_COUNT  = 9999
MIN_Fs_Hz       = 100e3
MAX_Fs_Hz       = 2e6

ADC_MIN_BUFFER  = 256
ADC_MAX_BUFFER  = 8192
ADC_REF_VALUE   = 2048

FFT_MIN_dB      = -100
FFT_MAX_dB      = 0
SPECTRUM_MIN_dB = -90
SPECTRUM_MAX_dB = 10

#------------------------------------------------------------------------------
# LTDZ sweep process
#------------------------------------------------------------------------------

# LTDZ sweep configuration
class LtdzSweepConfig:
    def __init__(self):
        # Serial port
        self.serial_port = None

        # Serial port baud rate
        self.serial_baud = None

        # Serial port read/write timeout in seconds
        self.serial_timeout = None

        # Sweep start frequency
        self.start_Hz = None

        # Sweep step frequency
        self.step_Hz = None

        # Sweep step count
        self.step_count = None

# LTDZ sweep data
class LtdzSweepData:
    def __init__(self):
        # Current sweep number
        self.n = None

        # Current frequency
        self.f_Hz = None

        # RX offset (0=Tracking generator disabled)
        self.rx_offset_Hz = None

        # Frequency step size
        self.step_Hz = None

        # ADC Sample rate
        self.sample_rate_Hz = None

        # ADC buffer size
        self.buffer_size = None

        # ADC samples
        self.samples = None

# LTDZ status
class LtdzStatus(Enum):
    # Success
    SUCCESS = 0
    # Serial port timeout
    TIMEOUT = 1
    # Serial port communication error
    ERROR = 2
    # Serial port other error
    FAIL = 3

# LTDZ process exit sentinel
class LtdzSweepExitStatus:
    def __init__(self, status: LtdzStatus, message: str=None):
        # Exit code
        self.status = status
        # Optional error message
        self.message = message

# LTDZ sweep processing thread
def ltdz_sweep_run(q: Queue, config: LtdzSweepConfig) -> None:
    print("LTDZ thread started.")

    try:
        with serial.Serial(config.serial_port,
            config.serial_baud, timeout=config.serial_timeout) as ser:

            # Create LTDZ command
            command = b'\x8f'
            command += b'b'
            command += "{:09}00{:06}{:04}".format(config.start_Hz//10, config.step_Hz//10, config.step_count).encode()

            # Write LTDZ command to serial port
            num_bytes = ser.write(command)
            if num_bytes < len(command):
                q.put(LtdzSweepExitStatus(LtdzStatus.FAIL,
                    "Fatal serial port '{}' error while writing command").format(config.serial_port))
                ser.close()
                return

            # Read sweep data from LTDZ
            for n in range(0, config.step_count):
                sd = LtdzSweepData()

                sd.n = n

                # Read center frequency
                buf = ser.read(4)
                if len(buf) < 4:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                sd.f_Hz = struct.unpack_from("<" + "I"*1, buf)[0]*10

                if sd.f_Hz < MIN_Hz or MAX_Hz < sd.f_Hz:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "Frequency '{}' outside valid range".format(sd.f_Hz)))
                    break

                # Read RX offset frequency (0 if tracking generator disabled)
                buf = ser.read(4)
                if len(buf) < 4:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                sd.rx_offset_Hz = struct.unpack_from("<" + "I"*1, buf)[0]*10

                # Read frequency step size
                buf = ser.read(4)
                if len(buf) < 4:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                sd.step_Hz = struct.unpack_from("<" + "i"*1, buf)[0]*10

                if sd.step_Hz < MIN_STEP_Hz or MAX_STEP_Hz < sd.step_Hz:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "Frequency step '{}' outside valid range.".format(sd.step_Hz)))
                    break

                # Read ADC sample clock M
                buf = ser.read(4)
                if len(buf) < 4:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                adc_clock_m = struct.unpack_from("<" + "I"*1, buf)[0]

                # Read ADC sample clock N
                buf = ser.read(4)
                if len(buf) < 4:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                adc_clock_n = struct.unpack_from("<" + "I"*1, buf)[0]

                if adc_clock_n == 0:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "ADC sample clock value N is zero"))
                    break

                sd.sample_rate_Hz = adc_clock_m/adc_clock_n;

                if sd.sample_rate_Hz < MIN_Fs_Hz or MAX_Fs_Hz < sd.sample_rate_Hz:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "Sample rate '{}' is outsize valid range.".format(sd.sample_rate_Hz)))
                    break

                if sd.sample_rate_Hz/2 < sd.rx_offset_Hz:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "RX offset '{}' is outsize valid range.".format(sd.rx_offset_Hz)))
                    break

                # Read ADC sample buffer size
                buf = ser.read(2)
                if len(buf) < 2:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                sd.buffer_size = struct.unpack_from("<" + "H"*1, buf)[0]

                if sd.buffer_size < ADC_MIN_BUFFER or ADC_MAX_BUFFER < sd.buffer_size:
                    q.put(LtdzSweepExitStatus(LtdzStatus.ERROR,
                        "ADC buffser size '{}' is outsize valid range.".format(sd.buffer_size)))
                    break

                # Read ADC sample buffer samples
                byte_count = 2*sd.buffer_size
                buf = ser.read(byte_count)
                if len(buf) < byte_count:
                    q.put(LtdzSweepExitStatus(LtdzStatus.TIMEOUT))
                    break

                sd.samples = struct.unpack_from("<" + "h"*sd.buffer_size, buf)

                # Push data to the main thread
                q.put(sd)

            # Successful sweep
            q.put(LtdzSweepExitStatus(LtdzStatus.SUCCESS))
            ser.close()

    except serial.SerialException:
        # Fatal error with serial port
        q.put(LtdzSweepExitStatus(LtdzStatus.FAIL,
            "Fatal serial port '{}' error.".format(config.serial_port)))

    print("LTDZ thread ended.")

# -----------------------------------------------------------------------------
# Application GUI
# -----------------------------------------------------------------------------

MENU_FILE               = "File"
MENU_FILE_NEW           = "New"
MENU_FILE_NEW_KB        = "<Control-n>"
MENU_FILE_SAVE          = "Save"
MENU_FILE_SAVE_KB       = "<Control-s>"
MENU_FILE_SAVE_ALL      = "Save all"
MENU_FILE_SAVE_ALL_KB   = "<Control-a>"
MENU_FILE_EXIT          = "Exit"
MENU_FILE_EXIT_KB1      = "<Control-x>"
MENU_FILE_EXIT_KB2      = "<Control-q>"

class Application(tk.Frame):
    def __init__(self, win, sweep_settings):
        super().__init__(win)
        # Sample frequencies
        self.f_Hz = []
        # Sample data items
        self.samples = []
        # Sample levels in dB
        self.level_dB = []
        self.win = win
        self.win.geometry(APP_GEOMETRY)
        self.win.resizable(False, False)
        self.sweep_settings = sweep_settings
        self.create_widgets()
        self.update_sweep_settings()
        self.sweep_run()

    def create_widgets(self):
        self.win.title(APP_TITLE)

        #
        # Create menus
        #

        self.menu_bar = tk.Menu(self.win)
        self.win.config(menu=self.menu_bar)
        self.file_menu = tk.Menu(self.menu_bar, tearoff=0)
        self.file_menu.add_command(label=MENU_FILE_NEW,
            command=self._file_new, accelerator="Ctrl+N")
        self.file_menu.add_command(label=MENU_FILE_SAVE,
            command=self._file_save, accelerator="Ctrl+S")
        self.file_menu.add_command(label=MENU_FILE_SAVE_ALL,
            command=self._file_save_all, accelerator="Ctrl+A")
        self.file_menu.add_separator();
        self.file_menu.add_command(label=MENU_FILE_EXIT,
            command=self._quit, accelerator="Ctrl-X")
        self.file_menu.bind_all(MENU_FILE_EXIT_KB1, self._quit_shortcut)
        self.file_menu.bind_all(MENU_FILE_EXIT_KB2, self._quit_shortcut)
        self.menu_bar.add_cascade(menu=self.file_menu, label=MENU_FILE)

        #
        # Default settings
        #

        PLOT_PADX = 5
        PLOT_PADY = 2

        PADX = 5
        PADY = 2

        LABEL_WIDTH = 10
        VALUE_WIDTH = 15

        #
        # Sweep parameters label frame items
        #

        COL = 0
        ROW = 0
        frm = ttk.LabelFrame(self.win, text=" Sweep Parameters ")
        frm.grid(column=COL, row=ROW, padx=5, pady=5, sticky=tk.EW)

        COL = 0
        ROW = 0
        self.sweep_start = tk.StringVar()
        ttk.Label(frm, text="Start:", width=LABEL_WIDTH, anchor=tk.W).grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_start, width=VALUE_WIDTH, anchor=tk.E).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        ROW += 1
        self.sweep_center = tk.StringVar()
        ttk.Label(frm, text="Center:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_center).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sweep_end = tk.StringVar()
        ttk.Label(frm, text="End:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_end).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sweep_span = tk.StringVar()
        ttk.Label(frm, text="Span:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_span).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sweep_step = tk.StringVar()
        ttk.Label(frm, text="Step:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_step).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sweep_count = tk.StringVar()
        ttk.Label(frm, text="Count:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sweep_count).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        #
        # Sampling configuration label frame items
        #

        COL = 0
        ROW = 1
        frm = ttk.LabelFrame(self.win, text=" Sampling Configuration ")
        frm.grid(column=COL, row=ROW, padx=5, pady=5, sticky=tk.EW)

        COL = 0
        ROW = 0
        self.sampling_rx_offset = tk.StringVar()
        ttk.Label(frm, text="RX Offset:", width=LABEL_WIDTH, anchor=tk.W).grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sampling_rx_offset, width=VALUE_WIDTH, anchor=tk.E).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sampling_step = tk.StringVar()
        ttk.Label(frm, text="Step size:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sampling_step).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sampling_rate = tk.StringVar()
        ttk.Label(frm, text="Sample rate:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sampling_rate).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sampling_buffer_length = tk.StringVar()
        ttk.Label(frm, text="Buffer length:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sampling_buffer_length).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        self.sampling_buffer_length.set("")

        #
        # Sample label frame items
        #

        COL = 0
        ROW = 5
        frm = ttk.LabelFrame(self.win, text=" Sample # ")
        frm.grid(column=COL, row=ROW, padx=5, pady=5, sticky=tk.EW)

        COL = 0
        ROW = 0
        self.sample_n = tk.StringVar()
        ttk.Label(frm, text="N:", width=LABEL_WIDTH, anchor=tk.W).grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_n, width=VALUE_WIDTH, anchor=tk.E).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        ROW += 1
        self.sample_tx = tk.StringVar()
        ttk.Label(frm, text="TX:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_tx).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sample_rx = tk.StringVar()
        ttk.Label(frm, text="RX:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_rx).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="Hz").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sample_level = tk.StringVar()
        ttk.Label(frm, text="Level:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_level).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="dB").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sample_rms = tk.StringVar()
        ttk.Label(frm, text="RMS:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_rms).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        ROW += 1
        self.sample_max = tk.StringVar()
        ttk.Label(frm, text="Max:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_max).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        ROW += 1
        self.sample_max_dB = tk.StringVar()
        ttk.Label(frm, text="Max:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_max_dB).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)
        ttk.Label(frm, text="dB").grid(column=COL+2, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        ROW += 1
        self.sample_mean = tk.StringVar()
        ttk.Label(frm, text="Mean:").grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)
        ttk.Label(frm, textvariable=self.sample_mean).grid(column=COL+1, row=ROW, padx=PADX, pady=PADY, sticky=tk.E)

        #
        # Navigation buttons
        #

        COL = 0
        ROW += 1
        frm = ttk.LabelFrame(frm, relief=tk.FLAT)
        frm.grid(column=COL, row=ROW, columnspan=3, padx=5, sticky=tk.EW)

        COL = 0
        ROW = 0
        self.sample_index = tk.IntVar()
        self.sample_scale = ttk.Scale(frm, variable=self.sample_index, orient='horizontal',
            to=self.sweep_settings.step_count-1, command=self._sample_index_set)
        self.sample_scale.grid(column=COL, row=ROW, columnspan=4, padx=PADX, sticky=tk.EW)

        ROW += 1
        self.sample_button_first = ttk.Button(frm, text="|<<", width=5, command=self._sweep_button_first)
        self.sample_button_first.grid(column=COL, row=ROW, padx=PADX, pady=5, sticky=tk.EW)

        self.sample_button_prev = ttk.Button(frm, text="<", width=5, command=self._sweep_button_prev)
        self.sample_button_prev.grid(column=COL+1, row=ROW, padx=PADX, pady=5, sticky=tk.EW)

        self.sample_button_next = ttk.Button(frm, text=">", width=5, command=self._sweep_button_next)
        self.sample_button_next.grid(column=COL+2, row=ROW, padx=PADX, pady=5, sticky=tk.EW)

        self.sample_button_last = ttk.Button(frm, text=">>|", width=5, command=self._sweep_button_last)
        self.sample_button_last.grid(column=COL+3, row=ROW, padx=PADX, pady=5, sticky=tk.EW)

        #
        # Download progressbar
        #

        COL = 0
        ROW = 14
        self.progressbar = ttk.Progressbar(self.win, orient='horizontal', mode='determinate')
        self.progressbar.grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.EW)
        self.progressbar['maximum'] = 100
        self.progressbar['value'] = 0

        #
        # Serial port parameters
        #

        COL = 0
        ROW = 16
        ttk.Label(self.win, text="Serial port: {}, Timeout: {}".format(serial_port, serial_timeout)).grid(column=COL, row=ROW, padx=PADX, pady=PADY, sticky=tk.W)

        #
        # Matplotlib time-domain graph
        #

        COL = 1
        ROW = 0
        fig = plt.Figure(figsize=(14,3))
        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.get_tk_widget().grid(column=COL, row=ROW, rowspan=5, columnspan=15, padx=PLOT_PADX, pady=PLOT_PADY, sticky=tk.EW)
        plot = fig.add_subplot(111)
        plot.set_ylim(-ADC_REF_VALUE, ADC_REF_VALUE)
        plot.set_title('Time Domain')
        self.time_domain_graph = fig
        self.time_domain_plot = plot
        self.time_domain_canvas = canvas;

        #
        # Matplotlib FFT graph
        #

        ROW += 5
        fig = plt.Figure(figsize=(14,3))
        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.get_tk_widget().grid(column=COL, row=ROW, rowspan=5, columnspan=15, padx=PLOT_PADX, pady=PLOT_PADY, sticky=tk.EW)
        plot = fig.add_subplot(111)
        plot.set_ylim(FFT_MIN_dB, FFT_MAX_dB)
        plot.set_title('FFT')
        self.fft_graph = fig
        self.fft_plot = plot
        self.fft_canvas = canvas;

        #
        # Matplotlib spectrum graph
        #

        ROW += 5
        fig = plt.Figure(figsize=(14,3))
        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.get_tk_widget().grid(column=COL, row=ROW, rowspan=5, columnspan=15, padx=PLOT_PADX, pady=PLOT_PADY, sticky=tk.EW)
        plot = fig.add_subplot(111)
        plot.set_ylim(SPECTRUM_MIN_dB, SPECTRUM_MAX_dB)
        plot.set_xlim(self.sweep_settings.start_Hz/1e6, self.sweep_settings.end_Hz/1e6)
        plot.set_title('Spectrum')
        self.spectrum_graph = fig
        self.spectrum_plot = plot
        self.spectrum_canvas = canvas;

    def _quit(self):
        self.sweep_stop()
        self.win.quit()
        self.win.destroy()
        print("Bye.")
        exit()

    def _file_new(self):
        self.sweep_run()

    @staticmethod
    def file_write_samples(f, sd):
        f.write("# Index: {}\n".format(round(sd.n)))
        f.write("# Frequency: {}\n".format(round(sd.f_Hz)))
        f.write("# Rx offset: {}\n".format(round(sd.rx_offset_Hz)))
        f.write("# Sampling rate: {}\n".format(round(sd.sample_rate_Hz, 1)))
        f.write("# Samples: {}\n".format(len(sd.samples)))
        for s in sd.samples:
            f.write("{}\n".format(s))

    def _file_save(self):
        if len(self.samples) > 0:
            n = self.sample_index.get()
            f = asksaveasfile(filetypes=[("MATLAB ascii data file", "*.dat")], mode="w")
            if f is not None:
                f.write("# LTDZ ADC SAMPLE BUFFER FILE\n")
                f.write("# Records: {}\n".format(1))
                sd = self.samples[n]
                self.file_write_samples(f, sd)
                f.close()

    def _file_save_all(self):
        if len(self.samples) > 0:
            f = asksaveasfile(filetypes=[("MATLAB ascii data file", "*.dat")], mode="w")
            if f is not None:
                f.write("# LTDZ ADC SAMPLE BUFFER FILE\n")
                f.write("# Records: {}\n".format(len(self.samples)))
                for sd in self.samples:
                    self.file_write_samples(f, sd)
                f.close()

    def _quit_shortcut(self, event):
        self._quit()

    def _file_new_shortcut(self, event):
        self._file_new()

    def _file_save_shortcut(self, event):
        self._file_save()

    def _file_save_all_shortcut(self, event):
        self._file_save_all()

    def _sweep_button_first(self):
        if len(self.samples) > 0:
            n = 0
            self.sample_index.set(n)
            sd = self.samples[n]
            self.update_sample_view(sd)

    def _sweep_button_prev(self):
        if len(self.samples) > 0:
            n = self.sample_index.get()
            if n > 0:
                n = n - 1
                self.sample_index.set(n)
            sd = self.samples[n]
            self.update_sample_view(sd)

    def _sweep_button_next(self):
        if len(self.samples) > 0:
            n = self.sample_index.get()
            if n < len(self.samples)-1:
                n = n + 1
                self.sample_index.set(n)
            sd = self.samples[n]
            self.update_sample_view(sd)

    def _sweep_button_last(self):
        if len(self.samples) > 0:
            n = len(self.samples)-1
            self.sample_index.set(n)
            sd = self.samples[n]
            self.update_sample_view(sd)

    def _sample_index_set(self, n):
        if len(self.samples) > 0:
            n = round(float(n))
            self.sample_index.set(n)
            sd = self.samples[n]
            self.update_sample_view(sd)

    def update_sweep_settings(self):
        FORMAT_FREQ = "{: >15,}"
        FORMAT_NUM  = "{: >15}"
        self.sweep_start.set(FORMAT_FREQ.format(self.sweep_settings.start_Hz))
        self.sweep_center.set(FORMAT_FREQ.format(self.sweep_settings.center_Hz))
        self.sweep_end.set(FORMAT_FREQ.format(self.sweep_settings.end_Hz))
        self.sweep_span.set(FORMAT_FREQ.format(self.sweep_settings.span_Hz))
        self.sweep_step.set(FORMAT_FREQ.format(self.sweep_settings.step_Hz))
        self.sweep_count.set(FORMAT_NUM.format(self.sweep_settings.step_count))
        self.spectrum_plot.set_xlim(self.sweep_settings.start_Hz/1e6,
            (self.sweep_settings.end_Hz-self.sweep_settings.step_Hz)/1e6)
        self.spectrum_canvas.draw()

    def gui_disable(self):
        NULL_FREQ = "{: >15}".format("-")
        NULL_NUM  = "{: >15}".format("-")
        self.file_menu.entryconfigure(MENU_FILE_NEW, state=tk.DISABLED)
        self.file_menu.unbind_all(MENU_FILE_NEW_KB)
        self.file_menu.entryconfigure(MENU_FILE_SAVE, state=tk.DISABLED)
        self.file_menu.unbind_all(MENU_FILE_SAVE_KB)
        self.file_menu.entryconfigure(MENU_FILE_SAVE_ALL, state=tk.DISABLED)
        self.file_menu.unbind_all(MENU_FILE_SAVE_ALL_KB)
        self.sampling_rx_offset.set(NULL_FREQ)
        self.sampling_step.set(NULL_FREQ)
        self.sampling_rate.set(NULL_FREQ)
        self.sampling_buffer_length.set(NULL_NUM)
        self.sample_n.set(NULL_NUM)
        self.sample_tx.set(NULL_FREQ)
        self.sample_rx.set(NULL_FREQ)
        self.sample_level.set(NULL_NUM)
        self.sample_mean.set(NULL_NUM)
        #self.sample_scale.state([tk.DISABLED])
        self.sample_button_first['state'] = tk.DISABLED
        self.sample_button_prev['state'] = tk.DISABLED
        self.sample_button_next['state'] = tk.DISABLED
        self.sample_button_last['state'] = tk.DISABLED

    def gui_enable(self):
        self.file_menu.entryconfigure(MENU_FILE_NEW, state=tk.NORMAL)
        self.file_menu.bind_all(MENU_FILE_NEW_KB, self._file_new_shortcut)
        self.file_menu.entryconfigure(MENU_FILE_SAVE, state=tk.NORMAL)
        self.file_menu.bind_all(MENU_FILE_SAVE_KB, self._file_save_shortcut)
        self.file_menu.entryconfigure(MENU_FILE_SAVE_ALL, state=tk.NORMAL)
        self.file_menu.bind_all(MENU_FILE_SAVE_ALL_KB, self._file_save_all_shortcut)
        #self.sample_scale.state([tk.ACTIVE])
        self.sample_button_first['state'] = tk.NORMAL
        self.sample_button_prev['state'] = tk.NORMAL
        self.sample_button_next['state'] = tk.NORMAL
        self.sample_button_last['state'] = tk.NORMAL

    def sweep_init(self):
        self.f_Hz = []
        self.samples = []
        self.level_dB = []
        self.sample_index.set(0)
        self.gui_disable()
        self.progressbar['value'] = 0
        self.progressbar['maximum'] = self.sweep_settings.step_count

    def sweep_poll(self):
        sd = self.q.get()
        if isinstance(sd, LtdzSweepExitStatus):
            self.sweep_stop()
            if sd.status == LtdzStatus.SUCCESS:
                pass
            elif sd.status == LtdzStatus.TIMEOUT:
                messagebox.showerror("LTDZ Timeout",
                    "Fatal serial port '{}' communication timeout.".format(serial_port))
            elif sd.status == LtdzStatus.ERROR:
                messagebox.showerror("LTDZ Protocol Error",
                    sd.message)
            elif sd.status == LtdzStatus.FAIL:
                messagebox.showerror("LTDZ Serial Port Error",
                    sd.message)
            else:
                messagebox.showerror("LTDZ Unknown Error",
                    "Error code: {}".format(sd.status))
        else:
            sd.rms = np.sqrt(np.mean(np.array(sd.samples)**2))
            sd.level_dB = 20.0*np.log10(sd.rms/ADC_REF_VALUE)
            sd.max = np.max(np.abs(sd.samples))
            sd.max_dB = 20.0*np.log10(sd.max/ADC_REF_VALUE)
            sd.mean = np.mean(sd.samples)
            self.f_Hz.append(sd.f_Hz/1e6)
            self.samples.append(sd)
            self.level_dB.append(sd.level_dB)
            self.progressbar['value'] = sd.n+1
            self.update_sample_view(sd)
            self.win.after(10, self.sweep_poll)

    def sweep_run(self):
        self.sweep_init()
        self.q = Queue()
        config = LtdzSweepConfig()
        config.serial_port = serial_port
        config.serial_baud = serial_baud
        config.serial_timeout = serial_timeout
        config.start_Hz = self.sweep_settings.start_Hz
        config.step_Hz = self.sweep_settings.step_Hz
        config.step_count = self.sweep_settings.step_count
        self.ltdz_thread = Thread(target=ltdz_sweep_run, args=(self.q, config,))
        self.ltdz_thread.start()
        self.win.after(10, self.sweep_poll)

    def sweep_stop(self):
        self.q = None
        self.gui_enable()
        self._sweep_button_first()

    def update_sample_view(self, sd):
        FORMAT_FREQ = "{: >15,}"
        FORMAT_NUM  = "{: >15}"
        FORMAT_LEVEL = "{: >15.2f}"

        self.sample_index.set(sd.n)

        # Sampling configuration
        self.sampling_rx_offset.set(FORMAT_FREQ.format(sd.rx_offset_Hz))
        self.sampling_step.set(FORMAT_FREQ.format(sd.step_Hz))
        self.sampling_rate.set(FORMAT_NUM.format(round(sd.sample_rate_Hz, 1)))
        self.sampling_buffer_length.set(FORMAT_NUM.format(sd.buffer_size))

        # Sample info
        self.sample_n.set(FORMAT_NUM.format(sd.n))
        self.sample_tx.set(FORMAT_FREQ.format(sd.f_Hz))
        self.sample_rx.set(FORMAT_FREQ.format(sd.rx_offset_Hz))
        self.sample_rms.set(FORMAT_LEVEL.format(sd.rms))
        self.sample_level.set(FORMAT_LEVEL.format(sd.level_dB))
        self.sample_max.set(FORMAT_LEVEL.format(sd.max))
        self.sample_max_dB.set(FORMAT_LEVEL.format(sd.max_dB))
        self.sample_mean.set(FORMAT_LEVEL.format(sd.mean))

        # Update time-domain samples
        self.time_domain_plot.clear()
        self.time_domain_plot.plot(sd.samples)
        #self.time_domain_plot.set_ylim(-ADC_REF_VALUE, ADC_REF_VALUE)
        self.time_domain_plot.set_title('Time Domain')
        self.time_domain_canvas.draw()

        # Update FFT
        f_Hz = np.fft.fftshift(np.fft.fftfreq(sd.buffer_size, 1.0/sd.sample_rate_Hz))
        fft = np.abs(np.fft.fftshift(np.fft.fft(sd.samples * np.hanning(sd.buffer_size)))) / len(sd.samples)
        fft_dB = 20.0*np.log10(fft/ADC_REF_VALUE)
        self.fft_plot.clear()
        self.fft_plot.plot(f_Hz, fft_dB)
        self.fft_plot.set_ylim(FFT_MIN_dB, FFT_MAX_dB)
        self.fft_plot.set_title('FFT')
        self.fft_canvas.draw()

        # Update spectrum
        self.spectrum_plot.clear()
        self.spectrum_plot.plot(self.f_Hz, self.level_dB)
        self.spectrum_plot.set_xlim(self.sweep_settings.start_Hz/1e6,
            (self.sweep_settings.end_Hz-self.sweep_settings.step_Hz)/1e6)
        self.spectrum_plot.set_ylim(SPECTRUM_MIN_dB, SPECTRUM_MAX_dB)
        self.spectrum_plot.set_title('Spectrum')
        self.spectrum_plot.axvline(x=sd.f_Hz/1e6, color='red')
        self.spectrum_canvas.draw()

#
# Sweep settings
#

class SweepSettings:
    def __init__(self, args):
        self.start_Hz = self.kmg_to_num(args.start_Hz)
        self.center_Hz = self.kmg_to_num(args.center_Hz)
        self.end_Hz = self.kmg_to_num(args.end_Hz)
        self.span_Hz = self.kmg_to_num(args.span_Hz)
        self.step_Hz = self.kmg_to_num(args.step_Hz)
        self.step_count = self.kmg_to_num(args.step_count)

    @staticmethod
    def kmg_to_num(v):
        if v is None:
            return None
        v = v.lower()
        scale = 1
        if "k" in v:
            scale = 1e3
            v = v.replace("k", ".")
        if "m" in v:
            scale = 1e6
            v = v.replace("m", ".")
        if "g" in v:
            scale = 1e9
            v = v.replace("g", ".")
        if "." in v:
            v=round(float(v)*scale)
        else:
            v=int(v)
        return v

    def parse(self):
        if self.start_Hz is not None:
            # Start frequency was given
            if self.end_Hz is not None:
                if self.end_Hz <= self.start_Hz:
                    print("End frequency must be larger than start frequency.")
                    exit(1)
                if self.span_Hz is not None:
                    print("Span not allowed when the start and the end frequency specified.")
                    exit(1)
                if self.center_Hz is not None:
                    print("Center frequency not allowed when the start and the end frequency specified.")
                    exit(1)
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when the start and the end frequency specified.")
                    exit(1)
                self.span_Hz = self.end_Hz - self.start_Hz
            elif self.span_Hz is not None:
                if self.center_Hz is not None:
                    print("Center frequency not allowed when the start frequency and span specified.")
                    exit(1)
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when the start frequency and span specified.")
                    exit(1)
            elif self.center_Hz is not None:
                if self.center_Hz <= self.start_Hz:
                    print("Center frequency must be larger than start frequency.")
                    exit(1)
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when the start frequency and center frequency specified.")
                    exit(1)
                self.span_Hz = 2*(self.center_Hz - self.start_Hz)
            elif self.step_Hz is not None and self.step_count is not None:
                self.span_Hz = self.step_Hz * self.step_count
            else:
                print("Could not solve the start frequency and the frequency span.")
                exit(1)
        elif self.end_Hz is not None:
            # End frequency was given
            if self.span_Hz is not None:
                if self.center_Hz is not None:
                    print("Center frequency not allowed when the end frequency and span specified.")
                    exit(1)
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when the end frequency and span specified.")
                    exit(1)
            elif self.center_Hz is not None:
                if self.end_Hz <= self.center_Hz:
                    print("Center frequency must be smaller than end frequency.")
                    exit(1)
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when the end frequency and center frequency specified.")
                    exit(1)
                self.span_Hz = 2*(self.end_Hz - self.center_Hz)
            elif self.step_Hz is not None and self.step_count is not None:
                self.span_Hz = self.step_Hz * self.step_count
            else:
                print("Could not solve the the start frequency and frequency span.")
                exit(1)
            self.start_Hz = self.end_Hz - self.span_Hz
        elif self.center_Hz is not None:
            # Center frequency was given
            if self.span_Hz is not None:
                if self.step_Hz is not None and self.step_count is not None:
                    print("Only step size or step count allowed when center frequency and span specified.")
                    exit(1)
            elif self.step_Hz is not None and self.step_count is not None:
                self.span_Hz = self.step_Hz * self.step_count
            else:
                print("Could not solve for start frequency and frequency span.")
                exit(1)
            self.start_Hz = round(self.center_Hz - self.span_Hz/2)
        else:
            print("Missing start, end or center frequency.")
            exit(1)

        assert(self.start_Hz is not None)
        assert(self.span_Hz is not None)

        self.end_Hz = self.start_Hz + self.span_Hz

        if self.start_Hz < MIN_Hz:
            print("Minimum start frequency is", MIN_Hz)
            exit(1)

        if self.end_Hz >= MAX_Hz:
            print("Maximum end frequency is", MAX_Hz)
            exit(1)

        if self.step_Hz is None and self.step_count is None:
            # Step size or step count is required
            print("Step size or step count need to be specified.")
            exit(1)

        if self.step_Hz is not None:
            self.step_count = round(self.span_Hz / self.step_Hz)

        if self.step_count is not None:
            self.step_Hz = round(self.span_Hz / self.step_count)

        if self.step_Hz < MIN_STEP_Hz:
            print("Warning: Minimum allowed step size is", MIN_STEP_Hz)
            self.step_Hz = round(MIN_STEP_Hz)
            self.step_count = round(self.span_Hz / self.step_Hz)

        if self.step_Hz > MAX_STEP_Hz:
            print("Warning: Maximum allowed step size is", MAX_STEP_Hz)
            self.step_Hz = round(MAX_STEP_Hz)
            self.step_count = round(self.span_Hz / self.step_Hz)

        if self.step_count < MIN_STEP_COUNT:
            print("Warning: Minimum allowed step count is", MIN_STEP_COUNT)
            self.step_count = MIN_STEP_COUNT
            self.span_Hz = self.step_Hz * self.step_count

        if self.step_count > MAX_STEP_COUNT:
            print("Warning: Maximum allowed step count is", MAX_STEP_COUNT)
            self.step_count = MAX_STEP_COUNT
            self.span_Hz = self.step_Hz * self.step_count

        self.end_Hz = self.start_Hz + self.span_Hz
        self.center_Hz = round((self.start_Hz + self.end_Hz) / 2)

        return self

#
# main starts here
#

parser = argparse.ArgumentParser(description='LTDZ ADC sample buffer reader.')
parser.add_argument('-p', "--port",
    dest='port',
    default=SERIAL_PORT,
    help="Serial port")
parser.add_argument('-b', "--baud",
    dest='baud',
    type=int,
    default=SERIAL_BAUD,
    help="Serial port baud rate")
parser.add_argument('-t', "--timeout",
    dest='timeout',
    type=int,
    default=SERIAL_TIMEOUT,
    help="Serial port timeout value in seconds")
parser.add_argument('--start',
    dest='start_Hz',
    default=None,
    type=str,
    help="Start frequency [Hz]")
parser.add_argument('--end',
    dest='end_Hz',
    default=None,
    type=str,
    help="End frequency [Hz]")
parser.add_argument('--center',
    dest='center_Hz',
    default=None,
    type=str,
    help="Center frequency [Hz]")
parser.add_argument('--span',
    dest='span_Hz',
    default=None,
    type=str,
    help="Frequency span [Hz]")
parser.add_argument('--step',
    dest='step_Hz',
    default=None,
    type=str,
    help="Step size [Hz]")
parser.add_argument('--count',
    dest='step_count',
    default=None,
    type=str,
    help="Step count")

args = parser.parse_args()

serial_port     = args.port
serial_baud     = args.baud
serial_timeout  = args.timeout

sweep = SweepSettings(args).parse()

win = tk.Tk()
app = Application(win=win, sweep_settings=sweep)
app.mainloop()
