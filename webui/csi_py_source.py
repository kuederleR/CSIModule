#!/usr/bin/env python3
# -*-coding:utf-8-*-

import csv
import json
import numpy as np
import serial
from io import StringIO
import time

CSI_DATA_INDEX = 200  # buffer size
CSI_DATA_COLUMNS = 490
DATA_COLUMNS_NAMES_C5C6 = [
    "type", "id", "mac", "rssi", "rate", "noise_floor", "fft_gain", "agc_gain",
    "channel", "local_timestamp", "sig_len", "rx_state", "len", "first_word", "data"
]
DATA_COLUMNS_NAMES = [
    "type", "id", "mac", "rssi", "rate", "sig_mode", "mcs", "bandwidth", "smoothing",
    "not_sounding", "aggregation", "stbc", "fec_coding", "sgi", "noise_floor",
    "ampdu_cnt", "channel", "secondary_channel", "local_timestamp", "ant", "sig_len",
    "rx_state", "len", "first_word", "data"
]

class CSIData:
    def __init__(self, amplitude, phase, raw, meta):
        self.amplitude = amplitude
        self.phase = phase
        self.raw = raw
        self.meta = meta

class CSIReader:
    def __init__(self, port, data_callback=None, rate=None, detect_static_samples=10, dynamic_filter=True):
        self.port = port
        self.csi_data_complex = np.zeros([CSI_DATA_INDEX, CSI_DATA_COLUMNS], dtype=np.complex64)
        self.data_callback = data_callback if data_callback is not None else self.default_callback
        self._stop = False
        self.rate = rate  # in Hz
        self.detect_static_samples = detect_static_samples
        self._static_detection_done = False
        self._static_raw_history = []
        self._dynamic_indices = None
        self._dynamic_filter = dynamic_filter

    def default_callback(self, data):
        pass

    def stop(self):
        self._stop = True

    def run(self):
        ser = serial.Serial(port=self.port, baudrate=921600, bytesize=8, parity='N', stopbits=1)
        if ser.isOpen():
            print("open success")
        else:
            print("open failed")
            return
        last_process = time.time()
        interval = 1.0 / self.rate if self.rate else 0
        try:
            while not self._stop:
                strings = str(ser.readline())
                if not strings:
                    break
                strings = strings.lstrip('b\'').rstrip('\\r\\n\'')
                index = strings.find('CSI_DATA')
                if index == -1:
                    continue
                csv_reader = csv.reader(StringIO(strings))
                csi_data = next(csv_reader)
                csi_data_len = int(csi_data[-3])
                if len(csi_data) != len(DATA_COLUMNS_NAMES) and len(csi_data) != len(DATA_COLUMNS_NAMES_C5C6):
                    continue
                if len(csi_data) == len(DATA_COLUMNS_NAMES):
                    meta = {
                        "channel": csi_data[15],
                        "bandwidth": csi_data[7],
                        "noise_floor": csi_data[13],
                        "rate_index": csi_data[6],
                        "rssi": csi_data[3],
                        "mac": csi_data[2]
                    }
                elif len(csi_data) == len(DATA_COLUMNS_NAMES_C5C6):
                    meta = {
                        "channel": csi_data[8],
                        "bandwidth": None,
                        "noise_floor": csi_data[5],
                        "rate_index": csi_data[4],
                        "rssi": csi_data[3],
                        "mac": csi_data[2]
                    }
                try:
                    csi_raw_data = json.loads(csi_data[-1])
                except json.JSONDecodeError:
                    continue
                if csi_data_len != len(csi_raw_data):
                    continue

                if self._dynamic_filter:
                    # --- Static index detection ---
                    if not self._static_detection_done:
                        self._static_raw_history.append(list(csi_raw_data))
                        if len(self._static_raw_history) >= self.detect_static_samples:
                            arr = np.array(self._static_raw_history)
                            # Find indices where all values are the same across samples
                            static_mask = np.all(arr == arr[0], axis=0)
                            self._dynamic_indices = np.where(~static_mask)[0]
                            self._static_detection_done = True
                            print(f"Detected {len(self._dynamic_indices)} dynamic indices out of {len(static_mask)} total.")
                        continue  # Don't process until detection is done

                    # Only use dynamic indices for processing
                    filtered_raw = [csi_raw_data[i] for i in self._dynamic_indices]
                    filtered_len = len(filtered_raw)
                    # Always keep buffer up to date
                    self.csi_data_complex[:-1] = self.csi_data_complex[1:]
                    for i in range(filtered_len // 2):
                        self.csi_data_complex[-1][i] = complex(
                            filtered_raw[i * 2 + 1], filtered_raw[i * 2]
                        )
                    now = time.time()
                    if not self.rate or (now - last_process) >= interval:
                        amplitude = np.abs(self.csi_data_complex[-1, :filtered_len // 2])
                        phase = np.angle(self.csi_data_complex[-1, :filtered_len // 2])
                        data = CSIData(amplitude, phase, filtered_raw, meta)
                        self.data_callback(data)
                        last_process = now
                else:
                    # No dynamic filtering, use all data
                    raw_len = len(csi_raw_data)
                    self.csi_data_complex[:-1] = self.csi_data_complex[1:]
                    for i in range(raw_len // 2):
                        self.csi_data_complex[-1][i] = complex(
                            csi_raw_data[i * 2 + 1], csi_raw_data[i * 2]
                        )
                    now = time.time()
                    if not self.rate or (now - last_process) >= interval:
                        amplitude = np.abs(self.csi_data_complex[-1, :raw_len // 2])
                        phase = np.angle(self.csi_data_complex[-1, :raw_len // 2])
                        data = CSIData(amplitude, phase, csi_raw_data, meta)
                        self.data_callback(data)
                        last_process = now
        finally:
            ser.close()