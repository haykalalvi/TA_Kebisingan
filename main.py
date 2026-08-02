#!/usr/bin/env python3
"""
Integrated Noise Monitoring System
ESP32-S3 (audio via USB) + GPS NEO-6M (location via GPIO UART)
"""

import multiprocessing as mp
import serial
import struct
import numpy as np
import time
import logging
import gc
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import json
import math

try:
    import pynmea2
    PYNMEA2_AVAILABLE = True
except ImportError:
    PYNMEA2_AVAILABLE = False
    print("[WARNING] pynmea2 not installed. GPS disabled. pip3 install pynmea2")


# CONFIGURATION


UART_PORT          = '/dev/ttyACM0'
UART_BAUD          = 115200
SAMPLE_RATE        = 48000
SAMPLES_SHORT      = 6000
PACKETS_PER_SECOND = 8

FRAME_START_MARKER = 0xAA55AA55
FRAME_END_MARKER   = 0x55AA55AA

GPS_PORT                = '/dev/ttyAMA0'
GPS_BAUD                = 9600
GPS_UPDATE_INTERVAL_SEC = 300
GPS_READ_TIMEOUT_SEC    = 90
GPS_ENABLED             = PYNMEA2_AVAILABLE

OCTAVE_FRACTION    = 3
OCTAVE_ORDER       = 6
FREQ_LIMITS        = [20, 20000]

MIC_SENSITIVITY    = -26         # dBFS
MIC_REF_DB         = 94.0         # dB SPL reference (1 Pa)
MIC_OFFSET_DB      = 0.0 #3.0103       # 2-channel correction (keep if mono: set 0)
MIC_BITS           = 24
MIC_REF_AMPL       = pow(10, MIC_SENSITIVITY / 20) * ((1 << (MIC_BITS - 1)) - 1)

WEIGHTING_A        = 'A'
WEIGHTING_C        = 'C'

DISPLAY_MODE       = 'second'     # 'second' or 'minute'
SECONDS_PER_MINUTE = 60	

if DISPLAY_MODE not in ('second', 'minute'):
    raise ValueError(f"Invalid DISPLAY_MODE '{DISPLAY_MODE}'.")

# Serial reconnect
SERIAL_RECONNECT_DELAY_SEC = 2    # seconds to wait before reopening USB port

# Frame layout
HEADER_FMT  = '<II5f' 
HEADER_SIZE = struct.calcsize(HEADER_FMT)                    # 24 bytes
FRAME_SIZE  = HEADER_SIZE + (SAMPLES_SHORT * 4) + 4         # 24032 bytes

RAW_QUEUE_SIZE    = 50
RESULT_QUEUE_SIZE = 20

PLOT_SECONDS_RAW = [30, 50]

MQTT_HOST  = 'localhost'
MQTT_PORT  = 1883
MQTT_TOPIC = "kebisingan/alat1"

# Leq diagnostic: print raw RMS / peak / Leq every N seconds (0 = disable)
LEQ_DIAGNOSTIC_INTERVAL = 5


# HELPERS

def _resolve_plot_seconds(plot_seconds, display_mode, secs_per_min):
    if display_mode == 'second':
        return list(plot_seconds)
    snapped = [max(secs_per_min, round(s / secs_per_min) * secs_per_min)
               for s in plot_seconds]
    if snapped != list(plot_seconds):
        logging.warning(f"PLOT_SECONDS snapped to minute boundaries → {snapped}")
    seen = set()
    return [x for x in snapped if not (x in seen or seen.add(x))]

PLOT_SECONDS = _resolve_plot_seconds(PLOT_SECONDS_RAW, DISPLAY_MODE, SECONDS_PER_MINUTE)

A_WEIGHTING_CORRECTIONS = np.array([
    -50.5, -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9,  -8.6,  -6.6,  -4.8,  -3.2,  -1.9,  -0.8,   0.0,   0.6,   1.0,
      1.2,   1.3,   1.2,   1.0,   0.5,  -0.1,  -1.1,  -2.5,  -4.3,  -6.6, -9.3
])
C_WEIGHTING_CORRECTIONS = np.array([
    -6.2, -4.4, -3.0, -2.0, -1.3, -0.8, -0.5, -0.3, -0.2, -0.1,
     0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
     0.0,  0.0,  0.0, -0.1, -0.2, -0.3, -0.5, -0.8, -1.3, -2.0, -3.0
])

WEIGHTING_DICT = {'A': A_WEIGHTING_CORRECTIONS, 'C': C_WEIGHTING_CORRECTIONS}


# SHARED GPS STATE

def make_gps_state():
    gps_array         = mp.Array('d', [0.0, 0.0, 0.0])
    gps_lock          = mp.Lock()
    gps_last_fix_time = mp.Value('d', 0.0)
    return gps_array, gps_lock, gps_last_fix_time

def gps_set(gps_array, gps_lock, gps_last_fix_time, lat, lon):
    with gps_lock:
        gps_array[0] = lat
        gps_array[1] = lon
        gps_array[2] = 1.0
        gps_last_fix_time.value = time.time()

def gps_get(gps_array, gps_lock):
    with gps_lock:
        return gps_array[0], gps_array[1], bool(gps_array[2])


# RING BUFFER

class RingBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer   = bytearray(capacity)
        self.head     = 0
        self.tail     = 0
        self.size     = 0

    def extend(self, data):
        data_len = len(data)
        if data_len >= self.capacity:
            self.buffer[:] = data[-self.capacity:]
            self.head = 0; self.tail = 0; self.size = self.capacity
            return
        if self.tail + data_len <= self.capacity:
            self.buffer[self.tail:self.tail + data_len] = data
        else:
            first_part = self.capacity - self.tail
            self.buffer[self.tail:]             = data[:first_part]
            self.buffer[:data_len - first_part] = data[first_part:]
        self.tail = (self.tail + data_len) % self.capacity
        self.size = min(self.size + data_len, self.capacity)

    def get_bytes(self, length):
        if length > self.size:
            return None
        if self.head + length <= self.capacity:
            return bytes(self.buffer[self.head:self.head + length])
        first_part = self.capacity - self.head
        return bytes(self.buffer[self.head:]) + bytes(self.buffer[:length - first_part])

    def consume(self, length):
        self.head = (self.head + length) % self.capacity
        self.size -= length

    def find(self, pattern):
        if self.head < self.tail:
            data = self.buffer[self.head:self.tail]
        else:
            data = bytes(self.buffer[self.head:]) + bytes(self.buffer[:self.tail])
        idx = data.find(pattern)
        return idx if idx != -1 else -1

    def __len__(self):
        return self.size


# PROCESS 0: GPS READER

def gps_reader_process(
    gps_array, gps_lock, gps_last_fix_time, stop_event,
    update_interval=GPS_UPDATE_INTERVAL_SEC,
    read_timeout=GPS_READ_TIMEOUT_SEC,
):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [GPS] %(message)s"
    )
    logger = logging.getLogger("gps_reader")

    if not GPS_ENABLED:
        logger.warning("GPS disabled (GPS_ENABLED=False or pynmea2 not installed).")
        return

    try:
        import pynmea2
    except ImportError:
        logger.error("pynmea2 not installed. GPS disabled. Run: pip install pynmea2")
        return

    logger.info(
        f"GPS reader started | port={GPS_PORT} baud={GPS_BAUD} | "
        f"mode=continuous (NEO-6M)"
    )

    ser = None

    def open_port():
        nonlocal ser
        try:
            if ser and ser.is_open:
                return True
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
            logger.info(f"Serial port {GPS_PORT} opened")
            return True
        except serial.SerialException as e:
            logger.error(f"Cannot open {GPS_PORT}: {e}. Retry in 10s...")
            ser = None
            return False

    def close_port():
        nonlocal ser
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass
        ser = None

    # Initial port open — retry until success
    while not stop_event.is_set() and not open_port():
        time.sleep(10)

    fix_count = 0

    while not stop_event.is_set():
        # Ensure port is open
        if not ser or not ser.is_open:
            if not open_port():
                time.sleep(10)
                continue

        try:
            raw = ser.readline()
        except serial.SerialException as e:
            logger.warning(f"Serial read error: {e}. Re-opening port...")
            close_port()
            time.sleep(2)
            continue

        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue

        if not line.startswith("$"):
            continue

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            continue

        # GGA sentence — has fix quality indicator
        if isinstance(msg, pynmea2.types.talker.GGA):
            if msg.gps_qual and int(msg.gps_qual) > 0:
                try:
                    lat = float(msg.latitude)
                    lon = float(msg.longitude)
                except (ValueError, TypeError):
                    continue

                gps_set(gps_array, gps_lock, gps_last_fix_time, lat, lon)
                fix_count += 1

                # Log every 60 fixes (~1 per minute) to avoid spamming
                if fix_count == 1 or fix_count % 60 == 0:
                    logger.info(
                        f"Fix #{fix_count} | "
                        f"lat={lat:.6f} lon={lon:.6f} | "
                        f"sats={msg.num_sats} alt={msg.altitude}m"
                    )

    close_port()
    logger.info("GPS reader stopped.")


# PROCESS 1: SERIAL READER  

def serial_reader_process(
    raw_queue, stat_pkt_received, stat_pkt_corrupted, stat_pkt_dropped, stop_event,
):
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [Reader] %(message)s')
    logger = logging.getLogger()

    start_marker_bytes = struct.pack('<I', FRAME_START_MARKER)

    # ── Outer reconnect loop ─────────────────────────────────────────────────
    # Instead of dying on a USB disconnect, we log the error, wait briefly,
    # and reopen the port. This keeps the process alive indefinitely.
    while not stop_event.is_set():
        ser = None
        try:
            logger.info(f"Opening {UART_PORT} ...")
            ser    = serial.Serial(UART_PORT, UART_BAUD, timeout=0)
            buffer = RingBuffer(capacity=FRAME_SIZE * 10)
            logger.info(f"Audio USB connected | frame={FRAME_SIZE}B")

            # ── Inner read loop ──────────────────────────────────────────────
            while not stop_event.is_set():
                try:
                    chunk = ser.read(8192)
                except serial.SerialException as e:
                    logger.warning(f"USB read error: {e}")
                    logger.warning("ESP32 disconnected — reconnecting in "
                                   f"{SERIAL_RECONNECT_DELAY_SEC}s...")
                    break 

                if not chunk:
                    time.sleep(0.001)
                    continue

                buffer.extend(chunk)

                while len(buffer) >= FRAME_SIZE:
                    first_bytes = buffer.get_bytes(4)
                    if first_bytes != start_marker_bytes:
                        idx = buffer.find(start_marker_bytes)
                        if idx == -1:
                            buffer.consume(max(0, len(buffer) - 3))
                            break
                        buffer.consume(idx)
                        continue

                    frame_bytes = buffer.get_bytes(FRAME_SIZE)
                    if frame_bytes is None:
                        break

                    end_marker = struct.unpack('<I', frame_bytes[-4:])[0]
                    if end_marker != FRAME_END_MARKER:
                        with stat_pkt_corrupted.get_lock():
                            stat_pkt_corrupted.value += 1
                        buffer.consume(1)
                        continue

                    try:
                        header      = struct.unpack(HEADER_FMT, frame_bytes[:HEADER_SIZE])
                        calib       = header[4]
                        batt        = header[5]
                        calib_mode  = header[6]

                        if math.isnan(batt):
                            batt = 0.0

                        samples_end = HEADER_SIZE + SAMPLES_SHORT * 4
                        raw_samples = np.frombuffer(
                            frame_bytes[HEADER_SIZE:samples_end], dtype=np.float32
                        ).copy()

                        if calib == 0.0:
                            logger.warning(
                                "calib=0.0 received from ESP32 — using 1.0. "
                                "Check NVS or calibration firmware."
                            )
                            calib = 1.0

                        packet = {
                            'raw_samples': raw_samples,
                            'calib':       float(calib),
                            'batt':        float(batt),
                            'calib_mode': int(round(calib_mode)),   
                        }

                        try:
                            raw_queue.put_nowait(packet)
                            with stat_pkt_received.get_lock():
                                stat_pkt_received.value += 1
                        except Exception:
                            with stat_pkt_dropped.get_lock():
                                stat_pkt_dropped.value += 1

                    except Exception as e:
                        logger.debug(f"Frame parse error: {e}")
                        with stat_pkt_corrupted.get_lock():
                            stat_pkt_corrupted.value += 1

                    buffer.consume(FRAME_SIZE)

        except serial.SerialException as e:
            logger.error(f"Cannot open {UART_PORT}: {e}")
        except Exception as e:
            logger.error(f"Unexpected reader error: {e}")
        finally:
            if ser and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass

        if not stop_event.is_set():
            logger.info(f"Reconnecting in {SERIAL_RECONNECT_DELAY_SEC}s...")
            time.sleep(SERIAL_RECONNECT_DELAY_SEC)

    logger.info("Serial Reader stopped.")


# FILTER BANK

class OctaveFilterBank:
    def __init__(self, fs, fraction, order, limits):
        from octave_filter import getansifrequencies, _downsamplingfactor

        freq, freq_d, freq_u = getansifrequencies(fraction, limits)
        nyquist   = fs / 2
        valid_idx = [i for i, f in enumerate(freq_u) if f < nyquist]

        self.freq   = [freq[i]   for i in valid_idx]
        freq_d      = [freq_d[i] for i in valid_idx]
        freq_u      = [freq_u[i] for i in valid_idx]
        self.factor = _downsamplingfactor(freq_u, fs)

        self.sos_filters = []
        for lower, upper, fac in zip(freq_d, freq_u, self.factor):
            fsd = fs / fac
            sos = signal.butter(
                N=order,
                Wn=np.array([lower, upper]) / (fsd / 2),
                btype='bandpass', analog=False, output='sos',
            )
            self.sos_filters.append(sos)

        self._resample_factors = [int(round(f)) for f in self.factor]

    def filter_signal_to_leq_bands(self, x):
        leq_bands = np.zeros(len(self.freq))
        for idx, (sos, down) in enumerate(zip(self.sos_filters,
                                              self._resample_factors)):
            sd  = signal.resample_poly(x, up=1, down=down) if down > 1 else x
            y   = signal.sosfilt(sos, sd)
            rms = np.sqrt(np.mean(y ** 2))
            leq_bands[idx] = (
                MIC_OFFSET_DB + MIC_REF_DB + 20 * np.log10(rms / MIC_REF_AMPL)
                if rms > 1e-10 else 0.0
            )
        return leq_bands


# LEQ AVERAGING

def _leq_average(leq_array_db, n):
    linear      = 10 ** (leq_array_db / 10)
    linear_mean = np.sum(linear, axis=0) / n
    linear_mean = np.where(linear_mean > 1e-10, linear_mean, 1e-10)
    return 10 * np.log10(linear_mean)


# PROCESS 2: OCTAVE ANALYZER 

def octave_leq_analyzer_process(
    raw_queue, result_queue, weighting_a, weighting_c,
    display_mode, stat_second, stop_event,
):
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [Analyzer] %(message)s')
    logger = logging.getLogger()

    logger.info(f"Analyzer started | mode={display_mode}")
    logger.info(f"MIC_SENSITIVITY  = {MIC_SENSITIVITY} dBFS")
    logger.info(f"MIC_REF_AMPL     = {MIC_REF_AMPL:.1f}  "
                f"(expected RMS of a 94 dBSPL / 1 Pa sine wave)")
    logger.info(f"MIC_BITS         = {MIC_BITS}")
    logger.info(f"MIC_OFFSET_DB    = {MIC_OFFSET_DB}")
    logger.info(f"MIC_REF_DB       = {MIC_REF_DB}")

    filter_bank             = OctaveFilterBank(SAMPLE_RATE, OCTAVE_FRACTION,
                                               OCTAVE_ORDER, FREQ_LIMITS)
    weighting_corrections_a = WEIGHTING_DICT[weighting_a]
    weighting_corrections_c = WEIGHTING_DICT[weighting_c]

    packet_counter  = 0
    all_samples     = np.zeros(SAMPLES_SHORT * PACKETS_PER_SECOND, dtype=np.float32)
    second_count    = 1
    current_batt    = 0.0
    last_diag_second = 0

    n_bands = len(filter_bank.freq)
    minute_buffer_bands            = np.zeros((SECONDS_PER_MINUTE, n_bands),  dtype=np.float64)
    minute_buffer_total_raw        = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_buffer_total_weighted_a = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_buffer_total_weighted_c = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_second_idx = 0

    while not stop_event.is_set():
        try:
            packet = raw_queue.get(timeout=1.0)
        except Exception:
            continue

        raw_samples  = packet['raw_samples']
        calib        = packet['calib']
        samples      = raw_samples * calib
        current_batt = packet['batt']
        current_mode = packet.get('calib_mode',0)

        active_display_mode = 'second' if current_mode == 1 else 'minute'

        start_idx = packet_counter * SAMPLES_SHORT
        all_samples[start_idx:start_idx + SAMPLES_SHORT] = samples
        packet_counter += 1

        if packet_counter < PACKETS_PER_SECOND:
            continue

        # 1 second Leq 
        leq_bands_raw        = filter_bank.filter_signal_to_leq_bands(all_samples)
        leq_bands_weighted_a = leq_bands_raw + weighting_corrections_a
        leq_bands_weighted_c = leq_bands_raw + weighting_corrections_c

        linear_sum_w_a       = np.sum(10 ** (leq_bands_weighted_a / 10))
        leq_total_weighted_a = 10 * np.log10(linear_sum_w_a) if linear_sum_w_a > 1e-10 else 0.0

        linear_sum_w_c       = np.sum(10 ** (leq_bands_weighted_c / 10))
        leq_total_weighted_c = 10 * np.log10(linear_sum_w_c) if linear_sum_w_c > 1e-10 else 0.0

        linear_sum_r  = np.sum(10 ** (leq_bands_raw / 10))
        leq_total_raw = 10 * np.log10(linear_sum_r) if linear_sum_r > 1e-10 else 0.0

        # Diagnostic logging 
        # Prints every LEQ_DIAGNOSTIC_INTERVAL seconds so can trace value
        if (LEQ_DIAGNOSTIC_INTERVAL > 0 and
                second_count - last_diag_second >= LEQ_DIAGNOSTIC_INTERVAL):
            last_diag_second = second_count
            rms_raw  = float(np.sqrt(np.mean(raw_samples ** 2)))
            rms_cal  = float(np.sqrt(np.mean(all_samples ** 2)))
            peak_cal = float(np.max(np.abs(all_samples)))
            logger.info(
                f"[DIAG s={second_count}] "
                f"calib={calib:.4f}  "
                f"rms_raw={rms_raw:.1f}  "
                f"rms_calibrated={rms_cal:.1f}  "
                f"peak_calibrated={peak_cal:.1f}  "
                f"MIC_REF_AMPL={MIC_REF_AMPL:.1f}  "
                f"LZeq={leq_total_raw:.1f}dB  "
                f"LAeq={leq_total_weighted_a:.1f}dB  "
                f"LCeq={leq_total_weighted_c:.1f}dB"
            )

        timestamp = time.strftime("%H:%M:%S")

        if active_display_mode == 'second':
            minute_second_idx = 0
            result = {
                'leq_total_weighted_a': float(leq_total_weighted_a),
                'leq_total_weighted_c': float(leq_total_weighted_c),
                'leq_total_raw':        float(leq_total_raw),
                'leq_bands_raw':        leq_bands_raw.tolist(),
                'freq_bands':           filter_bank.freq,
                'display_mode':         'second',
                'timestamp':            timestamp,
                'batt':                 float(current_batt),
                'second':               second_count,
                'label':                f"Second #{second_count}",
                'gain':                 calib,
                'calib_mode':           int(current_mode)
            }
            try:
                result_queue.put_nowait(result)
            except Exception:
                pass

        elif active_display_mode == 'minute':
            minute_buffer_bands[minute_second_idx]            = leq_bands_raw
            minute_buffer_total_raw[minute_second_idx]        = leq_total_raw
            minute_buffer_total_weighted_a[minute_second_idx] = leq_total_weighted_a
            minute_buffer_total_weighted_c[minute_second_idx] = leq_total_weighted_c
            minute_second_idx += 1

            if minute_second_idx >= SECONDS_PER_MINUTE:
                minute_count           = second_count // SECONDS_PER_MINUTE
                leq_1min_bands_raw     = _leq_average(minute_buffer_bands, SECONDS_PER_MINUTE)
                leq_1min_total_raw     = float(_leq_average(minute_buffer_total_raw, SECONDS_PER_MINUTE))
                leq_1min_total_w_a     = float(_leq_average(minute_buffer_total_weighted_a, SECONDS_PER_MINUTE))
                leq_1min_total_w_c     = float(_leq_average(minute_buffer_total_weighted_c, SECONDS_PER_MINUTE))

                result = {
                    'leq_total_weighted_a': leq_1min_total_w_a,
                    'leq_total_weighted_c': leq_1min_total_w_c,
                    'leq_total_raw':        leq_1min_total_raw,
                    'leq_bands_raw':        leq_1min_bands_raw.tolist(),
                    'freq_bands':           filter_bank.freq,
                    'display_mode':         'minute',
                    'timestamp':            timestamp,
                    'batt':                 float(current_batt),
                    'second':               second_count,
                    'minute':               minute_count,
                    'label':                f"Minute #{minute_count} (second #{second_count})",
                    'gain':                 calib,
                    'calib_mode':           int(current_mode)
                }
                try:
                    result_queue.put_nowait(result)
                except Exception:
                    pass

                minute_second_idx = 0
                minute_buffer_bands[:]            = 0.0
                minute_buffer_total_raw[:]        = 0.0
                minute_buffer_total_weighted_a[:] = 0.0
                minute_buffer_total_weighted_c[:] = 0.0

        packet_counter = 0
        second_count  += 1
        with stat_second.get_lock():
            stat_second.value = second_count

    logger.info("Analyzer stopped.")


# PROCESS 3: PUBLISHER

def publisher_process(
    result_queue, gps_array, gps_lock, gps_last_fix_time, plot_seconds, stop_event,
):
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [Publisher] %(message)s')
    logger = logging.getLogger()
    logger.info(f"Publisher started | plot at seconds: {plot_seconds}")

    try:
        client = mqtt.Client()
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        logger.info("Connected to MQTT")
    except Exception as e:
        logger.error(f"MQTT connect failed: {e}")
        client = None

    plotted_seconds  = set()
    plot_seconds_set = set(plot_seconds)

    nominal_labels = [
        "20", "25", "31.5", "40", "50", "63", "80", "100", "125", "160",
        "200", "250", "315", "400", "500", "630", "800", "1k", "1.25k",
        "1.6k", "2k", "2.5k", "3.15k", "4k", "5k", "6.3k", "8k",
        "10k", "12.5k", "16k", "20k",
    ]
    freq_keys = [
        "freq_00020", "freq_00025", "freq_00031_5", "freq_00040",
        "freq_00050", "freq_00063", "freq_00080", "freq_00100",
        "freq_00125", "freq_00160", "freq_00200", "freq_00250",
        "freq_00315", "freq_00400", "freq_00500", "freq_00630",
        "freq_00800", "freq_01000", "freq_01250", "freq_01600",
        "freq_02000", "freq_02500", "freq_03150", "freq_04000",
        "freq_05000", "freq_06300", "freq_08000", "freq_10000",
        "freq_12500", "freq_16000", "freq_20000",
    ]

    while not stop_event.is_set():
        try:
            result = result_queue.get(timeout=1.0)
        except Exception:
            continue

        lat, lon, gps_valid = gps_get(gps_array, gps_lock)
        if not gps_valid and int(time.time()) % 30 == 0:
            logger.warning("No valid GPS fix yet — lat/lon=0.0")

        ts    = result['timestamp']
        leq_z = result['leq_total_raw']
        leq_a = result['leq_total_weighted_a']
        leq_c = result['leq_total_weighted_c']
        label = result['label']
        mode  = result['display_mode']
        batt  = result.get('batt', 0.0)
        cal   = result['gain']
        calib_mode = result ['calib_mode']
        unit  = "minute" if mode == 'minute' else "second"
        fix   = f"lat={lat:.6f} lon={lon:.6f}" if gps_valid else "no GPS fix"

        logger.info(
            f"[{ts}] {label}: "
            f"LZeq={leq_z:.1f}dBZ  LAeq={leq_a:.1f}dBA  LCeq={leq_c:.1f}dBC  "
            f"({unit})  batt={batt:.1f}%  gain={cal:.3f}  {fix} {mode}"
        )

        sec = result['second']
        if sec in plot_seconds_set and sec not in plotted_seconds:
            _plot_result(result, nominal_labels, logger)
            plotted_seconds.add(sec)

        payload = {
            'no_dev':               'dev_001',
            'tanda':            result['second'],
            'leq_total_weighted_a': leq_a,
            'leq_total_weighted_c': leq_c,
            'calib_mode':           calib_mode,
            'leq_total':            leq_z,
            'lat':                  lat,
            'lon':                  lon,
            'gps_valid':            gps_valid,
            'batt':                 batt
        }
        for i, key in enumerate(freq_keys):
            payload[key] = result['leq_bands_raw'][i]

        if client:
            try:
                client.publish(MQTT_TOPIC, json.dumps(payload), qos=1)
            except Exception as e:
                logger.error(f"MQTT publish error: {e}")

    if client:
        client.loop_stop()
        client.disconnect()
    logger.info("Publisher stopped.")


# PLOT

def _plot_result(result, nominal_labels, logger):
    try:
        leq_bands_raw        = result['leq_bands_raw']
        leq_total_raw        = result['leq_total_raw']
        leq_total_weighted_a = result['leq_total_weighted_a']
        leq_total_weighted_c = result['leq_total_weighted_c']
        freq_bands           = result['freq_bands']
        label                = result['label']
        mode                 = result['display_mode']

        freq_labels = nominal_labels[:len(freq_bands)]
        fig, ax     = plt.subplots(figsize=(14, 7))
        x_pos       = np.arange(len(freq_bands))
        bars        = ax.bar(x_pos, leq_bands_raw, color='steelblue',
                             alpha=0.7, edgecolor='black', linewidth=0.5)

        for bar, level in zip(bars, leq_bands_raw):
            bar.set_color('red' if level > 80 else ('orange' if level > 65 else 'green'))
            ax.text(bar.get_x() + bar.get_width() / 2., bar.get_height() + 1.5,
                    f'{level:.1f}', ha='center', va='bottom',
                    fontsize=8, fontweight='bold')

        ax.set_xticks(x_pos)
        ax.set_xticklabels(freq_labels, rotation=45, ha='right', fontsize=10)
        ax.set_xlabel('Frequency (Hz)', fontsize=12, fontweight='bold')
        ax.set_ylabel('LZeq (dBZ)', fontsize=12, fontweight='bold')
        duration_str = "30-second" if mode == 'minute' else "1-second"
        ax.set_title(
            f'1/3 Octave Band LZeq [{duration_str}] [{label}]\n'
            f'LZeq={leq_total_raw:.1f}dBZ  '
            f'LAeq={leq_total_weighted_a:.1f}dBA  '
            f'LCeq={leq_total_weighted_c:.1f}dBC',
            fontsize=14, fontweight='bold'
        )
        ax.set_ylim(20, 125)
        ax.grid(True, alpha=0.3, axis='y', linestyle='--')
        plt.tight_layout()

        filename = f"leq_{mode}_s{result['second']}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        logger.info(f"Plot saved: {filename}")
        plt.close(fig)
        gc.collect()
    except Exception as e:
        logger.error(f"Plot error: {e}")


# MAIN

def main():
    mp.set_start_method('spawn')

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)

    logger.info("=" * 70)
    logger.info("Integrated Noise Monitoring System")
    logger.info(f"Audio  : {UART_PORT} (ESP32-S3 USB CDC)")
    logger.info(f"GPS    : {GPS_PORT} (NEO-7M GPIO UART)"
                if GPS_ENABLED else "GPS    : DISABLED")
    logger.info(f"Mode   : {DISPLAY_MODE} | Weighting: A + C + Z")
    logger.info(f"MIC_REF_AMPL = {MIC_REF_AMPL:.1f}  "
                f"(sensitivity={MIC_SENSITIVITY}dBFS, bits={MIC_BITS})")
    logger.info("=" * 70)

    gps_array, gps_lock, gps_last_fix_time = make_gps_state()
    raw_queue    = mp.Queue(maxsize=RAW_QUEUE_SIZE)
    result_queue = mp.Queue(maxsize=RESULT_QUEUE_SIZE)
    stop_event   = mp.Event()

    stat_pkt_received  = mp.Value('i', 0)
    stat_pkt_corrupted = mp.Value('i', 0)
    stat_pkt_dropped   = mp.Value('i', 0)
    stat_second        = mp.Value('i', 0)

    p_gps = mp.Process(
        target=gps_reader_process,
        args=(gps_array, gps_lock, gps_last_fix_time, stop_event,
              GPS_UPDATE_INTERVAL_SEC, GPS_READ_TIMEOUT_SEC),
        name="GpsReader", daemon=True,
    )
    p_reader = mp.Process(
        target=serial_reader_process,
        args=(raw_queue, stat_pkt_received, stat_pkt_corrupted,
              stat_pkt_dropped, stop_event),
        name="SerialReader", daemon=True,
    )
    p_analyzer = mp.Process(
        target=octave_leq_analyzer_process,
        args=(raw_queue, result_queue, WEIGHTING_A, WEIGHTING_C,
              DISPLAY_MODE, stat_second, stop_event),
        name="OctaveAnalyzer", daemon=True,
    )
    p_publisher = mp.Process(
        target=publisher_process,
        args=(result_queue, gps_array, gps_lock, gps_last_fix_time,
              PLOT_SECONDS, stop_event),
        name="Publisher", daemon=True,
    )

    p_gps.start();      time.sleep(0.5)
    p_reader.start();   time.sleep(0.3)
    p_analyzer.start(); time.sleep(0.2)
    p_publisher.start()

    logger.info("All processes started. Ctrl+C to stop.")
    logger.info("=" * 70)

    try:
        while True:
            time.sleep(1)
            if not p_analyzer.is_alive():
                logger.error("OctaveAnalyzer died unexpectedly!")
                break
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        logger.info("Stopping...")
    finally:
        stop_event.set()
        for p in [p_gps, p_reader, p_analyzer, p_publisher]:
            p.join(timeout=5)
            if p.is_alive():
                logger.warning(f"{p.name} did not stop — terminating.")
                p.terminate()

        logger.info("=== Final Statistics ===")
        logger.info(f"Packets received  : {stat_pkt_received.value}")
        logger.info(f"Packets corrupted : {stat_pkt_corrupted.value}")
        logger.info(f"Packets dropped   : {stat_pkt_dropped.value}")
        total = stat_pkt_received.value + stat_pkt_corrupted.value + stat_pkt_dropped.value
        if total > 0:
            logger.info(f"Success rate      : {stat_pkt_received.value/total*100:.2f}%")
        logger.info(f"Total seconds     : {stat_second.value}")
        lat, lon, valid = gps_get(gps_array, gps_lock)
        logger.info(f"Last GPS fix      : lat={lat:.6f} lon={lon:.6f} valid={valid}")
        logger.info("=" * 70)


if __name__ == "__main__":
    main()
