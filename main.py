#!/usr/bin/env python3
"""
Optimized Noise Monitoring System - Multiprocessing Version
Flow: SerialReaderProcess → Queue(A) → OctaveLeqAnalyzerProcess → Queue(B) → PublisherProcess

DISPLAY_MODE:
    'second' → hasil ditampilkan setiap detik (Leq 1 detik)
    'minute' → kalkulasi tetap per detik, hasil ditampilkan setiap 60 detik (Leq 1 menit)
               formula: 10 * log10((1/60) * Σ 10^(Leq_band_i / 10))  untuk i = 1..60

ARCHITECTURE (gain):
    ESP32 → raw samples + calib_multiplier header → Pi reader → raw_queue
    Analyzer applies calib_multiplier to raw_samples BEFORE filter bank.
    This keeps reader dumb/fast and gives analyzer full visibility of raw data.

PACKET LAYOUT (must match usb_packet_t in main_integrated.cpp):
    start_marker         : uint32   4 B
    timestamp_ms         : uint32   4 B
    latitude             : float    4 B
    longitude            : float    4 B
    calibration_multiplier: float   4 B
    battery_percentage   : float    4 B  
    samples[6000]        : float[] 24000 B
    end_marker           : uint32   4 B
    TOTAL                          24028 B
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
matplotlib.use('Agg')  # Wajib: nonaktifkan GUI backend di background process
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import json

# ===================== CONFIGURATION =====================
UART_PORT          = '/dev/ttyACM0'
UART_BAUD          = 115200
SAMPLE_RATE        = 48000
SAMPLES_SHORT      = 6000

FRAME_START_MARKER = 0xAA55AA55
FRAME_END_MARKER   = 0x55AA55AA

OCTAVE_FRACTION    = 3
OCTAVE_ORDER       = 6
FREQ_LIMITS        = [20, 20000]

PACKETS_PER_SECOND = 8
MIC_SENSITIVITY    = -26
MIC_REF_DB         = 94.0
MIC_OFFSET_DB      = 3.0103
MIC_BITS           = 24
MIC_REF_AMPL       = pow(10, MIC_SENSITIVITY / 20) * ((1 << (MIC_BITS - 1)) - 1)

WEIGHTING          = 'Z'   # 'A', 'C', or 'Z'

# ===================== DISPLAY MODE =====================
DISPLAY_MODE       = 'second'   # 'second' or 'minute'
SECONDS_PER_MINUTE = 60

if DISPLAY_MODE not in ('second', 'minute'):
    raise ValueError(f"Invalid DISPLAY_MODE '{DISPLAY_MODE}'. Must be 'second' or 'minute'.")

# ===================== FRAME SIZE =====================
# ESP32 packet layout (little-endian), must stay in sync with usb_packet_t:
#   start_marker            : uint32  →  4 bytes
#   timestamp_ms            : uint32  →  4 bytes
#   latitude                : float   →  4 bytes
#   longitude               : float   →  4 bytes
#   calibration_multiplier  : float   →  4 bytes
#   battery_percentage      : float   →  4 bytes  ← integrated firmware added this
#   samples[6000]           : float[] → 24000 bytes
#   end_marker              : uint32  →  4 bytes
#                                       ──────────
#   TOTAL                               24028 bytes
HEADER_FMT  = '<II4f'                          # start, ts, lat, lon, calib, batt
HEADER_SIZE = struct.calcsize(HEADER_FMT)      # = 24 bytes
FRAME_SIZE  = HEADER_SIZE + (SAMPLES_SHORT * 4) + 4   # 24 + 24000 + 4 = 24028 bytes

# ===================== QUEUE SIZES =====================
RAW_QUEUE_SIZE    = 50
RESULT_QUEUE_SIZE = 20

# ===================== PLOT CONFIGURATION =====================
# 'second' mode: plot at second numbers listed below
# 'minute' mode: use multiples of SECONDS_PER_MINUTE (e.g. [60, 120])
PLOT_SECONDS_RAW = [10, 20]

MQTT_HOST = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = "kebisingan/Leq"

def _resolve_plot_seconds(plot_seconds, display_mode, secs_per_min):
    """
    In minute mode, snap requested plot points to the nearest minute boundary
    so that the publisher's second-based trigger actually fires.
    E.g. [10, 20] → [60, 60] → deduplicated → [60]; [70, 130] → [60, 120].
    Emits a warning if snapping changes the values.
    """
    if display_mode == 'second':
        return list(plot_seconds)
    snapped = [max(secs_per_min, round(s / secs_per_min) * secs_per_min)
               for s in plot_seconds]
    if snapped != list(plot_seconds):
        logging.warning(
            f"PLOT_SECONDS {plot_seconds} snapped to minute boundaries → {snapped} "
            f"(results only arrive every {secs_per_min}s in 'minute' mode)"
        )
    # deduplicate while preserving order
    seen = set()
    return [x for x in snapped if not (x in seen or seen.add(x))]

PLOT_SECONDS = _resolve_plot_seconds(PLOT_SECONDS_RAW, DISPLAY_MODE, SECONDS_PER_MINUTE)

# ===================== WEIGHTING CORRECTIONS =====================
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
Z_WEIGHTING_CORRECTIONS = np.zeros(31)

WEIGHTING_DICT = {
    'A': A_WEIGHTING_CORRECTIONS,
    'C': C_WEIGHTING_CORRECTIONS,
    'Z': Z_WEIGHTING_CORRECTIONS,
}


# ===================== RING BUFFER =====================
class RingBuffer:
    """
    Circular byte buffer for the serial reader.
    Avoids repeated allocation/copying of large serial chunks.
    """
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
            self.head = 0
            self.tail = 0
            self.size = self.capacity
            return
        if self.tail + data_len <= self.capacity:
            self.buffer[self.tail:self.tail + data_len] = data
        else:
            first_part = self.capacity - self.tail
            self.buffer[self.tail:]         = data[:first_part]
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


# ===================== PROCESS 1: SERIAL READER =====================
def serial_reader_process(
    raw_queue:          mp.Queue,
    stat_pkt_received:  mp.Value,
    stat_pkt_corrupted: mp.Value,
    stat_pkt_dropped:   mp.Value, 
    stop_event:         mp.Event,
):
    """
    Reads UART bytes, parses framed packets, and forwards raw data to the
    analyzer queue.

    Design decisions:
    - Reader is intentionally kept DUMB: it only validates framing (start/end
      markers) and extracts header fields + raw sample bytes.
    - Gain (calib_multiplier) is passed through as a plain float field.
      The analyzer process applies it BEFORE the filter bank, so the analyzer
      always has access to both raw and calibrated data if needed.
    - put_nowait is intentional: if the analyzer falls behind we drop the
      oldest-arriving packet rather than blocking the UART read loop.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Reader] %(message)s')
    logger = logging.getLogger()

    buffer             = RingBuffer(capacity=FRAME_SIZE * 10)
    start_marker_bytes = struct.pack('<I', FRAME_START_MARKER)

    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.05)
        logger.info(f"Serial opened: {UART_PORT} @ {UART_BAUD} baud | frame={FRAME_SIZE}B")

        while not stop_event.is_set():
            if ser.in_waiting > 0:
                chunk = ser.read(max(ser.in_waiting, 8192))
                buffer.extend(chunk)

            while len(buffer) >= FRAME_SIZE:
                # ── Step 1: ensure buffer head is a start marker ──────────
                first_bytes = buffer.get_bytes(4)
                if first_bytes != start_marker_bytes:
                    idx = buffer.find(start_marker_bytes)
                    if idx == -1:
                        # no marker in buffer at all — discard all but last 3 bytes
                        buffer.consume(max(0, len(buffer) - 3))
                        break
                    buffer.consume(idx)
                    continue

                # ── Step 2: peek at full frame ────────────────────────────
                frame_bytes = buffer.get_bytes(FRAME_SIZE)
                if frame_bytes is None:
                    break   # not enough data yet

                # ── Step 3: validate end marker ───────────────────────────
                end_marker = struct.unpack('<I', frame_bytes[-4:])[0]
                if end_marker != FRAME_END_MARKER:
                    with stat_pkt_corrupted.get_lock():
                        stat_pkt_corrupted.value += 1
                    buffer.consume(FRAME_SIZE)
                    continue

                # ── Step 4: parse header (20 bytes) ──────────────────────
                try:
                    header  = struct.unpack(HEADER_FMT, frame_bytes[:HEADER_SIZE])
                    # header: (start_marker, timestamp_ms, lat, lon, calib, battery_pct)
                    lat   = header[2]
                    lon   = header[3]
                    calib = header[4]
                    batt  = header[5]   # battery_percentage from INA219

                    # ── Step 5: extract raw samples (no gain applied here) ─
                    samples_end  = HEADER_SIZE + SAMPLES_SHORT * 4
                    raw_samples  = np.frombuffer(
                        frame_bytes[HEADER_SIZE:samples_end], dtype=np.float32
                    ).copy()   # .copy() required: frame_bytes is a temporary bytes object

                    packet = {
                        'raw_samples': raw_samples,   # untouched sensor data
                        'calib':       float(calib),  # gain for analyzer to apply
                        'lat':         float(lat),
                        'lon':         float(lon),
                        'batt':        float(batt),   # passed through to result
                    }

                    try:
                        raw_queue.put_nowait(packet)
                        with stat_pkt_received.get_lock():
                            stat_pkt_received.value += 1
                    except Exception:
                        # Queue full → analyzer is behind → drop this packet
                        with stat_pkt_dropped.get_lock():
                            stat_pkt_dropped.value += 1

                except Exception:
                    with stat_pkt_corrupted.get_lock():
                        stat_pkt_corrupted.value += 1

                buffer.consume(FRAME_SIZE)

    except Exception as e:
        logger.error(f"Serial Reader fatal error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        logger.info("Serial Reader stopped.")


# ===================== FILTER BANK =====================
class OctaveFilterBank:
    """
    1/3-octave bandpass filter bank.

    Optimization notes vs original:
    - signal.resample (FFT-based, slow) replaced with signal.resample_poly
      (polyphase, integer ratios, significantly faster on Pi 4).
    - Filter coefficients are computed once at init and reused for every frame.
    """
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
                btype='bandpass',
                analog=False,
                output='sos',
            )
            self.sos_filters.append(sos)

        self._resample_factors = [int(round(f)) for f in self.factor]

    def filter_signal_to_leq_bands(self, x):
        leq_bands = np.zeros(len(self.freq))
        for idx, (sos, down) in enumerate(zip(self.sos_filters, self._resample_factors)):
            if down > 1:
                sd = signal.resample_poly(x, up=1, down=down)
            else:
                sd = x
            y   = signal.sosfilt(sos, sd)
            rms = np.sqrt(np.mean(y ** 2))
            leq_bands[idx] = (
                MIC_OFFSET_DB + MIC_REF_DB + 20 * np.log10(rms / MIC_REF_AMPL)
                if rms > 1e-10 else 0.0
            )
        return leq_bands


# ===================== LEQ AVERAGING =====================
def _leq_average(leq_array_db: np.ndarray, n: int) -> np.ndarray:
    """
    Logarithmic Leq average of n measurements.
    Formula: 10 * log10( (1/n) * Σ 10^(Leq_i / 10) )

    Args:
        leq_array_db : 2-D (n, n_bands) or 1-D (n,) array of dB values
        n            : number of measurements (divisor)
    Returns:
        1-D (n_bands,) array or scalar of averaged dB values
    """
    linear      = 10 ** (leq_array_db / 10)
    linear_mean = np.sum(linear, axis=0) / n
    linear_mean = np.where(linear_mean > 1e-10, linear_mean, 1e-10)
    return 10 * np.log10(linear_mean)


# ===================== PROCESS 2: OCTAVE ANALYZER =====================
def octave_leq_analyzer_process(
    raw_queue:    mp.Queue,
    result_queue: mp.Queue,
    weighting:    str,
    display_mode: str,
    stat_second:  mp.Value,
    stop_event:   mp.Event,
):
    """
    The heavy process: applies calibration gain, runs the 31-band filter bank,
    computes Leq per second (and per minute in 'minute' mode), and forwards
    results to the publisher.

    Key design:
    - Gain is applied HERE (not in the reader) so the analyzer has both the
      raw signal and can choose whether/how to scale before filtering.
    - In minute mode, per-second weighted totals are stored directly in a
      buffer, avoiding the expensive list-comprehension recalculation at
      minute boundaries.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Analyzer] %(message)s')
    logger = logging.getLogger()
    logger.info(f"Analyzer started | weighting={weighting} | mode={display_mode}")

    filter_bank          = OctaveFilterBank(SAMPLE_RATE, OCTAVE_FRACTION, OCTAVE_ORDER, FREQ_LIMITS)
    weighting_corrections = WEIGHTING_DICT[weighting]
    n_bands              = len(filter_bank.freq)

    # ── Per-second accumulation (always active) ───────────────────────────
    packet_counter = 0
    all_samples    = np.zeros(SAMPLES_SHORT * PACKETS_PER_SECOND, dtype=np.float32)
    current_lat    = 0.0
    current_lon    = 0.0
    second_count   = 1 

    # ── Per-minute accumulation (minute mode only) ────────────────────────
    # at minute boundaries from raw band buffers.
    minute_buffer_bands          = np.zeros((SECONDS_PER_MINUTE, n_bands), dtype=np.float64)
    minute_buffer_total_raw      = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_buffer_total_weighted = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_second_idx            = 0
    last_lat                     = 0.0
    last_lon                     = 0.0
    last_batt                    = 0.0
    current_batt                 = 0.0

    while not stop_event.is_set():
        try:
            packet = raw_queue.get(timeout=1.0)
        except Exception:
            continue

        # ── ngekalibrasi  ───────
        raw_samples = packet['raw_samples']
        calib       = packet['calib']
        samples     = raw_samples * calib   # ini kalibrasinya

        current_lat  = packet['lat']
        current_lon  = packet['lon']
        current_batt = packet['batt']

        # Accumulate 8 packets → 1 second of audio
        start_idx = packet_counter * SAMPLES_SHORT
        all_samples[start_idx:start_idx + SAMPLES_SHORT] = samples
        packet_counter += 1

        if packet_counter < PACKETS_PER_SECOND:
            continue

        # ── 1-second Leq calculation ──────────────────────────────────────
        leq_bands_raw      = filter_bank.filter_signal_to_leq_bands(all_samples)
        leq_bands_weighted = leq_bands_raw + weighting_corrections

        linear_sum_w       = np.sum(10 ** (leq_bands_weighted / 10))
        leq_total_weighted = 10 * np.log10(linear_sum_w) if linear_sum_w > 1e-10 else 0.0

        linear_sum_r  = np.sum(10 ** (leq_bands_raw / 10))
        leq_total_raw = 10 * np.log10(linear_sum_r) if linear_sum_r > 1e-10 else 0.0

        timestamp = time.strftime("%H:%M:%S")
        last_lat  = current_lat
        last_lon  = current_lon
        last_batt = current_batt

        # ── Dispatch to publisher ─────────────────────────────────────────
        if display_mode == 'second':
            result = {
                'leq_total_weighted':  float(leq_total_weighted),
                'leq_total_raw':       float(leq_total_raw),
                'leq_bands_weighted':  leq_bands_weighted.tolist(),
                'leq_bands_raw':       leq_bands_raw.tolist(),
                'freq_bands':          filter_bank.freq,
                'weighting':           weighting,
                'display_mode':        'second',
                'timestamp':           timestamp,
                'lat':                 float(current_lat),
                'lon':                 float(current_lon),
                'batt':                float(current_batt),
                'second':              second_count,
                'label':               f"Second #{second_count}",
                'gain':                calib,
            }
            try:
                result_queue.put_nowait(result)
            except Exception:
                pass   # publisher busy → skip, no block

        elif display_mode == 'minute':
            # Store per-second values.
            minute_buffer_bands[minute_second_idx]          = leq_bands_raw
            minute_buffer_total_raw[minute_second_idx]      = leq_total_raw
            minute_buffer_total_weighted[minute_second_idx] = leq_total_weighted
            minute_second_idx += 1

            if minute_second_idx >= SECONDS_PER_MINUTE:
                minute_count = second_count // SECONDS_PER_MINUTE

                # Leq 1-minute per band (raw), then weighted
                leq_1min_bands_raw      = _leq_average(minute_buffer_bands, SECONDS_PER_MINUTE)
                leq_1min_bands_weighted = leq_1min_bands_raw + weighting_corrections

                # Leq 1-minute total (raw and weighted)
            
                leq_1min_total_raw      = float(_leq_average(minute_buffer_total_raw,      SECONDS_PER_MINUTE))
                leq_1min_total_weighted = float(_leq_average(minute_buffer_total_weighted, SECONDS_PER_MINUTE))

                result = {
                    'leq_total_weighted':  leq_1min_total_weighted,
                    'leq_total_raw':       leq_1min_total_raw,
                    'leq_bands_weighted':  leq_1min_bands_weighted.tolist(),
                    'leq_bands_raw':       leq_1min_bands_raw.tolist(),
                    'freq_bands':          filter_bank.freq,
                    'weighting':           weighting,
                    'display_mode':        'minute',
                    'timestamp':           timestamp,
                    'lat':                 float(last_lat),
                    'lon':                 float(last_lon),
                    'batt':                float(last_batt),
                    'second':              second_count,
                    'minute':              minute_count,
                    'label':               f"Minute #{minute_count} (second #{second_count})",
                    'gain':                calib,
                }
                try:
                    result_queue.put_nowait(result)
                except Exception:
                    pass

                # Reset minute buffers
                minute_second_idx = 0
                minute_buffer_bands[:]          = 0.0
                minute_buffer_total_raw[:]      = 0.0
                minute_buffer_total_weighted[:] = 0.0

        # Reset second accumulation
        packet_counter = 0
        second_count  += 1

        with stat_second.get_lock():
            stat_second.value = second_count

    logger.info("Analyzer stopped.")


# ===================== PROCESS 3: PUBLISHER =====================
def publisher_process(
    result_queue: mp.Queue,
    plot_seconds: list,
    stop_event:   mp.Event,
):
    """
    Handles all output: terminal print, PNG plots, and (optionally) MQTT.
    Running in its own process ensures slow operations (matplotlib savefig)
    never stall the analyzer.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Publisher] %(message)s')
    logger = logging.getLogger()
    logger.info(f"Publisher started | will plot at seconds: {plot_seconds}")

    try:
        client = mqtt.Client()
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        logger.info(f"Connected to MQTT")
    except Exception as e:
        logger.error(f"Failed connecting to MQTT: {e}")
        client = None


    plotted_seconds  = set()
    plot_seconds_set = set(plot_seconds)

    nominal_labels = [
        "20", "25", "31.5", "40", "50", "63", "80", "100", "125", "160",
        "200", "250", "315", "400", "500", "630", "800", "1k", "1.25k",
        "1.6k", "2k", "2.5k", "3.15k", "4k", "5k", "6.3k", "8k",
        "10k", "12.5k", "16k", "20k",
    ]

    while not stop_event.is_set():
        try:
            result = result_queue.get(timeout=1.0)
        except Exception:
            continue

        w    = result['weighting']
        ts   = result['timestamp']
        leq_w = result['leq_total_weighted']
        label = result['label']
        mode  = result['display_mode']
        unit  = "minute" if mode == 'minute' else "second"
        batt  = result.get('batt', 0.0)
        calibration = result['gain']

        #print(f"[{ts}] {label}: L{w}eq = {leq_w:.1f} dB{w}  ({unit})  batt={batt:.1f}%")
        logger.info(f"[{ts}] {label}: L{w}eq = {leq_w:.1f} dB{w}  ({unit})  batt={batt:.1f}% gain={calibration}")

        sec = result['second']
        if sec in plot_seconds_set and sec not in plotted_seconds:
            _plot_result(result, nominal_labels, logger)
            plotted_seconds.add(sec)

        # MQTT (uncomment when ready):
        # _publish_mqtt(result)

        list_frekuensi = ["freq_00020", "freq_00025", "freq_00031_5", "freq_00040",
                "freq_00050", "freq_00063", "freq_00080", "freq_00100", "freq_00125", "freq_00160", "freq_00200",
                "freq_00250", "freq_00315", "freq_00400", "freq_00500", "freq_00630", "freq_00800", "freq_01000",
                "freq_01250", "freq_01600", "freq_02000", "freq_02500", "freq_03150", "freq_04000", "freq_05000",
                "freq_06300", "freq_08000", "freq_10000", "freq_12500", "freq_16000", "freq_20000"]
        
        data_kebisingan = {}
        data_kebisingan['no_dev'] = "dev_001"
        data_kebisingan['leq_total_weighted'] = result['leq_total_weighted']
        data_kebisingan['leq_total'] = result['leq_total_raw'] # ini perlu dikirim kah?
        data_kebisingan['lat'] = result['lat']
        data_kebisingan['lon'] = result['lon']
        data_kebisingan['batt'] = batt
        for i in range(len(list_frekuensi)): 
            data_kebisingan[list_frekuensi[i]] = result['leq_bands_raw'][i]
        
        if client:
            try:
                payload = json.dumps(data_kebisingan)
                client.publish(MQTT_TOPIC, payload, 1)
            except Exception as e:
                logger.error(f"Failed to transmit data: {e}")
        
    if client:
        client.loop_stop()
        client.disconnect()
        logger.info("MQTT Disconnected")


    logger.info("Publisher stopped.")


def _plot_result(result, nominal_labels, logger):
    """Saves a 1/3-octave bar chart PNG for a given result dict."""
    try:
        leq_bands_weighted = result['leq_bands_weighted'] # tinggal ganti leq_bands_raw kalau mau tampilin yang raw
        leq_total_weighted = result['leq_total_weighted']
        freq_bands         = result['freq_bands']
        w                  = result['weighting']
        label              = result['label']
        mode               = result['display_mode']

        freq_labels  = nominal_labels[:len(freq_bands)]
        fig, ax      = plt.subplots(figsize=(14, 7))
        x_pos        = np.arange(len(freq_bands))
        bars         = ax.bar(x_pos, leq_bands_weighted,
                              color='steelblue', alpha=0.7,
                              edgecolor='black', linewidth=0.5)

        for bar, level in zip(bars, leq_bands_weighted):
            bar.set_color('red' if level > 80 else ('orange' if level > 65 else 'green'))
            ax.text(
                bar.get_x() + bar.get_width() / 2., bar.get_height() + 1.5,
                f'{level:.1f}', ha='center', va='bottom',
                fontsize=8, fontweight='bold',
            )

        ax.set_xticks(x_pos)
        ax.set_xticklabels(freq_labels, rotation=45, ha='right', fontsize=10)
        ax.set_xlabel('Frequency (Hz)', fontsize=12, fontweight='bold')
        ax.set_ylabel(f'L{w}eq (dB{w})', fontsize=12, fontweight='bold')

        duration_str = "1-minute" if mode == 'minute' else "1-second"
        ax.set_title(
            f'1/3 Octave Band L{w}eq [{duration_str}] [{label}]\n'
            f'L{w}eq_total = {leq_total_weighted:.1f} dB{w}',
            fontsize=14, fontweight='bold',
        )
        ax.set_ylim(20, 125)
        ax.grid(True, alpha=0.3, axis='y', linestyle='--')
        plt.tight_layout()

        sec      = result['second']
        filename = f'leq_{w}_{mode}_s{sec}.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        logger.info(f"Plot saved: {filename}")
        plt.close(fig)
        gc.collect()

    except Exception as e:
        logger.error(f"Plot error: {e}")


# ===================== MAIN =====================
def main():
    mp.set_start_method('fork')

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
    )
    logger = logging.getLogger(__name__)

    logger.info("=" * 70)
    logger.info("Noise Monitoring - Multiprocessing + Display Mode")
    logger.info(f"Weighting    : {WEIGHTING}-weighting")
    logger.info(f"Display Mode : {DISPLAY_MODE}")
    logger.info(f"CPU cores    : {mp.cpu_count()}")
    logger.info(f"Frame size   : {FRAME_SIZE} bytes")
    if DISPLAY_MODE == 'minute':
        logger.info(f"Accumulation : {SECONDS_PER_MINUTE} seconds per output")
        logger.info(f"Plot at      : seconds {PLOT_SECONDS} "
                    f"→ minute(s) {[s // SECONDS_PER_MINUTE for s in PLOT_SECONDS]}")
    else:
        logger.info(f"Plot at      : seconds {PLOT_SECONDS}")
    logger.info("=" * 70)

    # ── Shared state ──────────────────────────────────────────────────────
    raw_queue    = mp.Queue(maxsize=RAW_QUEUE_SIZE)
    result_queue = mp.Queue(maxsize=RESULT_QUEUE_SIZE)
    stop_event   = mp.Event()

    stat_pkt_received  = mp.Value('i', 0)
    stat_pkt_corrupted = mp.Value('i', 0)
    stat_pkt_dropped   = mp.Value('i', 0)   
    stat_second        = mp.Value('i', 0)

    # ── Processes ─────────────────────────────────────────────────────────
    p_reader = mp.Process(
        target=serial_reader_process,
        args=(raw_queue, stat_pkt_received, stat_pkt_corrupted,
              stat_pkt_dropped, stop_event),
        name="SerialReader",
        daemon=True,
    )
    p_analyzer = mp.Process(
        target=octave_leq_analyzer_process,
        args=(raw_queue, result_queue, WEIGHTING, DISPLAY_MODE,
              stat_second, stop_event),
        name="OctaveAnalyzer",
        daemon=True,
    )
    p_publisher = mp.Process(
        target=publisher_process,
        args=(result_queue, PLOT_SECONDS, stop_event),
        name="Publisher",
        daemon=True,
    )

    p_reader.start()
    time.sleep(0.3)
    p_analyzer.start()
    time.sleep(0.2)
    p_publisher.start()

    logger.info("All processes started. Press Ctrl+C to stop.")
    logger.info("=" * 70)

    try:
        while True:
            time.sleep(1)
            if not p_reader.is_alive():
                logger.error("SerialReader died unexpectedly!")
                break
            if not p_analyzer.is_alive():
                logger.error("OctaveAnalyzer died unexpectedly!")
                break
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        logger.info("Stopping...")
    finally:
        stop_event.set()

        for p in [p_reader, p_analyzer, p_publisher]:
            p.join(timeout=5)
            if p.is_alive():
                logger.warning(f"{p.name} did not stop cleanly, terminating...")
                p.terminate()

        print("\n" + "=" * 70)
        logger.info("=== Final Statistics ===")
        logger.info(f"Packets received   : {stat_pkt_received.value}")
        logger.info(f"Packets corrupted  : {stat_pkt_corrupted.value}")
        logger.info(f"Packets dropped    : {stat_pkt_dropped.value}") 
        total = stat_pkt_received.value + stat_pkt_corrupted.value + stat_pkt_dropped.value
        if total > 0:
            logger.info(f"Success rate       : {stat_pkt_received.value / total * 100:.2f}%")
        logger.info(f"Total seconds      : {stat_second.value}")
        if DISPLAY_MODE == 'minute':
            logger.info(f"Total minutes      : {stat_second.value // SECONDS_PER_MINUTE}")
        logger.info("=" * 70)


if __name__ == "__main__":
    main()
