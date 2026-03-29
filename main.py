#!/usr/bin/env python3
"""
Optimized Noise Monitoring System - Multiprocessing Version
Flow: SerialReaderProcess → Queue(A) → OctaveLeqAnalyzerProcess → Queue(B) → PublisherProcess

DISPLAY_MODE:
    'second' → hasil ditampilkan setiap detik (Leq 1 detik)
    'minute' → kalkulasi tetap per detik, hasil ditampilkan setiap 60 detik (Leq 1 menit)
               formula: 10 * log10((1/60) * Σ 10^(Leq_band_i / 10))  untuk i = 1..60
"""

import multiprocessing as mp
import serial
import struct
import numpy as np
import time
import logging
import queue
from scipy import signal
import matplotlib
matplotlib.use('Agg') # WAJIB: supaya matplotlib engga buka GUI dari background process. jadi ga ngelag
import matplotlib.pyplot as plt
import gc

# ===================== CONFIGURATION =====================
UART_PORT = '/dev/ttyACM0'
UART_BAUD = 115200
SAMPLE_RATE = 48000
SAMPLES_SHORT = 6000

FRAME_START_MARKER = 0xAA55AA55
FRAME_END_MARKER = 0x55AA55AA

OCTAVE_FRACTION = 3
OCTAVE_ORDER = 6
FREQ_LIMITS = [20, 20000]

PACKETS_PER_SECOND = 8
MIC_SENSITIVITY = -26
MIC_REF_DB = 94.0
MIC_OFFSET_DB = 3.0103
MIC_BITS = 24
MIC_REF_AMPL = pow(10, MIC_SENSITIVITY / 20) * ((1 << (MIC_BITS - 1)) - 1)

WEIGHTING = 'Z'  # Opsi: 'A', 'C', 'Z'

# ===================== DISPLAY MODE =====================
DISPLAY_MODE = 'minute'  # Opsi: 'second' atau 'minute'
SECONDS_PER_MINUTE = 60  # Konstanta akumulasi mode 'minute'

# Validasi opsi displaynya
if DISPLAY_MODE not in ('second', 'minute'):
    raise ValueError(f"Invalid DISPLAY_MODE '{DISPLAY_MODE}'. Must be 'second' or 'minute'.")

# ===================== WEIGHTING CORRECTIONS =====================
A_WEIGHTING_CORRECTIONS = np.array([
    -50.5, -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9, -8.6, -6.6, -4.8, -3.2, -1.9, -0.8, 0.0, 0.6, 1.0,
    1.2, 1.3, 1.2, 1.0, 0.5, -0.1, -1.1, -2.5, -4.3, -6.6, -9.3
])
C_WEIGHTING_CORRECTIONS = np.array([
    -6.2, -4.4, -3.0, -2.0, -1.3, -0.8, -0.5, -0.3, -0.2, -0.1,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.1, -0.2, -0.3, -0.5, -0.8, -1.3, -2.0, -3.0
])
Z_WEIGHTING_CORRECTIONS = np.zeros(31)
WEIGHTING_DICT = {
    'A': A_WEIGHTING_CORRECTIONS,
    'C': C_WEIGHTING_CORRECTIONS,
    'Z': Z_WEIGHTING_CORRECTIONS
}

FRAME_SIZE = 4 + 4 + 4 + 4 + (SAMPLES_SHORT * 4) + 4

# ===================== PLOT CONFIGURATION =====================
# Mode 'second' : detik ke-1, 5, 10  → plot di second #1, #5, #10
# Mode 'minute' : detik ke-60, 120   → plot di minute #1, #2
PLOT_SECONDS = [60, 120]

RAW_QUEUE_SIZE = 50
RESULT_QUEUE_SIZE = 20


# ===================== RING BUFFER =====================
class RingBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = bytearray(capacity)
        self.head = 0
        self.tail = 0
        self.size = 0

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
            self.tail = (self.tail + data_len) % self.capacity
        else:
            first_part = self.capacity - self.tail
            self.buffer[self.tail:] = data[:first_part]
            self.buffer[:data_len - first_part] = data[first_part:]
            self.tail = data_len - first_part
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
            data = self.buffer[self.head:] + self.buffer[:self.tail]
        idx = data.find(pattern)
        return idx if idx != -1 else -1

    def __len__(self):
        return self.size


# ===================== PROCESS 1: SERIAL READER =====================
def serial_reader_process(
    raw_queue: mp.Queue,
    stat_pkt_received: mp.Value,
    stat_pkt_corrupted: mp.Value,
    stop_event: mp.Event
):
    """
    Proses untuk baca UART dan parsing frame.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Reader] %(message)s')
    logger = logging.getLogger()

    buffer = RingBuffer(capacity=FRAME_SIZE * 10)
    struct_fmt = f'<II2f{SAMPLES_SHORT}fI'
    start_marker_bytes = struct.pack('<I', FRAME_START_MARKER)

    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.05)
        logger.info(f"Serial Reader Process started: {UART_PORT}")

        while not stop_event.is_set():
            if ser.in_waiting > 0:
                chunk = ser.read(max(ser.in_waiting, 8192))
                buffer.extend(chunk)

            while len(buffer) >= FRAME_SIZE:
                first_bytes = buffer.get_bytes(4)
                if first_bytes != start_marker_bytes:
                    idx = buffer.find(start_marker_bytes)
                    if idx == -1:
                        buffer.consume(max(0, len(buffer) - 3))
                        break
                    else:
                        buffer.consume(idx)
                        continue

                frame_bytes = buffer.get_bytes(FRAME_SIZE)
                if frame_bytes is None:
                    break

                end_marker = struct.unpack('<I', frame_bytes[-4:])[0]
                if end_marker == FRAME_END_MARKER:
                    try:
                        data = struct.unpack(struct_fmt, frame_bytes)
                        samples_start = 16
                        samples_end = samples_start + (SAMPLES_SHORT * 4)
                        samples = np.frombuffer(
                            frame_bytes[samples_start:samples_end], dtype=np.float32
                        ).copy()

                        packet = {'samples': samples, 'lat': data[2], 'lon': data[3]}
                        
                        # put_nowait supaya kalau paket penuh ga blocking process -> data sabi ilang untuk memastikan real time processing
                        try:
                            raw_queue.put_nowait(packet)
                            with stat_pkt_received.get_lock():
                                stat_pkt_received.value += 1
                        except:
                            pass     # Queue penuh —> analyzer terlambat, skip paket ini
                    except Exception:
                        with stat_pkt_corrupted.get_lock():
                            stat_pkt_corrupted.value += 1
                else:
                    with stat_pkt_corrupted.get_lock():
                        stat_pkt_corrupted.value += 1

                buffer.consume(FRAME_SIZE)

    except Exception as e:
        logger.error(f"Serial Reader Process Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        logger.info("Serial Reader Process stopped.")


# ===================== FILTER BANK (diinisialisasi di dalam proses analyzer) =====================
class OctaveFilterBank:
    def __init__(self, fs, fraction, order, limits):
        from octave_filter import getansifrequencies, _downsamplingfactor

        freq, freq_d, freq_u = getansifrequencies(fraction, limits)
        nyquist = fs / 2
        valid_idx = [i for i, f in enumerate(freq_u) if f < nyquist]

        self.freq = [freq[i] for i in valid_idx]
        freq_d = [freq_d[i] for i in valid_idx]
        freq_u = [freq_u[i] for i in valid_idx]
        self.factor = _downsamplingfactor(freq_u, fs)

        self.sos_filters = []
        for lower, upper, fac in zip(freq_d, freq_u, self.factor):
            fsd = fs / fac
            sos = signal.butter(
                N=order,
                Wn=np.array([lower, upper]) / (fsd / 2),
                btype='bandpass',
                analog=False,
                output='sos'
            )
            self.sos_filters.append(sos)

    def filter_signal_to_leq_bands(self, x):
        leq_bands = np.zeros(len(self.freq))
        for idx in range(len(self.freq)):
            sd = signal.resample(x, round(len(x) / self.factor[idx]))
            y = signal.sosfilt(self.sos_filters[idx], sd)
            rms = np.sqrt(np.mean(y ** 2))
            if rms > 1e-10:
                leq_bands[idx] = MIC_OFFSET_DB + MIC_REF_DB + 20 * np.log10(rms / MIC_REF_AMPL)
            else:
                leq_bands[idx] = 0.0
        return leq_bands


# ===================== LEQ AVERAGING =====================
def _leq_average(leq_array_db: np.ndarray, n: int) -> np.ndarray:
    """
    Hitung rata-rata Leq secara logaritmik dari n pengukuran.

    Formula: 10 * log10( (1/n) * Σ 10^(Leq_i / 10) )
    
    Parameter:
        leq_array_db : array 2D shape (n, n_bands) atau 1D shape (n,)
                       berisi nilai Leq dalam dB untuk setiap detik
        n            : jumlah pengukuran (pembagi)
    
    Return:
        array 1D shape (n_bands,) atau skalar — Leq rata-rata dalam dB
    """
    linear = 10 ** (leq_array_db / 10)        # konversi dB ke linear
    linear_mean = np.sum(linear, axis=0) / n   # ini rata-rata linear dibagi dengan 60
    # Hindari log10(0). ganti nilai <= 0 dengan nilai sangat kecil
    linear_mean = np.where(linear_mean > 1e-10, linear_mean, 1e-10)
    return 10 * np.log10(linear_mean)          # kembali ke dB


# ===================== PROCESS 2: OCTAVE ANALYZER =====================
def octave_leq_analyzer_process(
    raw_queue: mp.Queue,
    result_queue: mp.Queue,
    weighting: str,
    display_mode: str,
    stat_second: mp.Value,
    stop_event: mp.Event
):
    """
    Proses paling berat — filter bank 31 band + Leq calculation.
    Berjalan di core tersendiri supaya scipy tidak dibatasi GIL.
    """

    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Analyzer] %(message)s')
    logger = logging.getLogger()
    logger.info(f"Octave Analyzer Process started | Weighting: {weighting} | Mode: {display_mode}")

    filter_bank = OctaveFilterBank(SAMPLE_RATE, OCTAVE_FRACTION, OCTAVE_ORDER, FREQ_LIMITS)
    weighting_corrections = WEIGHTING_DICT[weighting]
    n_bands = len(filter_bank.freq)

    # --- Akumulasi 1 detik (selalu aktif) ---
    packet_counter = 0
    all_samples = np.zeros(SAMPLES_SHORT * PACKETS_PER_SECOND, dtype=np.float32)
    current_lat = 0.0
    current_lon = 0.0
    second_count = 1  # total detik yang sudah diproses

    # ---  Akumulasi 60 detik (hanya aktif di mode 'minute') ---
    # Buffer menyimpan leq_bands_raw tiap detik, shape: (SECONDS_PER_MINUTE, n_bands) -> baris x kolom
    minute_buffer_bands = np.zeros((SECONDS_PER_MINUTE, n_bands), dtype=np.float64)
    # Buffer menyimpan leq_total_raw tiap detik, shape: (SECONDS_PER_MINUTE,)
    minute_buffer_total = np.zeros(SECONDS_PER_MINUTE, dtype=np.float64)
    minute_second_idx = 0   # index dalam buffer 60 detik (0..59)
    last_lat = 0.0
    last_lon = 0.0

    while not stop_event.is_set():
        try:
            packet = raw_queue.get(timeout=1.0)
        except:
            continue

        samples = packet['samples']
        current_lat = packet['lat']
        current_lon = packet['lon']

        # Akumulasi 8 paket → 1 detik
        start_idx = packet_counter * SAMPLES_SHORT
        all_samples[start_idx:start_idx + SAMPLES_SHORT] = samples
        packet_counter += 1

        if packet_counter >= PACKETS_PER_SECOND:

            # === KALKULASI PER DETIK (selalu aktif) ===
            leq_bands_raw = filter_bank.filter_signal_to_leq_bands(all_samples)
            leq_bands_weighted = leq_bands_raw + weighting_corrections

            linear_sum_w = np.sum(10 ** (leq_bands_weighted / 10))
            leq_total_weighted = 10 * np.log10(linear_sum_w) if linear_sum_w > 1e-10 else 0.0

            linear_sum_r = np.sum(10 ** (leq_bands_raw / 10))
            leq_total_raw = 10 * np.log10(linear_sum_r) if linear_sum_r > 1e-10 else 0.0

            timestamp = time.strftime("%H:%M:%S")
            last_lat = current_lat
            last_lon = current_lon

            # === MODE 'second': langsung kirim ke Publisher ===
            if display_mode == 'second':
                result = {
                    'leq_total_weighted': float(leq_total_weighted),
                    'leq_total_raw': float(leq_total_raw),
                    'leq_bands_weighted': leq_bands_weighted.tolist(),
                    'leq_bands_raw': leq_bands_raw.tolist(),
                    'freq_bands': filter_bank.freq,
                    'weighting': weighting,
                    'display_mode': 'second',
                    'timestamp': timestamp,
                    'lat': float(current_lat),
                    'lon': float(current_lon),
                    'second': second_count,
                    'label': f"Second #{second_count}"
                }
                # kirim hasil ke Publisher dengan non-blocking
                try:
                    result_queue.put_nowait(result)
                except:
                    pass    # publisher sibuk ya skip aja

            # === MODE 'minute': akumulasi dulu, kirim tiap 60 detik ===
            elif display_mode == 'minute':
                # Simpan nilai per detik (raw, sebelum weighting)
                # Weighting akan diterapkan kembali setelah averaging
                minute_buffer_bands[minute_second_idx] = leq_bands_raw
                minute_buffer_total[minute_second_idx] = leq_total_raw
                minute_second_idx += 1

                # Setiap 60 detik → hitung Leq 1 menit dan kirim
                if minute_second_idx >= SECONDS_PER_MINUTE:
                    minute_count = second_count // SECONDS_PER_MINUTE  # menit keberapa

                    # --- Leq 1 menit per band (raw) ---
                    # Shape input: (60, n_bands) → output: (n_bands,)
                    leq_1min_bands_raw = _leq_average(minute_buffer_bands, SECONDS_PER_MINUTE)

                    # --- ini weighting setelah averaging ---
                    leq_1min_bands_weighted = leq_1min_bands_raw + weighting_corrections

                    # --- Leq 1 menit total (dari 60 nilai leq_total_raw) ---
                    # Shape input: (60,) → output: scalar
                    leq_1min_total_raw = float(
                        _leq_average(minute_buffer_total, SECONDS_PER_MINUTE)
                    )

                    # --- Leq 1 menit total weighted (dari 60 nilai leq_total_weighted) ---
                    # Hitung ulang leq_total_weighted per detik dari buffer bands raw + weighting
                    # total = penjumlahan logaritmik dari 31 band weighted
                    leq_1min_total_weighted_val = float(
                        _leq_average(
                            np.array([
                                10 * np.log10(max(np.sum(10 ** ((minute_buffer_bands[i] + weighting_corrections) / 10)), 1e-10))
                                for i in range(SECONDS_PER_MINUTE)
                            ]),
                            SECONDS_PER_MINUTE
                        )
                    )

                    result = {
                        'leq_total_weighted': leq_1min_total_weighted_val,
                        'leq_total_raw': leq_1min_total_raw,
                        'leq_bands_weighted': leq_1min_bands_weighted.tolist(),
                        'leq_bands_raw': leq_1min_bands_raw.tolist(),
                        'freq_bands': filter_bank.freq,
                        'weighting': weighting,
                        'display_mode': 'minute',
                        'timestamp': timestamp,
                        'lat': float(last_lat),
                        'lon': float(last_lon),
                        'second': second_count,           # total detik (untuk PLOT_SECONDS)
                        'minute': minute_count,
                        'label': f"Minute #{minute_count} (second #{second_count})"
                    }
                    try:
                        result_queue.put_nowait(result)
                    except:
                        pass

                    # Reset buffer menit
                    minute_second_idx = 0
                    minute_buffer_bands[:] = 0.0
                    minute_buffer_total[:] = 0.0

            # Reset akumulasi detik
            packet_counter = 0
            second_count += 1

            with stat_second.get_lock():
                stat_second.value = second_count

    logger.info("Octave Analyzer Process stopped.")


# ===================== PROCESS 3: PUBLISHER =====================
def publisher_process(
    result_queue: mp.Queue,
    plot_seconds: list,
    stop_event: mp.Event
):
    """
    Proses untuk output: print, plot, MQTT.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [Publisher] %(message)s')
    logger = logging.getLogger()
    logger.info("Publisher Process started")

    plotted_seconds = set()
    plot_seconds_set = set(plot_seconds)

    nominal_labels = [
        "20", "25", "31.5", "40", "50", "63", "80", "100", "125", "160", "200",
        "250", "315", "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k",
        "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k", "16k", "20k"
    ]

    while not stop_event.is_set():
        try:
            result = result_queue.get(timeout=1.0)
        except:
            continue

        w = result['weighting']
        ts = result['timestamp']
        leq_w = result['leq_total_weighted']
        label = result['label']
        mode = result['display_mode']

        # --- Print terminal ---
        unit_label = "minute" if mode == 'minute' else "second"
        print(f"[{ts}] {label}: L{w}eq = {leq_w:.1f} dB{w}  ({unit_label})")

        # --- Plot hanya yang di request aja (cek berdasarkan total second, konsisten di kedua mode) ---
        sec = result['second']
        if sec in plot_seconds_set and sec not in plotted_seconds:
            _plot_result(result, nominal_labels, logger)
            plotted_seconds.add(sec)

        # --- MQTT publish (uncomment aje kalau udah mau dipake) ---
        # _publish_mqtt(result)

    logger.info("Publisher Process stopped.")


def _plot_result(result, nominal_labels, logger):
    """Plot function — berjalan di Publisher Process, jadi ga block analyzer"""
    try:
        leq_bands_weighted = result['leq_bands_weighted']
        leq_total_weighted = result['leq_total_weighted']
        freq_bands = result['freq_bands']
        w = result['weighting']
        label = result['label']
        mode = result['display_mode']

        freq_labels = nominal_labels[:len(freq_bands)]
        fig, ax = plt.subplots(figsize=(14, 7))
        x_pos = np.arange(len(freq_bands))
        bars = ax.bar(x_pos, leq_bands_weighted, color='steelblue', alpha=0.7,
                      edgecolor='black', linewidth=0.5)

        for bar, level in zip(bars, leq_bands_weighted):
            bar.set_color('red' if level > 80 else ('orange' if level > 65 else 'green'))
            ax.text(bar.get_x() + bar.get_width() / 2., bar.get_height() + 1.5,
                    f'{level:.1f}', ha='center', va='bottom', fontsize=8, fontweight='bold')

        ax.set_xticks(x_pos)
        ax.set_xticklabels(freq_labels, rotation=45, ha='right', fontsize=10)
        ax.set_xlabel('Frequency (Hz)', fontsize=12, fontweight='bold')
        ax.set_ylabel(f'L{w}eq (dB{w})', fontsize=12, fontweight='bold')

        # Judul menyesuaikan mode
        duration_str = "1-minute" if mode == 'minute' else "1-second"
        ax.set_title(
            f'1/3 Octave Band L{w}eq [{duration_str}] [{label}]\n'
            f'L{w}eq_total = {leq_total_weighted:.1f} dB{w}',
            fontsize=14, fontweight='bold'
        )
        ax.set_ylim(20, 125)
        ax.grid(True, alpha=0.3, axis='y', linestyle='--')
        plt.tight_layout()

        # Nama file menyesuaikan mode
        sec = result['second']
        filename = f'leq_{w}_{mode}_s{sec}.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        logger.info(f"Plot saved: {filename}")
        plt.close()
        gc.collect()

    except Exception as e:
        logger.error(f"Plot error: {e}")


# ===================== MAIN =====================
def main():
    # kalau di macos or windows pake spawn
    mp.set_start_method('spawn')  # 'fork' untuk Linux/Raspberry Pi (lebih cepat dari 'spawn')

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)

    logger.info("=" * 70)
    logger.info("Noise Monitoring - Multiprocessing + Display Mode")
    logger.info(f"Weighting    : {WEIGHTING}-weighting")
    logger.info(f"Display Mode : {DISPLAY_MODE}")
    logger.info(f"CPU cores    : {mp.cpu_count()}")
    if DISPLAY_MODE == 'minute':
        logger.info(f"Accumulation : {SECONDS_PER_MINUTE} seconds per output")
        logger.info(f"Plot at      : second(s) {PLOT_SECONDS} "
                    f"→ minute(s) {[s // SECONDS_PER_MINUTE for s in PLOT_SECONDS]}")
    else:
        logger.info(f"Plot at      : second(s) {PLOT_SECONDS}")
    logger.info("=" * 70)

    # Shared objects
    raw_queue    = mp.Queue(maxsize=RAW_QUEUE_SIZE) # P1 → P2
    result_queue = mp.Queue(maxsize=RESULT_QUEUE_SIZE) # P2 → P3
    stop_event   = mp.Event()

    # Shared memory untuk statistik (mp.Value lebih ringan dari Queue untuk angka)
    stat_pkt_received  = mp.Value('i', 0)
    stat_pkt_corrupted = mp.Value('i', 0)
    stat_second        = mp.Value('i', 0)

    # Buat proses
    p_reader = mp.Process(
        target=serial_reader_process,
        args=(raw_queue, stat_pkt_received, stat_pkt_corrupted, stop_event),
        name="SerialReader",
        daemon=True
    )
    p_analyzer = mp.Process(
        target=octave_leq_analyzer_process,
        args=(raw_queue, result_queue, WEIGHTING, DISPLAY_MODE, stat_second, stop_event),
        name="OctaveAnalyzer",
        daemon=True
    )
    p_publisher = mp.Process(
        target=publisher_process,
        args=(result_queue, PLOT_SECONDS, stop_event),
        name="Publisher",
        daemon=True
    )

    # === Start ===
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
                logger.error("Serial Reader Process died!")
                break
            if not p_analyzer.is_alive():
                logger.error("Octave Analyzer Process died!")
                break
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        logger.info("Stopping all processes...")
    finally:
        stop_event.set()  # sinyal stop ke semua proses

        # tunggu proses berhenti dengan timeout
        for p in [p_reader, p_analyzer, p_publisher]:
            p.join(timeout=5)
            if p.is_alive():
                logger.warning(f"{p.name} tidak berhenti, force terminate...")
                p.terminate()

        # statistik akhir
        print("\n" + "=" * 70)
        logger.info("=== Final Statistics ===")
        logger.info(f"Packets received   : {stat_pkt_received.value}")
        logger.info(f"Packets corrupted  : {stat_pkt_corrupted.value}")
        total = stat_pkt_received.value + stat_pkt_corrupted.value
        if total > 0:
            logger.info(f"Success rate       : {stat_pkt_received.value / total * 100:.2f}%")
        logger.info(f"Total seconds      : {stat_second.value}")
        if DISPLAY_MODE == 'minute':
            logger.info(f"Total minutes      : {stat_second.value // SECONDS_PER_MINUTE}")
        logger.info("=" * 70)


if __name__ == "__main__":
    main()