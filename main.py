
'''
#!/usr/bin/env python3
"""
Optimized Noise Monitoring System - Leq with A/C/Z Weighting
Flow: Serial → OctaveAnalyzer → Leq per band → Apply weighting → Leq total
"""

import serial
import struct
import numpy as np
import time
import logging
import threading
import queue
from scipy import signal
import matplotlib.pyplot as plt
import gc

# ===================== CONFIGURATION =====================
UART_PORT = '/dev/ttyACM0'
UART_BAUD = 115200
SAMPLE_RATE = 48000
SAMPLES_SHORT = 6000

# Frame markers
FRAME_START_MARKER = 0xAA55AA55
FRAME_END_MARKER = 0x55AA55AA

# Queue size
RAW_QUEUE_SIZE = 50

# 1/3 Octave Band Configuration
OCTAVE_FRACTION = 3
OCTAVE_ORDER = 6
FREQ_LIMITS = [20, 20000]

# Leq Configuration
LEQ_PERIOD = 1
PACKETS_PER_SECOND = 8
MIC_SENSITIVITY = -26
MIC_REF_DB = 94.0
MIC_OFFSET_DB = 3.0103
MIC_BITS = 24
MIC_REF_AMPL = pow(10, MIC_SENSITIVITY/20) * ((1 << (MIC_BITS-1)) - 1)

# ===================== WEIGHTING SELECTION =====================
# User can select: 'A', 'C', or 'Z'
WEIGHTING = 'A'  # ← EDIT THIS: Options: 'A', 'C', 'Z'

# ===================== WEIGHTING CORRECTIONS (dB) =====================
# Corrections for each of the 31 frequency bands (20Hz to 20kHz)
# TODO: Replace dummy values with actual weighting correction values

# A-weighting corrections (IEC 61672-1) - DUMMY VALUES, REPLACE LATER
A_WEIGHTING_CORRECTIONS = np.array([
    -50.5, -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9, -8.6, -6.6, -4.8, -3.2, -1.9, -0.8, 0.0, 0.6, 1.0,
    1.2, 1.3, 1.2, 1.0, 0.5, -0.1, -1.1, -2.5, -4.3, -6.6,
    -9.3
])  # 31 values for 31 bands

# C-weighting corrections (IEC 61672-1) - DUMMY VALUES, REPLACE LATER
C_WEIGHTING_CORRECTIONS = np.array([
    -6.2, -4.4, -3.0, -2.0, -1.3, -0.8, -0.5, -0.3, -0.2, -0.1,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1,
    -0.2, -0.3, -0.5, -0.8, -1.3, -2.0, -3.0, -4.4, -6.2, -8.5,
    -11.2
])  # 31 values for 31 bands

# Z-weighting (flat response, no correction)
Z_WEIGHTING_CORRECTIONS = np.zeros(31)  # All zeros

# Dictionary to select weighting
WEIGHTING_DICT = {
    'A': A_WEIGHTING_CORRECTIONS,
    'C': C_WEIGHTING_CORRECTIONS,
    'Z': Z_WEIGHTING_CORRECTIONS
}

# Validate user input
if WEIGHTING not in WEIGHTING_DICT:
    raise ValueError(f"Invalid WEIGHTING '{WEIGHTING}'. Must be 'A', 'C', or 'Z'.")

# ===================== PLOT CONFIGURATION =====================
PLOT_SECONDS = [1, 5, 10]  # Manual selection for plotting

# ===================== GLOBAL DATA STORAGE FOR MQTT =====================
# Dictionary to store latest Leq bands data (ready for MQTT publish)
leq_data_mqtt = {
    'leq_total_weighted': 0.0,      # LAeq, LCeq, or LZeq (depends on WEIGHTING)
    'leq_total_raw': 0.0,           # LZeq (always unweighted)
    'leq_bands_weighted': [],       # 31 values (with weighting applied)
    'leq_bands_raw': [],            # 31 values (without weighting)
    'freq_bands': [],               # 31 frequency values
    'weighting': WEIGHTING,         # 'A', 'C', or 'Z'
    'timestamp': '',
    'lat': 0.0,
    'lon': 0.0,
    'second': 0
}

# Frame structure
FRAME_SIZE = 4 + 4 + 4 + 4 + (SAMPLES_SHORT * 4) + 4

# ===================== LOGGING =====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ===================== PRE-CALCULATED FILTER BANK =====================
class OctaveFilterBank:
    """Pre-calculated 1/3 Octave filter coefficients for fast filtering"""
    
    def __init__(self, fs, fraction, order, limits):
        self.fs = fs
        self.fraction = fraction
        self.order = order
        
        logger.info("Pre-calculating filter coefficients...")
        start = time.time()
        
        from octave_filter import getansifrequencies, _downsamplingfactor
        
        freq, freq_d, freq_u = getansifrequencies(fraction, limits)
        
        nyquist = fs / 2
        valid_idx = [i for i, f in enumerate(freq_u) if f < nyquist]
        
        self.freq = [freq[i] for i in valid_idx]
        freq_d = [freq_d[i] for i in valid_idx]
        freq_u = [freq_u[i] for i in valid_idx]
        
        self.factor = _downsamplingfactor(freq_u, fs)
        
        self.sos_filters = []
        for idx, (lower, upper) in enumerate(zip(freq_d, freq_u)):
            fsd = fs / self.factor[idx]
            sos = signal.butter(
                N=order,
                Wn=np.array([lower, upper]) / (fsd / 2),
                btype='bandpass',
                analog=False,
                output='sos'
            )
            self.sos_filters.append(sos)
        
        elapsed = time.time() - start
        logger.info(f"Filter bank ready: {len(self.freq)} bands in {elapsed:.2f}s")
    
    def filter_signal_to_leq_bands(self, x):
        """
        Apply filters and calculate Leq for each frequency band (RAW, no weighting)
        Returns: leq_bands_raw (31 values in dB)
        """
        leq_bands = np.zeros(len(self.freq))
        
        for idx in range(len(self.freq)):
            # Resample
            sd = signal.resample(x, round(len(x) / self.factor[idx]))
            
            # Apply pre-calculated filter
            y = signal.sosfilt(self.sos_filters[idx], sd)
            
            # Calculate RMS for this band
            rms = np.sqrt(np.mean(y ** 2))
            
            # Calculate Leq for this band (unweighted)
            if rms > 1e-10:
                leq_bands[idx] = MIC_OFFSET_DB + MIC_REF_DB + 20 * np.log10(rms / MIC_REF_AMPL)
            else:
                leq_bands[idx] = 0.0
        
        return leq_bands

# ===================== RING BUFFER =====================
class RingBuffer:
    """Fixed-size ring buffer to prevent memory growth"""
    
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
            result = bytes(self.buffer[self.head:self.head + length])
            return result
        else:
            first_part = self.capacity - self.head
            result = bytes(self.buffer[self.head:]) + bytes(self.buffer[:length - first_part])
            return result
    
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

# ===================== THREAD 1: SERIAL READER =====================
class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, raw_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.raw_queue = raw_queue
        self.running = True
        self.buffer = RingBuffer(capacity=FRAME_SIZE * 10)
        self.packets_received = 0
        self.packets_corrupted = 0
        self.struct_fmt = f'<II2f{SAMPLES_SHORT}fI'
        self.start_marker_bytes = struct.pack('<I', FRAME_START_MARKER)

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            logger.info(f"Serial Reader started: {self.port}")
            
            while self.running:
                if ser.in_waiting > 0:
                    chunk = ser.read(max(ser.in_waiting, 8192))
                    self.buffer.extend(chunk)

                while len(self.buffer) >= FRAME_SIZE:
                    first_bytes = self.buffer.get_bytes(4)
                    if first_bytes != self.start_marker_bytes:
                        idx = self.buffer.find(self.start_marker_bytes)
                        if idx == -1:
                            self.buffer.consume(max(0, len(self.buffer) - 3))
                            break
                        else:
                            self.buffer.consume(idx)
                            continue
                    
                    frame_bytes = self.buffer.get_bytes(FRAME_SIZE)
                    if frame_bytes is None:
                        break
                    
                    end_marker = struct.unpack('<I', frame_bytes[-4:])[0]
                    if end_marker == FRAME_END_MARKER:
                        try:
                            data = struct.unpack(self.struct_fmt, frame_bytes)
                            samples_start = 16
                            samples_end = samples_start + (SAMPLES_SHORT * 4)
                            samples = np.frombuffer(frame_bytes[samples_start:samples_end], dtype=np.float32)
                            
                            if not self.raw_queue.full():
                                packet = {
                                    'samples': samples.copy(),
                                    'lat': data[2],
                                    'lon': data[3]
                                }
                                self.raw_queue.put(packet, block=False)
                                self.packets_received += 1
                        except Exception as e:
                            self.packets_corrupted += 1
                    else:
                        self.packets_corrupted += 1
                    
                    self.buffer.consume(FRAME_SIZE)
        except Exception as e:
            logger.error(f"Serial Reader Error: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()

    def stop(self):
        self.running = False

# ===================== THREAD 2: OCTAVE ANALYZER & LEQ CALCULATOR (WITH WEIGHTING) =====================
class OctaveLeqAnalyzer(threading.Thread):
    """
    Combined Octave Analysis + Leq Calculation with A/C/Z Weighting
    - Accumulates 8 packets (1 second)
    - Filters into 31 frequency bands
    - Calculates Leq per band (raw)
    - Applies weighting correction
    - Calculates Leq total (weighted and raw)
    """
    
    def __init__(self, raw_queue, filter_bank, weighting, plot_seconds):
        super().__init__(daemon=True)
        self.raw_queue = raw_queue
        self.filter_bank = filter_bank
        self.weighting = weighting
        self.weighting_corrections = WEIGHTING_DICT[weighting]
        self.running = True
        
        # Accumulation for 1 second
        self.packet_counter = 0
        self.all_samples = np.zeros(SAMPLES_SHORT * PACKETS_PER_SECOND, dtype=np.float32)
        
        # Metadata
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.second_count = 0
        
        # Plotting
        self.plot_seconds = set(plot_seconds)
        self.plotted_seconds = set()

    def run(self):
        logger.info(f"Octave+Leq Analyzer started (Weighting: {self.weighting})")
        logger.info(f"Will plot at seconds: {sorted(self.plot_seconds)}")
        
        while self.running:
            try:
                # Get packet from raw queue
                packet = self.raw_queue.get(timeout=1)
                samples = packet['samples']
                self.current_lat = packet['lat']
                self.current_lon = packet['lon']
                
                # Accumulate samples
                start_idx = self.packet_counter * SAMPLES_SHORT
                end_idx = start_idx + SAMPLES_SHORT
                self.all_samples[start_idx:end_idx] = samples
                
                self.packet_counter += 1
                
                # When 8 packets accumulated (1 second)
                if self.packet_counter >= PACKETS_PER_SECOND:
                    self._process_one_second()
                    
                    # Reset
                    self.packet_counter = 0
                    self.second_count += 1
                    
            except queue.Empty:
                continue
    
    def _process_one_second(self):
        """Process 1 second of data: octave filtering + weighting + Leq calculation"""
        
        # 1. Apply octave filtering and get Leq per band (RAW, no weighting)
        leq_bands_raw = self.filter_bank.filter_signal_to_leq_bands(self.all_samples)
        
        # 2. Apply weighting correction
        leq_bands_weighted = leq_bands_raw + self.weighting_corrections
        
        # 3. Calculate Leq total (weighted)
        leq_total_weighted = self._calculate_leq_total_from_bands(leq_bands_weighted)
        
        # 4. Calculate Leq total (raw, for comparison)
        leq_total_raw = self._calculate_leq_total_from_bands(leq_bands_raw)
        
        # 5. Store in global dictionary for MQTT
        timestamp = time.strftime("%H:%M:%S")
        global leq_data_mqtt
        leq_data_mqtt = {
            'leq_total_weighted': float(leq_total_weighted),
            'leq_total_raw': float(leq_total_raw),
            'leq_bands_weighted': leq_bands_weighted.tolist(),
            'leq_bands_raw': leq_bands_raw.tolist(),
            'freq_bands': self.filter_bank.freq,
            'weighting': self.weighting,
            'timestamp': timestamp,
            'lat': float(self.current_lat),
            'lon': float(self.current_lon),
            'second': self.second_count
        }
        
        # 6. Print to terminal (weighted Leq only)
        weighting_label = f"L{self.weighting}eq"  # LAeq, LCeq, or LZeq
        weighting_unit = f"dB{self.weighting}"    # dBA, dBC, or dBZ
        print(f"[{timestamp}] Second #{self.second_count}: {weighting_label} = {leq_total_weighted:.1f} {weighting_unit}")
        
        # 7. Plot if requested (show weighted values)
        if self.second_count in self.plot_seconds and self.second_count not in self.plotted_seconds:
            self._plot_leq_bands(leq_bands_weighted, leq_total_weighted, timestamp)
            self.plotted_seconds.add(self.second_count)
            
            if self.plotted_seconds == self.plot_seconds:
                logger.info("All requested plots completed. Continuing monitoring...")
    
    def _calculate_leq_total_from_bands(self, leq_bands):
        """
        Calculate total Leq from individual band Leq values using logarithmic sum
        Formula: Leq_total = 10 * log10(sum(10^(Leq_i / 10)))
        """
        # Convert dB to linear scale, sum, convert back to dB
        linear_sum = np.sum(10 ** (leq_bands / 10))
        
        if linear_sum > 1e-10:
            leq_total = 10 * np.log10(linear_sum)
        else:
            leq_total = 0.0
        
        return leq_total
    
    def _plot_leq_bands(self, leq_bands_weighted, leq_total_weighted, timestamp):
        """Generate and save weighted Leq bands plot"""
        try:
            freq_bands = self.filter_bank.freq
            
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # Nominal Center Frequencies (ISO standard)
            nominal_labels = [
                "20", "25", "31.5", "40", "50", "63", "80", "100", "125", "160", "200", 
                "250", "315", "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k", 
                "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k", "16k", "20k"
            ]
            freq_labels = nominal_labels[:len(freq_bands)]
            
            # Bar chart
            x_pos = np.arange(len(freq_bands))
            bars = ax.bar(x_pos, leq_bands_weighted, color='steelblue', alpha=0.7)
            
            # Color coding
            for bar, level in zip(bars, leq_bands_weighted):
                if level > 80:
                    bar.set_color('red')
                elif level > 65:
                    bar.set_color('orange')
                else:
                    bar.set_color('green')
            
            ax.set_xticks(x_pos)
            ax.set_xticklabels(freq_labels, rotation=45, ha='right')
            ax.set_xlabel('Frequency (Hz)', fontsize=12)
            
            # Y-axis label with weighting
            weighting_label_y = f"L{self.weighting}eq (dB{self.weighting})"
            ax.set_ylabel(weighting_label_y, fontsize=12)
            
            # Title with weighting
            weighting_label_title = f"L{self.weighting}eq"
            weighting_unit = f"dB{self.weighting}"
            ax.set_title(
                f'1/3 Octave Band {weighting_label_title} [Second #{self.second_count}] [{timestamp}]\n'
                f'{weighting_label_title}_total = {leq_total_weighted:.1f} {weighting_unit}',
                fontsize=14
            )
            ax.set_ylim(20, 120)
            ax.grid(True, alpha=0.3, axis='y')
            
            plt.tight_layout()
            
            # Save with unique filename
            filename = f'leq_bands_{self.weighting}weighted_second_{self.second_count}.png'
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            logger.info(f"Plot saved: {filename}")
            
            plt.show(block=False)
            plt.pause(2)
            plt.close()
            
            gc.collect()
            
        except Exception as e:
            logger.error(f"Plot error: {e}")
    
    def stop(self):
        self.running = False

# ===================== MQTT PUBLISH HELPER (COMMENTED OUT) =====================
def publish_to_mqtt():
    """
    MQTT publish function - COMMENTED OUT (focus on functionality, not IoT)
    Data is ready in leq_data_mqtt dictionary
    """
    # import paho.mqtt.client as mqtt
    # import json
    # 
    # client = mqtt.Client()
    # client.connect('localhost', 1883, 60)
    # 
    # mqtt_payload = {
    #     'leq_total_weighted': leq_data_mqtt['leq_total_weighted'],
    #     'leq_total_raw': leq_data_mqtt['leq_total_raw'],
    #     'weighting': leq_data_mqtt['weighting'],
    #     'timestamp': leq_data_mqtt['timestamp'],
    #     'lat': leq_data_mqtt['lat'],
    #     'lon': leq_data_mqtt['lon']
    # }
    # 
    # # Add frequency bands
    # for i, (freq, leq_w, leq_r) in enumerate(zip(
    #     leq_data_mqtt['freq_bands'], 
    #     leq_data_mqtt['leq_bands_weighted'],
    #     leq_data_mqtt['leq_bands_raw']
    # )):
    #     mqtt_payload[f'freq_{int(freq)}Hz_weighted'] = leq_w
    #     mqtt_payload[f'freq_{int(freq)}Hz_raw'] = leq_r
    # 
    # client.publish('noise/leq_data', json.dumps(mqtt_payload))
    # client.disconnect()
    
    pass

# ===================== MAIN =====================
def main():
    logger.info("="*70)
    logger.info("Noise Monitoring - Leq with A/C/Z Weighting")
    logger.info("="*70)
    logger.info(f"Platform: Raspberry Pi 3")
    logger.info(f"Sample Rate: {SAMPLE_RATE} Hz")
    logger.info(f"Weighting: {WEIGHTING}-weighting")
    logger.info(f"Leq Calculation: From 31 frequency bands (logarithmic sum)")
    
    if PLOT_SECONDS:
        logger.info(f"Plots will be generated at seconds: {sorted(PLOT_SECONDS)}")
    else:
        logger.info("No plots will be generated (monitoring only)")
    
    # Step 1: Pre-calculate filter bank
    filter_bank = OctaveFilterBank(SAMPLE_RATE, OCTAVE_FRACTION, OCTAVE_ORDER, FREQ_LIMITS)
    
    # Step 2: Create communication queue
    raw_queue = queue.Queue(maxsize=RAW_QUEUE_SIZE)
    
    # Step 3: Create threads
    reader = SerialReader(UART_PORT, UART_BAUD, raw_queue)
    analyzer = OctaveLeqAnalyzer(raw_queue, filter_bank, WEIGHTING, PLOT_SECONDS)
    
    # Step 4: Start threads
    reader.start()
    time.sleep(0.5)
    analyzer.start()
    
    logger.info("="*70)
    logger.info("System running. Press Ctrl+C to stop.")
    logger.info("="*70)
    print()
    
    try:
        while True:
            time.sleep(1)
            
            # MQTT publish is commented out (focus on functionality)
            # if analyzer.second_count % 5 == 0:
            #     publish_to_mqtt()
            
            if not reader.is_alive():
                logger.error("Serial reader died!")
                break
                
    except KeyboardInterrupt:
        print("\n" + "="*70)
        logger.info("Stopping system...")
    finally:
        reader.stop()
        analyzer.stop()
        
        time.sleep(1)
        
        print("\n" + "="*70)
        logger.info("=== Final Statistics ===")
        logger.info(f"Packets received: {reader.packets_received}")
        logger.info(f"Packets corrupted: {reader.packets_corrupted}")
        if reader.packets_received > 0:
            success_rate = (reader.packets_received / 
                          (reader.packets_received + reader.packets_corrupted)) * 100
            logger.info(f"Success rate: {success_rate:.2f}%")
        logger.info(f"Leq measurements: {analyzer.second_count}")
        logger.info(f"Plots generated: {len(analyzer.plotted_seconds)}")
        logger.info("="*70)
        logger.info("System stopped cleanly.")
        
        # Print last MQTT data for verification
        logger.info("\n=== Last MQTT Data (Ready to Send) ===")
        logger.info(f"Weighting: {leq_data_mqtt['weighting']}")
        logger.info(f"L{leq_data_mqtt['weighting']}eq_total: {leq_data_mqtt['leq_total_weighted']:.1f} dB{leq_data_mqtt['weighting']}")
        logger.info(f"LZeq_total (raw): {leq_data_mqtt['leq_total_raw']:.1f} dBZ")
        logger.info(f"Location: ({leq_data_mqtt['lat']:.5f}, {leq_data_mqtt['lon']:.5f})")
        logger.info(f"Frequency bands: {len(leq_data_mqtt['leq_bands_weighted'])}")

if __name__ == "__main__":
    main()

'''

#!/usr/bin/env python3
"""
Optimized Noise Monitoring System - Leq with A/C/Z Weighting
Flow: Serial → OctaveAnalyzer → Leq per band → Apply weighting → Leq total
"""

import serial
import struct
import numpy as np
import time
import logging
import threading
import queue
from scipy import signal
import matplotlib.pyplot as plt
import gc

# ===================== CONFIGURATION =====================
UART_PORT = '/dev/ttyACM0' # Komunikasi dilakukan secara serial
UART_BAUD = 115200
SAMPLE_RATE = 48000
SAMPLES_SHORT = 6000

# Frame marker
# Untuk menandakan awal dari paket dan akhir dari paket
FRAME_START_MARKER = 0xAA55AA55
FRAME_END_MARKER = 0x55AA55AA

# Queue size
RAW_QUEUE_SIZE = 50

# 1/3 Octave Band Configuration
OCTAVE_FRACTION = 3
OCTAVE_ORDER = 6
FREQ_LIMITS = [20, 20000]

# Leq Configuration
LEQ_PERIOD = 1
PACKETS_PER_SECOND = 8
MIC_SENSITIVITY = -26
MIC_REF_DB = 94.0
MIC_OFFSET_DB = 3.0103
MIC_BITS = 24
MIC_REF_AMPL = pow(10, MIC_SENSITIVITY/20) * ((1 << (MIC_BITS-1)) - 1) # nilai rujukan absolut untuk mengonversi sinyal digital kembali ke tekanan suara (Pascal/dB)

# ===================== WEIGHTING SELECTION =====================
WEIGHTING = 'A'  #  Opsi: 'A', 'C', 'Z'

# ===================== WEIGHTING CORRECTIONS (dB) =====================
# Corrections for each of the 31 frequency bands (20Hz to 20kHz)
# TODO: Replace dummy values with actual weighting correction values

# A-weighting corrections (IEC 61672-1) 
A_WEIGHTING_CORRECTIONS = np.array([
    -50.5, -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9, -8.6, -6.6, -4.8, -3.2, -1.9, -0.8, 0.0, 0.6, 1.0,
    1.2, 1.3, 1.2, 1.0, 0.5, -0.1, -1.1, -2.5, -4.3, -6.6,
    -9.3
])  # 31 values for 31 bands

# C-weighting corrections (IEC 61672-1) 
C_WEIGHTING_CORRECTIONS = np.array([
    -6.2, -4.4, -3.0, -2.0, -1.3, -0.8, -0.5, -0.3, -0.2, -0.1,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.1, -0.2, -0.3, -0.5, -0.8, -1.3, -2.0,
    -3.0
])  # 31 values for 31 bands

# Z-weighting (flat response, no correction)
Z_WEIGHTING_CORRECTIONS = np.zeros(31)  # All zeros

# Dictionary to select weighting
WEIGHTING_DICT = {
    'A': A_WEIGHTING_CORRECTIONS,
    'C': C_WEIGHTING_CORRECTIONS,
    'Z': Z_WEIGHTING_CORRECTIONS
}

# Validate user input
if WEIGHTING not in WEIGHTING_DICT:
    raise ValueError(f"Invalid WEIGHTING '{WEIGHTING}'. Must be 'A', 'C', or 'Z'.")

# ===================== PLOT CONFIGURATION =====================
PLOT_SECONDS = [1, 5, 10]  # Manual selection for plotting

# ===================== GLOBAL DATA STORAGE FOR MQTT =====================
# Dictionary to store latest Leq bands data (ready for MQTT publish)
leq_data_mqtt = {
    'leq_total_weighted': 0.0,      # LAeq, LCeq, or LZeq (depends on WEIGHTING)
    'leq_total_raw': 0.0,           # LZeq (always unweighted)
    'leq_bands_weighted': [],       # 31 values (with weighting applied)
    'leq_bands_raw': [],            # 31 values (without weighting)
    'freq_bands': [],               # 31 frequency values
    'weighting': WEIGHTING,         # 'A', 'C', or 'Z'
    'timestamp': '',
    'lat': 0.0,
    'lon': 0.0,
    'second': 0
}

# Frame structure
FRAME_SIZE = 4 + 4 + 4 + 4 + (SAMPLES_SHORT * 4) + 4

# ===================== LOGGING =====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ===================== PRE-CALCULATED FILTER BANK =====================
class OctaveFilterBank:
    """Pre-calculated 1/3 Octave filter coefficients for fast filtering"""
    
    def __init__(self, fs, fraction, order, limits):
        self.fs = fs
        self.fraction = fraction
        self.order = order
        
        logger.info("Pre-calculating filter coefficients...")
        start = time.time()
        
        from octave_filter import getansifrequencies, _downsamplingfactor
        
        freq, freq_d, freq_u = getansifrequencies(fraction, limits)
        # buat ngebuang target frekuensi diatas 24kHz di pembuatan filter untuk tiap bandnya ()
        nyquist = fs / 2
        valid_idx = [i for i, f in enumerate(freq_u) if f < nyquist]
        
        self.freq = [freq[i] for i in valid_idx]
        freq_d = [freq_d[i] for i in valid_idx]
        freq_u = [freq_u[i] for i in valid_idx]
        
        # satu angka integer untuk setiap band frekuensi. Angka ini nentuin seberapa banyak kita bisa mengurangi sampel data sebelum memfilternya
        # hanya frekuensi rendah aja dan hanya dimaksimalin cuma 50 angka (cuma 50 jumlah data  yang bisa kita kurangi)
        self.factor = _downsamplingfactor(freq_u, fs)
        
        self.sos_filters = []
        for idx, (lower, upper) in enumerate(zip(freq_d, freq_u)):
            fsd = fs / self.factor[idx]
            sos = signal.butter(
                N=order,
                Wn=np.array([lower, upper]) / (fsd / 2),
                btype='bandpass',
                analog=False,
                output='sos'
            )
            self.sos_filters.append(sos)
        
        elapsed = time.time() - start
        logger.info(f"Filter bank ready: {len(self.freq)} bands in {elapsed:.2f}s")
    
    def filter_signal_to_leq_bands(self, x):
        """
        Apply filters and calculate Leq for each frequency band (RAW, no weighting)
        Returns: leq_bands_raw (31 values in dB)
        """
        leq_bands = np.zeros(len(self.freq))
        
        # ngitung leq tiap bandnya
        for idx in range(len(self.freq)):
            # Resample
            # melakukan resampling menggunakan metode Transformasi Fourier (FFT) bawaan scipy
            # data yang masuk itu data yang terdiri dari 8 paket (6000*8)
            sd = signal.resample(x, round(len(x) / self.factor[idx]))

            # data yang masuk itu data yang terdiri dari 8 paket (6000*8) normalnya
            # tapi untuk frekuensi rendah akan lebih sedikit
            # Apply pre-calculated filter
            y = signal.sosfilt(self.sos_filters[idx], sd)
            
            # ngitung rms di bandnya
            rms = np.sqrt(np.mean(y ** 2))
            
            # unweighted leq di bandnya
            if rms > 1e-10:
                leq_bands[idx] = MIC_OFFSET_DB + MIC_REF_DB + 20 * np.log10(rms / MIC_REF_AMPL)
            else:
                leq_bands[idx] = 0.0
        
        return leq_bands

# ===================== RING BUFFER =====================

# kayak dua lokomotif kereta di jalur melingkar sebagai tempat transit data dari esp32 ke raspi

class RingBuffer:
    """Fixed-size ring buffer to prevent memory growth"""
    
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
            result = bytes(self.buffer[self.head:self.head + length])
            return result
        else:
            first_part = self.capacity - self.head
            result = bytes(self.buffer[self.head:]) + bytes(self.buffer[:length - first_part])
            return result
    
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

# ===================== THREAD 1: SERIAL READER =====================
# thread biar data yang diterima terus ada penanggung jawab yang mengirim dan menerimanya
class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, raw_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.raw_queue = raw_queue
        self.running = True
        self.buffer = RingBuffer(capacity=FRAME_SIZE * 10)
        self.packets_received = 0
        self.packets_corrupted = 0
        self.struct_fmt = f'<II2f{SAMPLES_SHORT}fI'
        self.start_marker_bytes = struct.pack('<I', FRAME_START_MARKER)

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            logger.info(f"Serial Reader started: {self.port}")
            
            while self.running:
                #langsung kirim ke ringbuffer
                if ser.in_waiting > 0:
                    chunk = ser.read(max(ser.in_waiting, 8192))
                    self.buffer.extend(chunk)

                while len(self.buffer) >= FRAME_SIZE:
                    first_bytes = self.buffer.get_bytes(4)
                    # cek itu frame start marker ga
                    if first_bytes != self.start_marker_bytes:
                        idx = self.buffer.find(self.start_marker_bytes)
                        if idx == -1:
                            self.buffer.consume(max(0, len(self.buffer) - 3))
                            break
                        else:
                            self.buffer.consume(idx)
                            continue
                    
                    frame_bytes = self.buffer.get_bytes(FRAME_SIZE)
                    if frame_bytes is None:
                        break
                    
                    end_marker = struct.unpack('<I', frame_bytes[-4:])[0]
                    if end_marker == FRAME_END_MARKER:
                        try:
                            # kalau udah nemu paketnya, mengkonversi data biner mentah ke data desimal supaya bisa diolah
                            data = struct.unpack(self.struct_fmt, frame_bytes)
                            samples_start = 16
                            samples_end = samples_start + (SAMPLES_SHORT * 4)
                            samples = np.frombuffer(frame_bytes[samples_start:samples_end], dtype=np.float32)
                            # di taro di raw_queue untuk diolah di octaveleqanalyzer
                            if not self.raw_queue.full():
                                packet = {
                                    'samples': samples.copy(),
                                    'lat': data[2],
                                    'lon': data[3]
                                }
                                self.raw_queue.put(packet, block=False)
                                self.packets_received += 1
                        except Exception as e:
                            self.packets_corrupted += 1
                    else:
                        self.packets_corrupted += 1
                    
                    self.buffer.consume(FRAME_SIZE)
        except Exception as e:
            logger.error(f"Serial Reader Error: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()

    def stop(self):
        self.running = False

# ===================== THREAD 2: OCTAVE ANALYZER & LEQ CALCULATOR (WITH WEIGHTING) =====================
class OctaveLeqAnalyzer(threading.Thread):
    """
    Combined Octave Analysis + Leq Calculation with A/C/Z Weighting
    - Accumulates 8 packets (1 second)
    - Filters into 31 frequency bands
    - Calculates Leq per band (raw)
    - Applies weighting correction
    - Calculates Leq total (weighted and raw)
    """
    
    def __init__(self, raw_queue, filter_bank, weighting, plot_seconds):
        super().__init__(daemon=True)
        self.raw_queue = raw_queue
        self.filter_bank = filter_bank
        self.weighting = weighting
        self.weighting_corrections = WEIGHTING_DICT[weighting]
        self.running = True
        
        # Accumulation for 1 second
        self.packet_counter = 0
        self.all_samples = np.zeros(SAMPLES_SHORT * PACKETS_PER_SECOND, dtype=np.float32)
        
        # Metadata
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.second_count = 0
        
        # Plotting
        self.plot_seconds = set(plot_seconds)
        self.plotted_seconds = set()

    def run(self):
        logger.info(f"Octave+Leq Analyzer started (Weighting: {self.weighting})")
        logger.info(f"Will plot at seconds: {sorted(self.plot_seconds)}")
        
        while self.running:
            try:
                # Get packet from raw queue
                packet = self.raw_queue.get(timeout=1)
                samples = packet['samples']
                self.current_lat = packet['lat']
                self.current_lon = packet['lon']
                
                # Accumulate samples
                start_idx = self.packet_counter * SAMPLES_SHORT
                end_idx = start_idx + SAMPLES_SHORT
                self.all_samples[start_idx:end_idx] = samples
                
                self.packet_counter += 1
                
                # nunggu 8 paket (1 second)
                if self.packet_counter >= PACKETS_PER_SECOND:
                    self._process_one_second()
                    
                    # Reset
                    self.packet_counter = 0
                    self.second_count += 1
                    
            except queue.Empty:
                continue
    
    def _process_one_second(self):
        """Process 1 second of data: octave filtering + weighting + Leq calculation"""
        
        # 1. oktaf filtering dan hitung leq per band (no weighting)
        leq_bands_raw = self.filter_bank.filter_signal_to_leq_bands(self.all_samples)
        
        # 2. ini buat weightingnya
        leq_bands_weighted = leq_bands_raw + self.weighting_corrections
        
        # 3. ini leqtotal dari seluruh band (weighitng)
        leq_total_weighted = self._calculate_leq_total_from_bands(leq_bands_weighted)
        
        # 4. Calculate Leq total (no wegithting untuk bandingin aje)
        leq_total_raw = self._calculate_leq_total_from_bands(leq_bands_raw)
        
        # 5. ini mqtt tapi nanti
        timestamp = time.strftime("%H:%M:%S")
        global leq_data_mqtt
        leq_data_mqtt = {
            'leq_total_weighted': float(leq_total_weighted),
            'leq_total_raw': float(leq_total_raw),
            'leq_bands_weighted': leq_bands_weighted.tolist(),
            'leq_bands_raw': leq_bands_raw.tolist(),
            'freq_bands': self.filter_bank.freq,
            'weighting': self.weighting,
            'timestamp': timestamp,
            'lat': float(self.current_lat),
            'lon': float(self.current_lon),
            'second': self.second_count
        }
        
        # 6. print terminal 
        weighting_label = f"L{self.weighting}eq"  # LAeq, LCeq, or LZeq
        weighting_unit = f"dB{self.weighting}"    # dBA, dBC, or dBZ
        print(f"[{timestamp}] Second #{self.second_count}: {weighting_label} = {leq_total_weighted:.1f} {weighting_unit}")
        
        # 7. Plot if requested (show weighted values)
        if self.second_count in self.plot_seconds and self.second_count not in self.plotted_seconds:
            self._plot_leq_bands(leq_bands_weighted, leq_total_weighted, timestamp)
            self.plotted_seconds.add(self.second_count)
            
            if self.plotted_seconds == self.plot_seconds:
                logger.info("All requested plots completed. Continuing monitoring...")
    
    def _calculate_leq_total_from_bands(self, leq_bands):
        """
        Calculate total Leq from individual band Leq values using logarithmic sum
        Formula: Leq_total = 10 * log10(sum(10^(Leq_i / 10)))
        """
        # penjumlahan logaritmik
        linear_sum = np.sum(10 ** (leq_bands / 10))
        
        if linear_sum > 1e-10:
            leq_total = 10 * np.log10(linear_sum)
        else:
            leq_total = 0.0
        
        return leq_total
    
    def _plot_leq_bands(self, leq_bands_weighted, leq_total_weighted, timestamp):
        """Generate and save weighted Leq bands plot with values displayed"""
        try:
            freq_bands = self.filter_bank.freq
            
            fig, ax = plt.subplots(figsize=(14, 7))
            
            # Nominal Center Frequencies (ISO standard)
            nominal_labels = [
                "20", "25", "31.5", "40", "50", "63", "80", "100", "125", "160", "200", 
                "250", "315", "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k", 
                "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k", "16k", "20k"
            ]
            freq_labels = nominal_labels[:len(freq_bands)]
            
            # Bar chart
            x_pos = np.arange(len(freq_bands))
            bars = ax.bar(x_pos, leq_bands_weighted, color='steelblue', alpha=0.7, edgecolor='black', linewidth=0.5)
            
            # Color coding and add value labels on top of bars
            for bar, level in zip(bars, leq_bands_weighted):
                # Color coding
                if level > 80:
                    bar.set_color('red')
                elif level > 65:
                    bar.set_color('orange')
                else:
                    bar.set_color('green')
                
                # Add text label on top of bar
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height + 1.5,
                       f'{level:.1f}',
                       ha='center', va='bottom', fontsize=8, fontweight='bold')
            
            ax.set_xticks(x_pos)
            ax.set_xticklabels(freq_labels, rotation=45, ha='right', fontsize=10)
            ax.set_xlabel('Frequency (Hz)', fontsize=12, fontweight='bold')
            
            # Y-axis label with weighting
            weighting_label_y = f"L{self.weighting}eq (dB{self.weighting})"
            ax.set_ylabel(weighting_label_y, fontsize=12, fontweight='bold')
            
            # Title with weighting
            weighting_label_title = f"L{self.weighting}eq"
            weighting_unit = f"dB{self.weighting}"
            ax.set_title(
                f'1/3 Octave Band {weighting_label_title} [Second #{self.second_count}] [{timestamp}]\n'
                f'{weighting_label_title}_total = {leq_total_weighted:.1f} {weighting_unit}',
                fontsize=14, fontweight='bold'
            )
            ax.set_ylim(20, 125)  # Extra space for text labels
            ax.grid(True, alpha=0.3, axis='y', linestyle='--')
            
            plt.tight_layout()
            
            # Save with unique filename
            filename = f'leq_bands_{self.weighting}weighted_second_{self.second_count}.png'
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            logger.info(f"Plot saved: {filename}")
            
            plt.show(block=False)
            plt.pause(2)
            plt.close()
            
            gc.collect()
            
        except Exception as e:
            logger.error(f"Plot error: {e}")
    
    def stop(self):
        self.running = False

# ===================== MQTT PUBLISH HELPER (COMMENTED OUT) =====================
def publish_to_mqtt():
    """
    MQTT publish function - COMMENTED OUT (focus on functionality, not IoT)
    Data is ready in leq_data_mqtt dictionary
    """
    # import paho.mqtt.client as mqtt
    # import json
    # 
    # client = mqtt.Client()
    # client.connect('localhost', 1883, 60)
    # 
    # mqtt_payload = {
    #     'leq_total_weighted': leq_data_mqtt['leq_total_weighted'],
    #     'leq_total_raw': leq_data_mqtt['leq_total_raw'],
    #     'weighting': leq_data_mqtt['weighting'],
    #     'timestamp': leq_data_mqtt['timestamp'],
    #     'lat': leq_data_mqtt['lat'],
    #     'lon': leq_data_mqtt['lon']
    # }
    # 
    # # Add frequency bands
    # for i, (freq, leq_w, leq_r) in enumerate(zip(
    #     leq_data_mqtt['freq_bands'], 
    #     leq_data_mqtt['leq_bands_weighted'],
    #     leq_data_mqtt['leq_bands_raw']
    # )):
    #     mqtt_payload[f'freq_{int(freq)}Hz_weighted'] = leq_w
    #     mqtt_payload[f'freq_{int(freq)}Hz_raw'] = leq_r
    # 
    # client.publish('noise/leq_data', json.dumps(mqtt_payload))
    # client.disconnect()
    
    pass

# ===================== MAIN =====================
def main():
    logger.info("="*70)
    logger.info("Noise Monitoring - Leq with A/C/Z Weighting")
    logger.info("="*70)
    logger.info(f"Platform: Raspberry Pi 3")
    logger.info(f"Sample Rate: {SAMPLE_RATE} Hz")
    logger.info(f"Weighting: {WEIGHTING}-weighting")
    logger.info(f"Leq Calculation: From 31 frequency bands (logarithmic sum)")
    
    if PLOT_SECONDS:
        logger.info(f"Plots will be generated at seconds: {sorted(PLOT_SECONDS)}")
    else:
        logger.info("No plots will be generated (monitoring only)")
    
    # Step 1: Pre-calculate filter bank
    filter_bank = OctaveFilterBank(SAMPLE_RATE, OCTAVE_FRACTION, OCTAVE_ORDER, FREQ_LIMITS)
    
    # Step 2: Create communication queue
    raw_queue = queue.Queue(maxsize=RAW_QUEUE_SIZE)
    
    # Step 3: Create threads
    reader = SerialReader(UART_PORT, UART_BAUD, raw_queue)
    analyzer = OctaveLeqAnalyzer(raw_queue, filter_bank, WEIGHTING, PLOT_SECONDS)
    
    # Step 4: Start threads
    reader.start()
    time.sleep(0.5)
    analyzer.start()
    
    logger.info("="*70)
    logger.info("System running. Press Ctrl+C to stop.")
    logger.info("="*70)
    print()
    
    try:
        while True:
            # infiniter loop
            time.sleep(1)
            
            # MQTT publish is commented out (focus on functionality)
            # if analyzer.second_count % 5 == 0:
            #     publish_to_mqtt()
            
            if not reader.is_alive():
                logger.error("Serial reader died!")
                break
                
    except KeyboardInterrupt:
        print("\n" + "="*70)
        logger.info("Stopping system...")
    finally:
        reader.stop()
        analyzer.stop()
        
        time.sleep(1)
        
        print("\n" + "="*70)
        logger.info("=== Final Statistics ===")
        logger.info(f"Packets received: {reader.packets_received}")
        logger.info(f"Packets corrupted: {reader.packets_corrupted}")
        if reader.packets_received > 0:
            success_rate = (reader.packets_received / 
                          (reader.packets_received + reader.packets_corrupted)) * 100
            logger.info(f"Success rate: {success_rate:.2f}%")
        logger.info(f"Leq measurements: {analyzer.second_count}")
        logger.info(f"Plots generated: {len(analyzer.plotted_seconds)}")
        logger.info("="*70)
        logger.info("System stopped cleanly.")
        
        # Print last MQTT data for verification
        logger.info("\n=== Last MQTT Data (Ready to Send) ===")
        logger.info(f"Weighting: {leq_data_mqtt['weighting']}")
        logger.info(f"L{leq_data_mqtt['weighting']}eq_total: {leq_data_mqtt['leq_total_weighted']:.1f} dB{leq_data_mqtt['weighting']}")
        logger.info(f"LZeq_total (raw): {leq_data_mqtt['leq_total_raw']:.1f} dBZ")
        logger.info(f"Location: ({leq_data_mqtt['lat']:.5f}, {leq_data_mqtt['lon']:.5f})")
        logger.info(f"Frequency bands: {len(leq_data_mqtt['leq_bands_weighted'])}")

if __name__ == "__main__":
    main()