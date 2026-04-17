import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Float32, Header
import sensor_msgs_py.point_cloud2 as pc2

import serial
import time
import struct
import math
import numpy as np
from scipy.signal import butter, lfilter

class AWR1642VitalSignsNode(Node):
    def __init__(self):
        super().__init__('awr1642_vital_signs_node')
        
        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, 'radar/points', 10)
        self.phase_pub = self.create_publisher(Float32MultiArray, 'radar/phase_data', 10)
        self.hr_pub = self.create_publisher(Float32, 'radar/heart_rate', 10)
        
        # Serial and Buffer Setup
        self.send_config()
        self.ser_data = serial.Serial('/dev/ttyUSB1', 921600, timeout=0.1)
        self.magic_word = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        self.buffer = b''
        
        # Signal Processing Variables
        self.fs = 10.0  # Sampling rate (Hz) - matches frameCfg (100ms)
        self.phase_history = []
        self.window_size = 100 # 10 seconds of data for FFT
        
        self.timer = self.create_timer(0.01, self.main_loop)
        self.get_logger().info("AWR1642 ODS Vital Signs Node Started")

    def send_config(self):
        cfg = [
            "sensorStop", "flushCfg", "dfeDataOutputMode 1", "channelCfg 15 3 0",
            "adcCfg 2 1", "adcbufCfg -1 0 0 1 1",
            "profileCfg 0 77 7 7 58.0 0 0 67.978 1 256 5020 0 0 36",
            "chirpCfg 0 0 0 0 0 0 0 1", "chirpCfg 1 1 0 0 0 0 0 2",
            "frameCfg 0 1 32 0 100 1 0", "lowPower 0 1", 
            "guiMonitor 1 1 0 1", 
            "cfarCfg 1 4 12 4 2 8 2 350 30 2 0 5 20", "dbscanCfg 4 4 13 20 3 256",
            "sensorStart"
        ]
        with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
            for line in cfg:
                ser.write((line + '\n').encode())
                time.sleep(0.1)

    def butter_bandpass_filter(self, data, lowcut, highcut, order=4):
        nyq = 0.5 * self.fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return lfilter(b, a, data)

    def process_heart_rate(self, current_phase):
        # 1. Phase Unwrapping
        if not self.phase_history:
            self.phase_history.append(current_phase)
            return
        
        diff = current_phase - self.phase_history[-1]
        unwrapped = self.phase_history[-1] + (diff + math.pi) % (2 * math.pi) - math.pi
        self.phase_history.append(unwrapped)

        # Keep window size
        if len(self.phase_history) > self.window_size:
            self.phase_history.pop(0)
        
        # 2. Need enough data for FFT (at least 5 seconds)
        if len(self.phase_history) >= 50:
            # Bandpass for Heart Rate: 0.8Hz (48 BPM) to 2.5Hz (150 BPM)
            filtered = self.butter_bandpass_filter(self.phase_history, 0.8, 2.5)
            
            # 3. FFT to find frequency
            fft_res = np.abs(np.fft.rfft(filtered))
            freqs = np.fft.rfftfreq(len(filtered), 1/self.fs)
            
            # Find peak frequency (ignore DC/Low freqs)
            max_idx = np.argmax(fft_res)
            bpm = freqs[max_idx] * 60.0
            
            hr_msg = Float32()
            hr_msg.data = float(bpm)
            self.hr_pub.publish(hr_msg)

    def main_loop(self):
        if self.ser_data.in_waiting > 0:
            self.buffer += self.ser_data.read(self.ser_data.in_waiting)
            
            if self.magic_word in self.buffer:
                idx = self.buffer.index(self.magic_word)
                if len(self.buffer[idx:]) < 40: return
                
                header = self.buffer[idx:idx+40]
                total_len = struct.unpack('<I', header[12:16])[0]
                num_tlvs = struct.unpack('<I', header[32:36])[0]
                
                if len(self.buffer[idx:]) < total_len: return
                
                payload = self.buffer[idx+40 : idx+total_len]
                tlv_ptr = 0

                for _ in range(num_tlvs):
                    t_type, t_len = struct.unpack('<II', payload[tlv_ptr : tlv_ptr+8])
                    t_data = payload[tlv_ptr+8 : tlv_ptr+8+t_len]
                    
                    if t_type == 1: # Points
                        num_obj = struct.unpack('<H', t_data[0:2])[0]
                        q_val = struct.unpack('<H', t_data[2:4])[0]
                        # ... point parsing logic from previous script ...
                        
                    elif t_type == 4: # Heatmap
                        num_samples = len(t_data) // 4
                        phases = []
                        amplitudes = []
                        for i in range(num_samples):
                            imag, real = struct.unpack('<hh', t_data[i*4 : i*4+4])
                            phases.append(math.atan2(imag, real))
                            amplitudes.append(math.sqrt(real**2 + imag**2))
                        
                        # Find the strongest reflection (the person)
                        if amplitudes:
                            peak_idx = np.argmax(amplitudes)
                            self.process_heart_rate(phases[peak_idx])
                            
                            p_msg = Float32MultiArray()
                            p_msg.data = [float(p) for p in phases]
                            self.phase_pub.publish(p_msg)

                    tlv_ptr += (8 + t_len)
                self.buffer = self.buffer[idx+total_len:]

def main(args=None):
    rclpy.init(args=args)
    node = AWR1642VitalSignsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser_data.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
