import serial
import serial.tools.list_ports
import time
import numpy as np
from pythonosc import udp_client
from threading import Thread
import keyboard
from scipy import stats
from biotuner.biotuner_object import *
from biotuner.scale_construction import harmonic_tuning
from biotuner.rhythm_construction import consonant_euclid, euclid_long_to_short

# Constants
BUFFER_SIZE = 4000
OVERLAP = 1000
#FS = 50  # Sampling rate of the PPG signal in Hz
OSC_ADDRESS = "/bt"
OSC_IP = "<your IP address>"
OSC_PORT = 4560
    

def send_osc_message(value, address):
    # Send the value as an OSC message
    client = udp_client.SimpleUDPClient(OSC_IP, OSC_PORT)
    if isinstance(value, list):
        client.send_message(address, *value)
    else:
        client.send_message(address, value)



def detect_serial_port():
    ports = serial.tools.list_ports.comports()
    print(ports)
    for port in ports:
        if "Arduino" in port.description:
            return port.device
    return None

def bt_realtime(data, fs):
    '''
    Compute the biotuner metrics from the data.

    Parameters
    ----------
    data : list
        The data to be processed.
    fs : int
        The sampling rate of the data in Hz.
    Returns
    -------
    peaks : list
        A list containing the peaks.
    metrics : list
        A dictionary containing the biotuner metrics.
    '''
    # Compute the biotuner metrics
    # peaks_function can be 'harmonic_recurrence', 'emd', 'wavelet', 'spectral' 
    bt_plant = compute_biotuner(peaks_function='harmonic_recurrence', sf=fs)
    
    # extract peaks
    bt_plant.peaks_extraction(np.array(data), graph=False, min_freq=0.1, max_freq=65, precision=0.1, nIMFs=5, n_peaks=5, smooth_fft=4)
    #bt_plant.peaks_extension(method='harmonic_fit')
    bt_plant.peaks_extension(method='harmonic_fit', n_harm=50)
    # compute metrics from peaks
    bt_plant.compute_peaks_metrics(n_harm=3, delta_lim=100)
    #bt_plant.compute_diss_curve(plot=True, input_type='peaks')
    
    harm_tuning = harmonic_tuning(bt_plant.all_harmonics)
    euclid, cons_euclid = consonant_euclid(harm_tuning, n_steps_down = 2, limit_denom = 16, 
                                      limit_cons = 0.1, limit_denom_final = 16)
    euclid_final = [euclid_long_to_short(x) for x in euclid]
    # compute metrics of spectral curve
    bt_plant.compute_spectromorph(comp_chords=True, graph=False)

    return bt_plant.peaks, bt_plant.extended_peaks, bt_plant.peaks_metrics, euclid_final


def read_serial_data():
    '''
    Read the serial data from the Arduino and send it as OSC messages.

    The Arduino should be sending 2-byte packets of unsigned integers
    representing the electrovalue. The Arduino code is in the arduino folder.

    The function reads the data from the serial port and computes 
    biotuner metrics to send as OSC messages.
    '''
    # Open the serial port
    ser = serial.Serial(detect_serial_port(), 115200, timeout=1)
    if not ser.isOpen():
        ser.open()
    time.sleep(2)

    #buffer = np.zeros(BUFFER_SIZE)
    buffer = []
    buffer_index = 0

    time_diff_list = []
    last_received_packet_time = time.time()
    while True:
        # Wait until a packet is received
        # Read the packet
        packet = bytearray()
        while True:
            next_byte = ser.read()
            if next_byte == b'\xC0':
                break
            elif next_byte == b'\xDB':
                next_byte = ser.read()  
                if next_byte == b'\xDC':
                    packet.append(0xC0)
                elif next_byte == b'\xDD':
                    packet.append(0xDB)
            else:
                packet.append(next_byte[0])
        
        # Extract the integer value from the packet
        if len(packet) == 2:
            # Add the value to the buffer
            value = packet[0] << 8 | packet[1]
            buffer.append(value)

            # take the mode of time_diff_list
            current_time = time.time()
            time_diff = current_time - last_received_packet_time
            last_received_packet_time = current_time
            time_diff_list.append(time_diff)
            mode = stats.mode(np.array(time_diff_list))
            
            # compute the sampling rate from the mode 
            # of the time difference between packets
            FS = 1 / mode[0][0]
            FS = np.round(FS)
            print(FS)
            print('BUFFER LENGTH', len(buffer))
            buffer_index = len(buffer)
            if len(time_diff_list) > 50000:
                time_diff_list = time_diff_list[20000:50000]
            # Compute the biotuner metrics and send them as OSC messages
            # when buffer is full
            ib = 0
            if buffer_index == BUFFER_SIZE:
                try:
                    buffer = [float(x) for x in buffer]
                    peaks, extended_peaks, metrics, euclid = bt_realtime(buffer, Fs=FS)
                    print('PEAKS', peaks)

                    # Send the PPG value as an OSC message
                    send_osc_message(metrics['harmsim'], '/bt/metric/harmsim')
                    send_osc_message(metrics['cons'], '/bt/metric/cons')
                    send_osc_message(metrics['tenney'], '/bt/metric/tenney')
                    send_osc_message(metrics['subharm_tension'], '/bt/metric/subharm')
                    send_osc_message(peaks, '/bt/peaks')
                    send_osc_message(extended_peaks, '/bt/extended_peaks')
                    send_osc_message(euclid[0], '/bt/euclid/1')
                    send_osc_message(euclid[1], '/bt/euclid/2')
                    send_osc_message(euclid[2], '/bt/euclid/3')
                    send_osc_message(euclid[3], '/bt/euclid/4')
                    
                except:
                    pass
                # Reset the buffer
                ib += 1
                buffer = buffer[OVERLAP:]
                buffer_index = BUFFER_SIZE - OVERLAP

            if buffer_index > BUFFER_SIZE:
                # This should never happen, but just in case
                print("Buffer overflow!")
                buffer = np.zeros(BUFFER_SIZE)
                buffer_index = 0

        else:
            # Serial read timeout
            time.sleep(0.001)
        
        if keyboard.is_pressed('x'):
            print('Space bar pressed. Exiting...')
            break



if __name__ == '__main__':
    # Start the thread that reads the serial data
    # command line example : python plants_serial2osc.py
    thread = Thread(target=read_serial_data)
    thread.start()
