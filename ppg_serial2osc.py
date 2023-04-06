import serial
import serial.tools.list_ports
import time
import numpy as np
from pythonosc import udp_client
from threading import Thread
import keyboard
import matplotlib.pyplot as plt
import neurokit2 as nk
import subprocess


import operator

# Constants
BUFFER_SIZE = 15000
OVERLAP = 1000
#FS = 50  # Sampling rate of the PPG signal in Hz
OSC_ADDRESS = "/hrv"
OSC_IP = "<your computer's IP address>"
OSC_PORT = 4560


        
def process_PPG(data, FS):
    '''
    Process the PPG signal that will be sent as OSC messages.

    This function computes the heart rate and heart rate variability (HRV)
    indices from the PPG signal.

    Parameters
    ----------
    data : list
        The PPG signal.
    FS : int
        The sampling rate of the PPG signal in Hz.
    
    Returns
    -------
    hrv_indices : pd.DataFrame
        A dictionary containing the HRV indices.
    hrv_time_ind : pd.DataFrame
        A dictionary containing the HRV time domain indices.
    '''
    data = nk.ppg_clean(np.array(data), sampling_rate=FS)
    peaks = nk.ppg_findpeaks(data, sampling_rate=FS, show=False)
    print('PEAKS', peaks)
    hrv_indices = nk.hrv_frequency(peaks, sampling_rate=FS, show=False)
    hrv_time_ind = nk.hrv_time(peaks['PPG_Peaks'][-100:], sampling_rate=FS, show=False)
    print(hrv_indices)
    print(hrv_time_ind)
    return hrv_indices, hrv_time_ind

def send_osc_message(value, address):
    '''
    Send an OSC message.

    Parameters
    ----------
    value : int or float or list
        The value to send.
    address : str
        The OSC address to send the value to.
    '''
    # Send the value as an OSC message
    client = udp_client.SimpleUDPClient(OSC_IP, OSC_PORT)
    client.send_message(address, value)


def detect_serial_port():
    '''
    Detect the serial port to which the Arduino is connected.
    
    Returns
    -------
    str or None
    '''
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description:
            return port.device
    return None


def read_serial_data():
    '''
    Read the serial data from the Arduino and send it as OSC messages.

    The Arduino should send a packet of 2 bytes every time a PPG sample is
    available. The first byte is the most significant byte of the 16-bit
    integer value, and the second byte is the least significant byte.

    The PPG signal is processed in chunks of BUFFER_SIZE samples, and the
    processing is overlapped by OVERLAP samples. The processing is done
    every time BUFFER_SIZE samples are received.

    The sampling rate of the PPG signal is computed from the time between
    two consecutive packets. The sampling rate is then sent as an OSC
    message.

    The PPG signal's extracted variables of interest are sent as an OSC message.

    The OSC messages are sent to the IP address and port specified by the
    OSC_IP and OSC_PORT constants.

    The serial port is detected automatically by looking for a port with
    "Arduino" in its description.

    The serial port is opened at 115200 bauds, and the timeout is set to 0.001
    second.

    The serial port is closed when the user presses the 'x' key.
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
            current_time = time.time()
            time_diff = current_time - last_received_packet_time
            last_received_packet_time = current_time
            value = packet[0] << 8 | packet[1]
            
            buffer.append(value)
            buffer_index = len(buffer)
            time_diff_list.append(time_diff)
            mode = stats.mode(np.array(time_diff_list))
            
            FS = 1 / mode[0][0]
            FS = np.round(FS)
            buffer_index = len(buffer)
            if len(time_diff_list) > 50000:
                time_diff_list = time_diff_list[20000:50000]
            
            ib = 0
            if buffer_index == BUFFER_SIZE:
                    buffer = [float(x) for x in buffer]
                try:
                    HRV, HRV_time = process_PPG(buffer, 320)
                    HRV_HF = HRV['HRV_HF']
                    HRV_VHF = HRV['HRV_VHF']
                    LF_HF = HRV['HRV_LFHF']
                    rate = (1000/HRV_time['HRV_MeanNN'])*60
                    print('HRV', HRV_VHF)
                    print('RaTE', rate)

                    # Send the PPG value as an OSC message
                    send_osc_message(HRV_HF, '/hrv/hf')
                    send_osc_message(HRV_VHF, '/hrv/vhf')
                    send_osc_message(LF_HF, '/hrv/lfhf')
                    send_osc_message(rate, '/rate')
                    send_osc_message(HRV_time['HRV_SDNN'], '/hrv/sd')
                except:
                    print('not enough peaks detected')
                    pass

                # Clear the first n samples of the buffer
                ib+=1
                buffer = buffer[OVERLAP:]
                FS_buffer = FS_buffer[OVERLAP:]
                #buffer = list(buffer)
                #print('NEW_BUFFER_SIZE', len(buffer))
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
    # example command line: python3 ppg_serial2osc.py
    thread = Thread(target=read_serial_data)
    thread.start()

