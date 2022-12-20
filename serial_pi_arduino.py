#!/usr/bin/env python3
import serial
import pickle

if __name__ == '__main__':

    # Initialize the pickle file 
    with open('pi_input/arduino-input.pickle', 'wb') as handle:
        pickle.dump(0, handle, protocol=pickle.HIGHEST_PROTOCOL)

    # Connecting to the Arduino Serial Port
    connected = False
    try:
        print("Trying connection to port0")
        ser=serial.Serial('/dev/ttyACM0',9600)
        print("Connection to 0 successful!")
        connected = True
    except:
        print("Failed port 0")

    if connected == False:
        print("Connections failed")

    ser.reset_input_buffer()

    # Constantly checking if a new Serial Message is being posted and writes it into arduino-input.pickle
    while(True):
        if ser.in_waiting > 0:
            line = int(ser.readline().decode('utf-8').rstrip())
            print(line)
            print(type(int(line)))

            # Writing to the pickle file 
            with open('pi_input/arduino-input.pickle', 'wb') as handle:
                pickle.dump(line, handle, protocol=pickle.HIGHEST_PROTOCOL)

            # reading
            try:
                with open('pi_input/arduino-input.pickle', 'rb') as handle:
                    a = pickle.load(handle)
            except:
                print("file cannot be opened")

            print("Read: ", a)        
