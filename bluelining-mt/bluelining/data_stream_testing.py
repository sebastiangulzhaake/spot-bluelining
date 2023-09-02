# FOR TESTING THE DATA STREAM.
# SOME PRINT STATEMENTS HAVE TO BE UNCOMMENTED IN THE DATASTREAMTHREAD/MONITOR TO SEE IF IT WORKS

import threading
import os
from data_stream_monitor import DataStreamMonitor
from data_stream_thread import DataStreamThread

def main():
     # Create data stream monitor and start data stream thread
    # Comment out the lines for the tracker that is not used

    # host='192.168.78.31' #IP = my IP at maxiv_visitor wifi
    # port=65431 #same as server
    # tracker = 'leica'
    host='192.168.18.89' #ip of krypton Computer (server)
    port=12000 #same as server
    tracker = 'krypton'
    data_stream_monitor = DataStreamMonitor()
    data_stream_thread = DataStreamThread(data_stream_monitor, host, port, tracker)
    data_stream_thread.start()
    while(True):
        a = 1

if __name__ == "__main__":
    if not main():
        os._exit(1)
    os._exit(0)