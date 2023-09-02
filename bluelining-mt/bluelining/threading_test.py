import threading
from stream_handler_krypton import stream_handler
import numpy as np




def main():
    host='192.168.18.89' #ip of krypton Computer (server)
    port=12000 #same as server
    handler = stream_handler(host,port)
    handler_thread = threading.Thread(target=handler.handle, daemon=True)
    handler_thread.start()
    while(True):
        coords = np.around(handler.get_filtered_coords(), 3)
        
        print(coords, end='\r')

if __name__ == "__main__":
    if not main():
        os._exit(1)
    os._exit(0)