"""
@Filename:  DGRecv.py
@Author:    Jiangfan Li
@Email:     rdemezerl@gmail.com
@Description: DataGlove Socket Receiever
    @ verbose: print data receieved
    @ show:	show mesh using MANO
@Note:
	Please change addr in esp32 codes.
@TODO:
    Receieve data from 2 gloves
"""
import socket
import numpy as np
import threading
import struct

FRAME_LENGTH = 132 # 16*4*2 + 4

class DGRecv:
    def __init__(self, port=9000, verbose=False, show = False):
        self.port = port
        self.s = socket.socket()
        self.t = threading.Thread(target=self.run, name="listen")
        self.t.daemon=True

        self.buffer = []
        self.data = [] #json
        self.times = []
        # save data periodically? ~90MB/h

        self.verbose = verbose
        self.show = show
        if show:
            self.mViewer = MANO_Viewer.MANO_Viewer()


    def decode(self, msg):
        # time
        timestamp = msg[0] + msg[1] * (1<<16)
        # data
        data = np.zeros((16, 4))
        for i in range(16):
            for j in range(4):
                data[i, j] = msg[2+i*4+j]*(1.0/(1<<12))
                # we will store '2-byte float' as float(4bytes)...
                # for Euler: /900.0 in Rad
        self.data.append(data)
        self.times.append(timestamp)
        
        if self.verbose:
            print timestamp
            print data
        if self.show:
            self.update_show(data)
        # fps
        if len(self.times) > 1:
            dt = self.times[-1] - self.times[-2]
            if dt < 1:
                fps = ">1000"
            else:
                fps = str(1000.0 / dt)
            sys.stdout.flush()
            print("\r["+str(self.times[-1])+"]:"+fps+" fps")

    def save(self, filename = 'test'):
        np.save(filename, self.data)
        with open(filename+".txt","w") as f: # 'a' to add
            for t in self.times:
                f.write(str(t))
                f.write("\n")         

    def handle(self, msg):
        self.buffer += msg
        if len(self.buffer) >= FRAME_LENGTH:
            frame_data = self.buffer[:FRAME_LENGTH]
            self.buffer = self.buffer[FRAME_LENGTH:]
            content = struct.unpack("<"+"h"*(FRAME_LENGTH/2), frame_data)
            self.decode(content)
            # client.send("OK.".encode("ascii"))


    def start(self):
        # localhost IP
        def get_host_ip():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('8.8.8.8', 80))
                ip = s.getsockname()[0]
            finally:
                s.close()

            return ip
        print("starting server at " + get_host_ip() + ":" + str(self.port))

        self.s.bind(('0.0.0.0', self.port))
        self.s.listen(2)        
        # reset buffer 
        self.buffer = ""
        # listenning
        self.t.start()

    def run(self):
        client, addr = self.s.accept()
        print("Accept Connection from "+ client.getsockname()[0])
        # client.settimeout(5)
        while True:
            # client.settimeout()
            content = client.recv(256)
            #content = list(content) # for python3
            if len(content) == 0:
                break # client close
            else:
                self.handle(content)
        client.close()
        print("Connection Closed")

    def update_show(self, quats):        
        p = MANO_Viewer.styleMANO(quats)
        self.mViewer.view(p)

if __name__ == "__main__":
    import time
    import MANO_Viewer
    import sys
    dgr = DGRecv(port=9090, verbose=True, show=False)
    dgr.start()  
    try: 
        while True:
            time.sleep(1)
        
    # auto save
    except KeyboardInterrupt:
        dgr.save()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)