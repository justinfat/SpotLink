import cv2
import socket
import numpy as np
import struct
import threading

stop_sockets = False

class RecvController:
    def recv_video(self, connection_socket):
        global stop_sockets
        data = b'' # empty btytes
        payload_size = struct.calcsize("L")
        
        while not stop_sockets:
            try:
                # process the data that received
                while len(data) < payload_size:
                    data += connection_socket.recv(4096)
                
                #if not data:
                #    print("Connection closed by client.")
                #    break

                packed_data_size = data[:payload_size]
                data = data[payload_size:]
                data_size = struct.unpack("L", packed_data_size)[0]

                while len(data) < data_size:
                    data += connection_socket.recv(4096)

                frame_data = data[:data_size]
                data = data[data_size:]

                # show the video
                frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(240, 320, 3)
                cv2.imshow('Received Video', frame)

                # stop video calling if type q
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_sockets = True
                    break
            except socket.error as e:
                print("Receive video socket error:", e)
                stop_sockets = True
                break

        cv2.destroyAllWindows()