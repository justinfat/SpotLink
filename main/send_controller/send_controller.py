import cv2
import socket
import numpy as np
import struct
import threading

stop_sockets = False

class SendController:
    def send_video(self, connection_socket):
        global stop_sockets
        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FPS, 10)

        while not stop_sockets:
            try:
                ret, frame = capture.read()
                if not ret:
                    print('unable to read the video...')
                    break

                frame = cv2.resize(frame, (320, 240))
                data = frame.tobytes()

                connection_socket.sendall(struct.pack("L", len(data)))
                connection_socket.sendall(data)

                # stop video calling if type q
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_sockets = True
                    break
            except socket.error as e:
                print("Receive video socket error:", e)
                stop_sockets = True
                break

        capture.release()