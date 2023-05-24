import cv2
import socket
# import numpy as np
import struct

sever_ip = '0.0.0.0'
sever_port = 8485

class SendController:
    def send_video(self, connection_socket):
        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FPS, 10)

        # Load the Haar cascade for face detection
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        while True:
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
                    break

            except socket.error as e:
                print("Send video socket error:", e)
                break

        capture.release()

if __name__ == '__main__':
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # client socket declaration: ipv4, TCP
    server_socket.bind((sever_ip, sever_port))
    server_socket.listen(1)

    connection_socket, client_address = server_socket.accept()

    SendController().recv_video(connection_socket)

    # connection_socket.shutdown(socket.SHUT_RDWR)
    connection_socket.close()
    server_socket.close()