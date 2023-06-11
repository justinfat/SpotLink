import cv2
import socket
import numpy as np
import struct
import threading

sever_ip = '0.0.0.0'
sever_port = 8485
videoHeight = 240
videoWidth = 320
face_in_center = True

class SendController:
    def __init__(self, communication_queues):
        self._motion_queue = communication_queues['motion_controller']
        self._socket_queue = communication_queues['socket_queue']
    
    def run(self, communication_queues):
        controller = SendController(communication_queues)
        send_video_thread = threading.Thread(target=controller.send_video, arg=())
        face_track_thread = threading.Thread(target=controller.face_track, arg=())
        send_video_thread.start()
        face_track_thread.start()
        send_video_thread.join()
        face_track_thread.join()

    def send_video(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # client socket declaration: ipv4, TCP
        server_socket.bind((sever_ip, sever_port))
        server_socket.listen(1)

        connection_socket, client_address = server_socket.accept()
        self._socket_queue.put(connection_socket)

        while True:
            try:
                connection_socket.sendall(struct.pack("L", len(self.data)))
                connection_socket.sendall(self.data)

            except socket.error as e:
                print("Send video socket error:", e)
                # connection_socket.shutdown(socket.SHUT_RDWR)
                connection_socket.close()
                break


        # return connection_socket

    def face_track(self):
        # connection_socket = self.create_sever_socket()
        # self._socket_queue.put(connection_socket)

        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FPS, 10)

        # Load the Haar cascade for face detection
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        while True:
            ret, frame = capture.read()
            if not ret:
                print('unable to read the video...')
                break

            frame = cv2.resize(frame, (320, 240))
            frame = np.frombuffer(frame, dtype=np.uint8).reshape(videoHeight, videoWidth, 3)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

            for (x, y, w, h) in faces:
                faceCenter = (int(x+w/2), int(y+h/2))
                # cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2) # face region
                cv2.putText(frame, 'x: %s, y: %s'%faceCenter, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1) # face center coordinates
                cv2.line(frame, faceCenter, (int(videoWidth/2), int(videoHeight/2)), (0, 0, 255), 2) # face center to frame center line
                cv2.rectangle(frame, (int(videoWidth*0.25), int(videoHeight*0.25)), (int(videoWidth*0.75), int(videoHeight*0.75)), (255, 0, 0), 2)
                # face = gray[y:y+h, x:x+w] # choose the face region from gray

                if faceCenter[0] > videoWidth*0.75:
                    self._motion_queue.put('TooRight', timeout=60)
                    # print('Too right...')
                elif faceCenter[0] < videoWidth*0.25:
                    self._motion_queue.put('TooLeft', timeout=60)
                    # print('Too left...')

                if faceCenter[1] > videoHeight*0.75:
                    self._motion_queue.put('TooLow', timeout=60)
                    # print('Too low...')
                elif faceCenter[1] < videoHeight*0.25:
                    self._motion_queue.put('TooHigh', timeout=60)
                    # print('Too high...')

            self.data = frame.tobytes()

        capture.release()

if __name__ == '__main__':
    SendController().send_video()