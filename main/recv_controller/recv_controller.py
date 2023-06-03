import cv2
import socket
import numpy as np
import struct
from tflite_runtime.interpreter import Interpreter
from flask import Flask, Response
from flask_cors import CORS
import threading

sever_ip = '0.0.0.0'
sever_port = 8485

global_buffer = None
app = Flask(__name__)
CORS(app)


def generate_frames():
    global global_buffer

    while True:
        if global_buffer is not None:
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + global_buffer + b'\r\n\r\n')
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


class RecvController:
    def __init__(self, communication_queues):
        self._motion_queue = communication_queues['motion_controller']
        self._socket_queue = communication_queues['socket_queue']
        # app.run(host='0.0.0.0', port=8586)

    def run(self, communication_queues):
        controller = RecvController(communication_queues)
        recv_video_thread = threading.Thread(target=controller.recv_video, args=())
        recv_video_thread.start()
        app.run(host='0.0.0.0', port=8586)
        recv_video_thread.join()

    def recv_video(self):
        global global_buffer
        connection_socket = self._socket_queue.get(block=True)
        data = b'' # empty btytes
        payload_size = struct.calcsize("L")

        # # Load the TFLite model and allocate tensors.
        # interpreter = Interpreter(model_path="/home/pi/MayTest/main/recv_controller/model_mobilenet.tflite")
        # interpreter.allocate_tensors()

        # # Get input and output tensor details
        # input_details = interpreter.get_input_details()
        # output_details = interpreter.get_output_details()

        # # Load the Haar cascade for face detection
        # face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # # Define the emotion labels
        # emotions = ('angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral')
        
        while True:
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
                frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(240, 320, 3)

                # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
                # for (x, y, w, h) in faces:
                #     cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

                #     face = gray[y:y+h, x:x+w] # choose the face region from gray
                #     face = cv2.resize(face, (224, 224)).reshape(224, 224, 1)
                #     #face = np.array(Image.fromarray(face))
                #     face = face.astype('float32') / 255.0
                #     face = np.expand_dims(face, axis=0)

                #     interpreter.set_tensor(input_details[0]['index'], face)
                #     interpreter.invoke()

                #     predictions = interpreter.get_tensor(output_details[0]['index'])
                #     max_index = np.argmax(predictions[0])
                #     emotion = emotions[max_index]

                #     cv2.putText(frame, emotion, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # cv2.imshow('Received Video', frame)
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    print('unable to encode the video...')
                    break
                global_buffer = jpeg.tobytes()
                # if global_buffer is not None:
                #     print(len(global_buffer))
                # else:
                #     print('global_buffer is None.')

                # stop video calling if type q
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    # connection_socket.shutdown(socket.SHUT_RDWR)
                    connection_socket.close()
                    break
                
            except socket.error as e:
                print("Receive video socket error:", e)
                # connection_socket.shutdown(socket.SHUT_RDWR)
                connection_socket.close()
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # client socket declaration: ipv4, TCP
    server_socket.bind((sever_ip, sever_port))
    server_socket.listen(1)

    connection_socket, client_address = server_socket.accept()

    RecvController().recv_video(connection_socket)

    # connection_socket.shutdown(socket.SHUT_RDWR)
    connection_socket.close()
    server_socket.close()