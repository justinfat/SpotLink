import cv2
import socket
import numpy as np
import struct
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
# import threading

stop_sockets = False

class RecvController:
    def recv_video(self, connection_socket):
        global stop_sockets
        data = b'' # empty btytes
        payload_size = struct.calcsize("L")

        # Load the TFLite model and allocate tensors.
        interpreter = Interpreter(model_path="/home/pi/MayTest/main/recv_controller/model_mobilenet.tflite")
        interpreter.allocate_tensors()

        # Get input and output tensor details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Load the Haar cascade for face detection
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Define the emotion labels
        emotions = ('angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral')
        
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
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

                    face = gray[y:y+h, x:x+w]
                    face = cv2.resize(face, (224, 224)).reshape(224, 224, 1)
                    #face = np.array(Image.fromarray(face))
                    face = face.astype('float32') / 255.0
                    face = np.expand_dims(face, axis=0)

                    interpreter.set_tensor(input_details[0]['index'], face)
                    interpreter.invoke()

                    predictions = interpreter.get_tensor(output_details[0]['index'])
                    max_index = np.argmax(predictions[0])
                    emotion = emotions[max_index]

                    cv2.putText(frame, emotion, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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