import cv2
import socket
import numpy as np
import struct
import threading

sever_ip = '0.0.0.0'
sever_port = 8485
stop_threads = False

class VideocallController:

    def send_video(connection_socket):
        global stop_threads
        capture = cv2.VideoCapture(0)

        while not stop_threads:
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
                    stop_threads = True
                    break
            except socket.error as e:
                print("Receive video socket error:", e)
                stop_threads = True
                break

        capture.release()

    def recv_video(connection_socket):
        global stop_threads
        data = b'' # empty btytes
        payload_size = struct.calcsize("L")
        
        while not stop_threads:
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
                    stop_threads = True
                    break
            except socket.error as e:
                print("Receive video socket error:", e)
                stop_threads = True
                break

        cv2.destroyAllWindows()

    if __name__ == '__main__':
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # client socket declaration: ipv4, TCP
        server_socket.bind((sever_ip, sever_port))
        server_socket.listen(1)

        connection_socket, client_address = server_socket.accept()

        send_video_thread = threading.Thread(target=send_video, args=(connection_socket,))
        recv_video_thread = threading.Thread(target=recv_video, args=(connection_socket,))

        send_video_thread.start()
        recv_video_thread.start()

        send_video_thread.join()
        recv_video_thread.join()

        # connection_socket.shutdown(socket.SHUT_RDWR)
        connection_socket.close()
        server_socket.close()
