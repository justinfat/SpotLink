import socket
import os

# The address of the UNIX domain socket
server_address = '/home/pi/uds_socket'

# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

# Bind the socket to the address
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    connection, client_address = sock.accept()
    try:
        print('connection from', client_address)

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(1024).decode()
            if data:
                print(data)
                # print('received {!r}'.format(data))
                connection.sendall(b'Hello from Python')
            else:
                print('no more data from', client_address)
                break

    finally:
        # Clean up the connection
        connection.close()
