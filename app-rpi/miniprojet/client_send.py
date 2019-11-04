import socket
import sys

HOST, PORT = "localhost", 10000
data = " ".join(sys.argv[1:])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.sendto(data, (HOST, PORT))

print("Sent:     {}".format(data))