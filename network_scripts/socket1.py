import socket
import sys

host = ''
port = 555
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    s.bind((host, port))
except:
    print(str(e))
s.listen(5)

def threaded_client(conn):
    conn.send(str.encode('Welcome, type your shit: '))
    while True:
        data = conn.recv(2048)
        reply = 'Server out :'
        if not data:
            break
            
# conn, addr = s.accept()
# print ('Connected to: ' + addr[0] + ':'  + str(addr[0]))
