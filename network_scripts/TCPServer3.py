from socket import *
import pickle
from primeCalc import *
import time

serverPort = 12005
serverSocket = socket(AF_INET, SOCK_STREAM)
serverSocket.bind(("",serverPort))
serverSocket.listen(1)
print "The server is ready to receive"
while 1:
	connectionSocket, addr = serverSocket.accept()
	pickledLimits = connectionSocket.recv(1024)
	limits = pickle.loads(pickledLimits)

	num , t = countPrimes(limits[0],limits[1])
	
	toSend = [num, t]
	connectionSocket.send(pickle.dumps(toSend))
	connectionSocket.close()
