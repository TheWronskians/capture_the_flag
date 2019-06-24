from socket import *
import pickle #Pickle used for data transfering.
from primeCalc import *
import time

#Set server socket settings.
serverPort = 12007
serverSocket = socket(AF_INET, SOCK_STREAM)
serverSocket.bind(("",serverPort))
serverSocket.listen(1)
print("The server is ready to receive")
while 1:
	connectionSocket, addr = serverSocket.accept() #connecting to client
	pickledLimits = connectionSocket.recv(1024)#Recieve data in Pickle form.
	limits = pickle.loads(pickledLimits)#Convert data to array

	num , t = countPrimes(limits[0],limits[1])
	
	toSend = [num, t] #Preparing data to be sent
	connectionSocket.send(pickle.dumps(toSend)) #Converting to Pickle form and sending
	connectionSocket.close()#Closing connections.
