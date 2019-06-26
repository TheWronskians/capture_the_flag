from socket import *
import pickle
import time

clientSockets= socket(AF_INET, SOCK_STREAM)
serverNames = "10.199.61.21"



def createConnections(): #Creating connections with servers.

	serverPort = 12007 # ports created

	clientSockets.connect((serverNames,serverPort))
	print ("connected to host "+ serverNames)


def sendToServer(limitSet): #Looping through servers and
	                           #sending the search limits 'range' to servers.

	print ("sending range " + str(limitSet) +" to host " + str(serverNames))
	data_numbers = pickle.dumps(limitSet)
	clientSockets.send(data_numbers)

def sendToServerString(string): #Looping through servers and
	                           #sending the search limits 'range' to servers.

	print("Sending string: " + str(string))
	data_numbers = pickle.dumps(string)
	clientSockets.send(data_numbers)


def getReplies(): #Getting replies from servers.

	replyAnswerSerialized = clientSockets.recv(1024) #Recieving data in Pickle format

	replyAnswerDeserialized = pickle.loads(replyAnswerSerialized)#Converting from Pickle format to 'array' format

	results=(replyAnswerDeserialized[0]) #Extracting results from the array
	times=(replyAnswerDeserialized[1]) #Extracting search time

	return results, times

def closeConnections(): #Closing connections to different servers.

	clientSockets.close()
'''
if __name__ == "__main__":#Main function

	createConnections()	#Creates three connections to servers
	for i in range(10):
		#createConnections()	#Creates three connections to servers
		# lLimit = input("input lower limit ")
		# uLimit = input("input upper limit ")
		x = input("x ")
		y = input("y ")
		z = input("z ")
		ax = input("ax ")
		ay = input("ay ")
		az = input("az ")
		#string = input("Enter the string you wish to send\n")
		print ("\n")
		limitSet = []
		startTime=time.time() #Start of connection time

		# limitSet = [lLimit,uLimit] #Breaking range into sets
		limitSet = [x, y, z, ax, ay, az]
		sendToServer(limitSet) #Sending sets to servers

		results, times = getReplies() #Getting results and search times
		#replyAnswerSerialized = clientSockets.recv(1024)
		#replyAnswerDeserialized = pickle.loads(replyAnswerSerialized)
		#print(replyAnswerDeserialized)
		elapsedTime = time.time() - startTime

		totalResults = 0
		totalTime = 0

		print ("got reply = " + str(results) + " from host " +str(serverNames)+ " after " +str(times)+ " seconds")
		totalResults = totalResults + results
		totalTime = totalTime + times
	closeConnections() #Closing all connections to servers.
'''
