from socket import *
import pickle
import time

clientSockets= [socket(AF_INET, SOCK_STREAM), socket(AF_INET, SOCK_STREAM), socket(AF_INET, SOCK_STREAM)]
serverNames = ["10.100.9.44", "10.100.9.45", "10.100.9.46"]



def createConnections(): #Creating connections with servers.
	
	serverPort = [12006, 12004, 12005] #Multiple ports created
	                     		   #for the 3 servers.
	for i in range(len(clientSockets)): #Looping the socket
		                            #connection.
		clientSockets[i].connect((serverNames[i],serverPort[i]))
		print "connected to host "+ serverNames[i] 


def setLimits(lLimit, uLimit): #Settting prime no. search ranges.

	lBreak = lLimit + (uLimit-lLimit)/3 #Breaking prime no search
	uBreak = lLimit + 2*(uLimit-lLimit)/3 #range into 3 sets 
	set1 = [lLimit,lBreak]
	set2 = [lBreak+1,uBreak]
	set3 = [uBreak+1,uLimit]
	
	return [set1, set2, set3]


def sendRangesToServers(limitSet): #Looping through servers and
	                           #sending the search limits 'range' to servers.
	for i in range(len(clientSockets)):
		
		print "sending range " + str(limitSet[i]) +" to host " + str(serverNames[i])
		data_numbers = pickle.dumps(limitSet[i])
		clientSockets[i].send(data_numbers)
	

def getReplies(): #Getting replies from servers.

	results = [] 
	times = []
	for i in range(len(clientSockets)):
		replyAnswerSerialized = clientSockets[i].recv(1024) #Recieving data in Pickle format

		replyAnswerDeserialized = pickle.loads(replyAnswerSerialized)#Converting from Pickle format to 'array' format
		
		results.append(replyAnswerDeserialized[0]) #Extracting results from the array
		times.append(replyAnswerDeserialized[1]) #Extracting search time
		
	return results, times
	
def closeConnections(): #Closing connections to different servers.

	for i in range(0, len(clientSockets)):
		clientSockets[i].close()






if __name__ == "__main__":#Main function

	createConnections()	#Creates three connections to servers
	
	lLimit = input("input lower limit ")
	uLimit = input("input upper limit ")
	print "\n"
	
	startTime=time.time() #Start of connection time
	
	limitSet = setLimits(lLimit, uLimit) #Breaking range into sets
	
	sendRangesToServers(limitSet) #Sending sets to servers
	
	results, times = getReplies() #Getting results and search times

	elapsedTime = time.time() - startTime
	
	totalResults = 0
	totalTime = 0
	
	for i in range(len(results)):#Displaying search results of the servers
		print "got reply = " + str(results[i]) + " from host " +str(serverNames[i])+ " after " +str(times[i])+ " seconds"
		totalResults = totalResults + results[i]
		totalTime = totalTime + times[i]

	print "answer is " + str(totalResults) #Total prime nos from each server
	print "Total time elapsed " +str(elapsedTime) #Time after each result has been returned
	print "Total time elapsed on all servers combined " + str(totalTime)#Total time taken for each server combined
	
	closeConnections() #Closing all connections to servers.
	
	

