from socket import *
import pickle
import time

clientSockets= socket(AF_INET, SOCK_STREAM)
serverNames = "10.100.9.44"



def createConnections(): #Creating connections with servers.
	
	serverPort = 12006 # ports created
	                     		  
	clientSockets.connect((serverNames,serverPort))
	print "connected to host "+ serverNames


def sendToServer(limitSet): #Looping through servers and
	                           #sending the search limits 'range' to servers.
		
	print "sending range " + str(limitSet) +" to host " + str(serverNames)
	data_numbers = pickle.dumps(limitSet)
	clientSockets.send(data_numbers)
	

def getReplies(): #Getting replies from servers.

	replyAnswerSerialized = clientSockets.recv(1024) #Recieving data in Pickle format

	replyAnswerDeserialized = pickle.loads(replyAnswerSerialized)#Converting from Pickle format to 'array' format
		
	results=(replyAnswerDeserialized[0]) #Extracting results from the array
	times=(replyAnswerDeserialized[1]) #Extracting search time
		
	return results, times
	
def closeConnections(): #Closing connections to different servers.

	clientSockets.close()






if __name__ == "__main__":#Main function

	createConnections()	#Creates three connections to servers
	
	lLimit = input("input lower limit ")
	uLimit = input("input upper limit ")
	print "\n"
	
	startTime=time.time() #Start of connection time
	
	limitSet = [lLimit,uLimit] #Breaking range into sets
	
	sendToServer(limitSet) #Sending sets to servers
	
	results, times = getReplies() #Getting results and search times

	elapsedTime = time.time() - startTime
	
	totalResults = 0
	totalTime = 0
	
	print "got reply = " + str(results) + " from host " +str(serverNames)+ " after " +str(times)+ " seconds"
	totalResults = totalResults + results
	totalTime = totalTime + times

	print "answer is " + str(totalResults) #Total prime nos from each server
	print "Total time elapsed " +str(elapsedTime) #Time after each result has been returned
	print "Total time elapsed on all servers combined " + str(totalTime)#Total time taken for each server combined
	
	closeConnections() #Closing all connections to servers.
	
	

