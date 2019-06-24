from math import *
import time

def checkingPrime(number): #Recieves number, true if prime. Algorithem reference mentioned in readme file.
	checkValue = int(sqrt(number))+1
	printBool = True
	#print checkValue
	for i in range(2, checkValue):
		if (number%i) == 0:
			printBool = False
			break
	return printBool
		
def countPrimes(lLimit, uLimit): #Recieves range limit and returns amount of primes and total search time.
	start_time = time.time()
	count = 0
	temp=lLimit
	if lLimit<=1: #Since 1 cannot be considered a prime.
		lLimit=2
	if (uLimit<lLimit) or (uLimit<=1):
		return 0,0
	for number in range(lLimit,uLimit+1):
		if checkingPrime(number):
			count = count + 1
	elapsed_time = time.time() - start_time
	lLimit=temp
	print("There are " + str(count) + " primes between " + str(lLimit) + " and " + str(uLimit)) 
	print("Time elapsed " + str(elapsed_time) + " seconds.")
	
	return count, elapsed_time
	
