
from primeCalc import *

#calculates locally (no servers) the amount of primes in a range of natural numbers.

if __name__ == "__main__":#Main function

	#get input for user
	lLimit = input("input lower limit ")
	uLimit = input("input upper limit ")
	print "\n"
	
	#Send to countPrimes in calcPrime.py for calculation and gets results
	num,time = countPrimes(lLimit, uLimit)

	#prints out results
	print "answer is " + str(num) #Total prime nos from each server
	print "Total time elapsed " +str(time) #Time after each result has been returned

