import sys

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("Count the number of iterations in a track-log")
		print("Usage: {} <yourlogfile>".format(sys.argv[0]))
		sys.exit(0)

	numIterations = 0

	with open(sys.argv[1], 'r') as f:
		for l in f:
			if 'Finished after' in l:
				numIters =  int(l.strip().split(' ')[-2])
				numIterations += numIters

	print("Ran {} iterations in total".format(numIterations))

