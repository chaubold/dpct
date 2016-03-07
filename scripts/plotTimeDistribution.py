import argparse
import numpy as np

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

def parseFile(filename):
    iterationTimes = []
    initBfTimes = []
    bfTimes = []
    pathTimes = []
    augmentTimes = []
    constraintTimes = []

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if 'initializing BF took' in line:
                t = float(line.split(' ')[-2])
                initBfTimes.append(t)
            elif 'BF took' in line and not 'init' in line:
                t = float(line.split(' ')[-2])
                bfTimes.append(t)
            elif 'extracting path took' in line:
                t = float(line.split(' ')[-2])
                pathTimes.append(t)
            elif 'augmenting flow took' in line:
                t = float(line.split(' ')[4])
                t2 = float(line.split(' ')[-2])
                augmentTimes.append(t)
                constraintTimes.append(t2)
            elif '<<<Iteration' in line:
                # parse flow solver log
                a = line.split(' ')
                timeDelta = float(a[5])
                iterationTimes.append(timeDelta)
            else:
                continue

    return np.array(initBfTimes), np.array(bfTimes), np.array(pathTimes), np.array(augmentTimes), np.array(constraintTimes), np.array(iterationTimes)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""
        Plot how the iteration runtime is made up
        """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--log', dest='log', type=str, required=True, help='file containing log')
    parser.add_argument('--out', dest='outFile', type=str, required=True, help='output filename')
    parser.add_argument('--pieout', dest='pieOutFile', type=str, required=False, default=None, help='output filename')
    args = parser.parse_args()

    plt.figure()
    plt.hold(True)
    plt.xlabel('Iteration')
    plt.ylabel('Time [s]')

    colors = ['r', 'g', 'b', 'y', 'c', 'm', 'k']

    timeSeries = parseFile(args.log)
    labels = ['BF init', 'BF', 'pathExtract', 'flowAugment', 'constraintUpdate', 'full iteration']

    for i, label in enumerate(labels):
        plt.plot(timeSeries[i], label=label, color=colors[i])

    plt.legend()
    plt.savefig(args.outFile)

    if args.pieOutFile is not None:
        plt.figure()
        cumulativeTime = [sum(i) for i in timeSeries]
        print(cumulativeTime)
        plt.pie(cumulativeTime[:-1], labels=labels[:-1], colors=colors[:4],
            autopct='%1.1f%%', shadow=True, startangle=90)
        plt.savefig(args.pieOutFile)


