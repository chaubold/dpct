import argparse
import numpy as np

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

# python plotAnytimePerformance.py --logs /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/flow.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/magnusson.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/flow-ordered.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/gurobi.log --labels flow magnusson flow-orderedNodes gurobi --out /Users/chaubold/Dropbox/VerbTeX/eccv16-divisibleCellFlow2/fig/anytime-drosophila.pdf
# python plotAnytimePerformance.py --logs /Users/chaubold/hci/data/rapoport_sub/flow.log /Users/chaubold/hci/data/rapoport_sub/magnusson.log /Users/chaubold/hci/data/rapoport_sub/flow-orderedNodes.log /Users/chaubold/hci/data/rapoport_sub/gurobi.log --labels flow magnusson flowOrdered gurobi --out /Users/chaubold/Dropbox/VerbTeX/eccv16-divisibleCellFlow2/fig/anytime-rapoport.pdf

def parseFile(filename):
    time = 0.0
    points = []
    zeroEnergy = 0.0

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if 'Model has state zero energy:' in line:
                zeroEnergy = float(line.split(':')[1])
                continue
            elif line.startswith('Explored') and line.endswith('seconds'):
                time = float(line.split(' ')[-2])
                continue
            elif line.startswith('Total (root+branch&cut)'):
                time = float(line.split('=')[1].split('s')[0].strip())
                continue
            elif 'solution has energy:' in line:
                energy = float(line.split(':')[-1])
            elif line.startswith('Found') and 'overall score=' in line:
                # parse magnusson log
                a = line.split(' ')
                energy = zeroEnergy - float(a[4].split('=')[1])
                time = float(a[-2])

            elif '<<<Iteration' in line:
                # parse flow solver log
                a = line.split(' ')
                energy=float(a[-1].split('=')[1])
                timeDelta = float(a[5])
                time += timeDelta
            else:
                continue

            points.append([time, energy])
    return np.array(points)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""
        Plot anytime performance from logs.
        """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--logs', dest='logs', type=str, nargs='+', required=True, help='list of files containing logs')
    parser.add_argument('--labels', dest='labels', type=str, nargs='+', required=True, help='list of labels to use for each line')
    parser.add_argument('--out', dest='outFile', type=str, required=True, help='output filename')
    args = parser.parse_args()

    assert(len(args.logs) == len(args.labels))

    plt.figure()
    plt.hold(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Energy')
    plt.yscale('log')

    lower = np.infty
    upper = -np.infty
    maxTime = 0
    colors = ['r', 'g', 'b', 'y', 'c', 'm', 'k']
    scatterIdx = 0

    for log, label in zip(args.logs, args.labels):
        points = parseFile(log)

        lower = min(points[:,1].min(), lower)
        upper = max(points[:,1].max(), upper)
        maxTime = max(points[:,0].max(), maxTime)

        if points.shape[0] == 1:
            plt.scatter(points[:,0], points[:,1], label=label, color=colors[scatterIdx])
            scatterIdx += 1
        else:
            plt.plot(points[:,0], points[:,1], label=label)

    padding = 0.01 * (upper - lower)
    plt.ylim([lower-padding, upper+10*padding])
    plt.xlim([0, maxTime*1.01])
    plt.legend()
    plt.savefig(args.outFile)


