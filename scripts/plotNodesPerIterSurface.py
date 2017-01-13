import argparse
import numpy as np
import matplotlib.pyplot as plt

# python plotAnytimePerformance.py --logs /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/flow.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/magnusson.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/flow-ordered.log /Users/chaubold/hci/data/hufnagel2012-08-03/2016-01-29-flow-result-comparison/gurobi.log --labels flow magnusson flow-orderedNodes gurobi --out /Users/chaubold/Dropbox/VerbTeX/eccv16-divisibleCellFlow2/fig/anytime-drosophila.pdf
# python plotAnytimePerformance.py --logs /Users/chaubold/hci/data/rapoport_sub/flow.log /Users/chaubold/hci/data/rapoport_sub/magnusson.log /Users/chaubold/hci/data/rapoport_sub/flow-orderedNodes.log /Users/chaubold/hci/data/rapoport_sub/gurobi.log --labels flow magnusson flowOrdered gurobi --out /Users/chaubold/Dropbox/VerbTeX/eccv16-divisibleCellFlow2/fig/anytime-rapoport.pdf

def parseFile(filename):
    time = 0.0
    points = []
    zeroEnergy = 0.0

    # first pass checks for the highest number of iterations
    maxNumIters = 0
    numPaths = 1
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()

            if 'Finished after' in line:
                maxNumIters = max(maxNumIters, int(line.split(' ')[2]))
            elif '<<<Iteration' in line:
                numPaths += 1

    surf = np.zeros([numPaths, maxNumIters+1])
    pathIdx = 0
    iterIdx = 0
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('Updating') and line.endswith('nodes'):
                numNodes = int(line.split(' ')[1])
                surf[pathIdx, iterIdx] = numNodes
                iterIdx += 1
            elif '<<<Iteration' in line:
                pathIdx += 1
                iterIdx = 0
            
    return surf

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""
        Plot a surface of the number of nodes updated per iteration.
        """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--log', dest='log', type=str, required=True, help='list of files containing logs')
    parser.add_argument('--label', dest='label', type=str, required=True, help='list of labels to use for each line')
    parser.add_argument('--out', dest='outFile', type=str, required=True, help='output filename')
    args = parser.parse_args()


    surf = parseFile(args.log)
    print(surf.shape)

    from mpl_toolkits.mplot3d import Axes3D # for 3D surface plot
    x, y = np.meshgrid(list(range(surf.shape[1]), range(surf.shape[0])))
    print(x.shape)
    print(y.shape)

    fig = plt.figure()
    # plt.title(args.label)

    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(x, y, surf, rstride=10, cstride=10, cmap='jet',
                           linewidth=0, antialiased=False)
    plt.show()
    # plt.savefig(args.outFile)


