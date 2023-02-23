# Dynamic Programming Cell Tracking (DPCT)

> *Note:* This repository is no longer maintained, find the maintained version: here https://github.com/ilastik/dpct/

by Carsten Haubold, 2016

This is a stand-alone tool for running tracking of divisible objects using a modified successive shortest paths solver.

## Installation

### Conda

On OSX and Linux you can install the python module of this package within a conda environment using:

    conda install dpct -c chaubold -c ilastik

### Manual compilation

Requirements: 

* a compiler capable of C++11 (clang or GCC >= 4.8)
* cmake >= 2.8 for configuration (on OSX e.g. `brew install cmake`)
* boost (e.g. `brew install boost`)
* the [lemon](http://lemon.cs.elte.hu/trac/lemon) graph library

If you want to parse the JSON files with comments, use e.g. [commentjson](https://pypi.python.org/pypi/commentjson/) for python, or [Jackson](https://github.com/FasterXML/jackson-core/wiki/JsonParser-Features) for Java.


## Binaries

The `bin` folder contains the tracking tool that can be run from the command line. 
It uses a JSON file formats as input and output (see below). Invok it once to see usage instructions.

* `track`: given a graph and weights, return the best tracking result

**Example:**
```
$ ls
>>> weights.json	track	Makefile	model.json	train

$ ./track -m model.json -w weights.json -o trackingresult.json
>>> lots of output...
```

Or if you want to use it from python, you can create the model and weight as dictionaries (exactly same structure as the JSON format) and then in python run the following:

```python
import dpct

# run tracking
mymodel = {...}
myweights = {"weights": [10,10,500,500]}
result = dpct.trackFlowBased(mymodel, myweights)

```

See [test/test.py](test/test.py) for a complete example.

## JSON file formats

See the [Readme](https://github.com/chaubold/multiHypothesesTracking/blob/master/Readme.md) of the accompanying ILP solver for details of the JSON file format.
The formats are compatible (but only `size_t` ids are allowed here), the only difference is that here we use also the start and end-`timestep` of each detection
to order the nodes by time. See [test/test.py](test/test.py).

## References

The algorithm implemented here is described in:

* C. Haubold, J. Ales, S. Wolf, F. A. Hamprecht. **A Generalized Successive Shortest Paths Solver for Tracking Dividing Targets.** ECCV 2016 Proceedings. [Bibtex](https://hci.iwr.uni-heidelberg.de/biblio/export/bibtex/6077)
