# Hetero-MAPF

## Overview
Solve MAPF problem when the agents are heterogeneous.

<img src="https://i.loli.net/2021/02/21/eijqSGzHZ8fvlRT.gif" width="40%" height="40%">

## Source Code
### Requirement

**For the C++ Code**

```bash
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
clang-tidy clang-format python3-matplotlib libompl-dev libeigen3-dev
```
**For the Python visualization script**

```bash
python3 -m pip install -r requirements.txt
```

### Build
```bash
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release  ..
make -j8
```

* `make`: Build Hetero-MAPF code
* `make clang-format`: Re-format all source files
* `make all`: Build all three targets above

> Todo: Prepare for doxygen.

### Run example instances
```bash
# make sure your are in build folder
# default 5 agent in a batch
./planner -i ../maps/random.yaml -o output.yaml 
# or compute 10 agents in a whole batch
./planner -i ../maps/random.yaml -o output.yaml -b 10
```
The `maps` folder also contains a warehouse map for test.

### Visualize Results
```bash
# make sure your are in build folder
python3 ../src/visualize.py -m ../maps/random.yaml -s output.yaml 
```

### Agent Configuration
The agent configurations, including the size, the kinematic constraints, and penalty functions can be changed in `src/config.yaml`.


## License

The code is provided under the [MIT License](https://opensource.org/licenses/MIT).