# sample_drl_planner
A path planning framework based on Sampling-based algorithm and Deep Reinforcement learning.

# Platform
| interface | version |
| --------- | ------- |
| ubuntu    | 18.04   |
| ros       | melodic |
| pytorch   | 1.4.0   |

# Installation

**ROS melodic**

refer to [here](http://wiki.ros.org/Installation/Ubuntu)

**Cmake version 3.5  or above**

**OMPL 1.5.2**

Install dependence
```
sudo apt install libboost-dev libeigen3-dev pkg-config
sudo pip install numpy pyplusplus flask celery fcl
```
Install OMPL

refer to [here](https://ompl.kavrakilab.org/download.html)

**Anaconda**

refer to [here](https://www.anaconda.com/)

**virtual environment**

create env

```
conda create -n my_env python=2.7
conda activate my_env
```
update pip
```
pip install --upgrade pip
```
install dependence

```
pip install requirements.txt
```

**Building pkg**

create workspace

```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```
Download this project and put it into `src`.

```
cd ..
catkin_make
```

**Run Samples**

train
```
roslaunch my_simple_planner train_env.launch
rosrun my_simple_planner train.py
```
test
```
roslaunch my_simple_planner test_env.launch
rosrun my_simple_planner test.py
```