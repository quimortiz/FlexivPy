# FlexivPy

# Instructions Machines in Motion

1 - Turn on the robot, connect the ethernet cabel, put the safety switch up, slider should on position with two arrows up.

2 - Check that we can talk to the robot.

```
ping 192.168.2.100
```

3 - Compile the robot server.
From the root of the FlexivPy repository:

```
cd FlexivPy/cpp
mkdir && cd build
cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
make
```
with gripper
```
sudo ./robot_server -g --path ../../assets/ -rcf ../config.yaml
```
withour gripper remove -g

use 
```
sudo ./robot_server --help to see all flags
```


There are a lot of command line options!
If the robot is using the Flexiv gripper add `-g`. Check them all with:

```
sudo ./robot_server --help
```


Do not deactivate safety checks without reading the c++ code before and asking people from the lab.



4 - Run the hello world example from Python in another terminal.

```
PYTHONPATH=. python examples/hello_world.py --mode=real
```

Now you can run your code!

Before running on the real robot, try out the synchronous mujoco simulation and the asynchronous mujoco simulation.

Run for example:

```
PYTHONPATH=. python examples/hello_world.py --mode=sim
PYTHONPATH=. python examples/hello_world.py --mode=sim_async
```





# Experiments:

- in state: send the joint torque as a msg!
- One joint: how high the gains?
X Compare grav. compensation.
NO Urdf with better dynamical model.
X Task space impedance (moving one, rigid another)
- Excert force without/with Force sensor - z axis control using measurement of force
X More security checks.
- Reaching task using Croccodyll
X Option to do grav. comp. with MUJOCO.
X Rerun integration
X Camera simulation
X IK controller
X Smooth Polynomials
X Run controller by controller interface
- Camera in real
- Vicon?/
- Visual Markers with camera



# Dependencies

We recommend to install all packages in a conda environment!

```
conda install conda-forge::cmake
conda install conda-forge::mim-solvers
```

## Async Simulation

We need Cyclone DDS with Python Bindings

```
git clone https://github.com/eclipse-cyclonedds/cyclonedds
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
cmake --build . --config Release --target install
cd ..
export CYCLONEDDS_HOME=$CONDA_PREFIX
pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python
```

## Real Robot

We need Cyclone DDS with both C++ bindings and Python Bindings

(see Async simulation to install first cyclone DDS with Python Bindings)



```
git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx
cd cyclonedds-cxx && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX ..
cmake --build .
cmake --build . --target install
cd ../..
```



Install Flexiv Sdk

Follow the instructions in:


https://github.com/flexivrobotics/flexiv_rdk

```
git clone https://github.com/flexivrobotics/flexiv_rdk
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh $CONDA_PREFIX
cd ..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DINSTALL_PYTHON_RDK=OFF
cd flexiv_rdk/build
cmake --build . --target install --config Release
cd ../..
```


# Acknowledgement

This repository is inspired by:

https://github.com/Rooholla-KhorramBakht/go2Py

https://github.com/Rooholla-KhorramBakht/FR3Py

https://github.com/machines-in-motion/dynamic_graph_head
