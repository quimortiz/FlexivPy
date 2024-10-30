# FlexivPy

# new:

(base) quim@mango:~$ xhost +


# Instructions for use without dev containers and vscode


```
docker build -t my_project_image .
docker run -it -v (pwd):/home/FlexivPy  my_project_image
```
Modify the second command as needed. just check the dev containers file!

Install nvim

```
bash /home/FlexivPy/install_nvim.sh 
source ~/.bashrc 
```

using devcontainers cli

```
devcontainer up --workspace-folder .
```

then check the container 

```
docker ps                                                                                                                                 (base) 
```
and attach to it

```
docker exec -it 9ab86f8990400ce7c944024c3a4d9048c3ada921848c996b26e342ced5f73912 bash
```


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
sudo robot_server --path ../../assets/
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



git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx                           (/home/quim/code/alex/env)
                                          cd cyclonedds-cxx && mkdir build && cd build
                                          cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX ..
                                          cmake --build .
                                          cmake --build . --target install


```
git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx
cd build
cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
        -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
        ..
cmake --build .
cmake --build . --target install
```

Install Flexiv Sdk

Follow the instructions in:


https://github.com/flexivrobotics/flexiv_rdk

```
git clone https://github.com/flexivrobotics/flexiv_rdk
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh $CONDA_PREFIX
cd ../..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DINSTALL_PYTHON_RDK=OFF
cd flexiv_rdk/build
cmake --build . --target install --config Release
```


# Acknowledgement

This repository is inspired by:

https://github.com/Rooholla-KhorramBakht/go2Py

https://github.com/Rooholla-KhorramBakht/FR3Py

https://github.com/machines-in-motion/dynamic_graph_head
