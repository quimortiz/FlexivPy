# FlexivPy

## Getting Started

1. Turn on the robot, connect the Ethernet cable, lift the safety switch, and ensure the slider button is in the position with two arrows pointing up.

2. Check that you can communicate with the robot:
   ```bash
   ping 192.168.2.100
   ```

3. Get the code!

   We provide the infrastructure to run the code in a Docker container using Dev Containers, which integrates nicely with VS Code.
   ```bash
   git clone https://github.com/Rooholla-KhorramBakht/FlexivPy
   cd FlexivPy
   code .
   ```

4. In VS Code, press **Ctrl+Shift+P**, select **"Dev Containers: Rebuild and Reopen in Container"**, or something similar.

5. Open a new terminal in VS Code.

   Now start the robot server:
   ```bash
   robot_server -cm 3 -g --path /home/FlexivPy/FlexivPy/assets/ -rcf /home/FlexivPy/flexivpy_bridge/config.yaml
   ```

6. How to run your code?

   **Option 1:** Create your own fork of FlexivPy and develop there.

   **Option 2:** Add FlexivPy as a dependency (e.g., as a submodule). Run the bridge and the Python part in the same project (you may reuse the Dockerfile).

   **Option 3:** Run the C++ bridge in a different terminal/environment, and only add the Python part as a dependency to your project.

   **Adding the Python part to your project**  
   If you don’t want to modify the code:
   ```bash
   pip install git+https://github.com/Rooholla-KhorramBakht/FlexivPy
   ```
   
   If you want to modify the Python code:
   ```bash
   mkdir deps
   cd deps
   git clone https://github.com/Rooholla-KhorramBakht/FlexivPy
   pip install .
   ```

   You will also need the Python binding of cyclone-dds. You can put all into a deps repository.
   ```bash
   git clone https://github.com/eclipse-cyclonedds/cyclonedds
   cd cyclonedds && mkdir build install && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
   cmake --build . --config Release --target install
   cd ..
   export CYCLONEDDS_HOME=$CONDA_PREFIX
   pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python
   ```


## Troubleshooting

1. If the docker container doesn't have access to 

```
xhost +
```

2. When you open flexivpy in vscode, make sure to rebuild the container so that it is mounted in your user. Otherwise, vscode might reuse an old container mounted in another user. Another option is to change the name of the container.


# Instructions Machines in Motion

1 - Turn on the robot, connect the ethernet cabel, put the safety switch up, slider button should on position with two arrows up.

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
