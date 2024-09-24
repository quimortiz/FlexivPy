# FlexivPy


# TODO: 


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
cmake --build . --config RelWithDebInfo --target install
cd ../..
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

