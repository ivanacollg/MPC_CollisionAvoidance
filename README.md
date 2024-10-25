# MPC_CollisionAvoidance

## Dependencies

### [Python 3.7](https://medium.com/analytics-vidhya/how-to-install-and-switch-between-different-python-versions-in-ubuntu-16-04-dc1726796b9b)
```
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt update
  sudo apt install python3.7

```
### Python dependencies
```
python3.7 -m pip install pip
pip3.7 install numpy
pip3.7 install matplotlib
pip3.7 install scipy
pip3.7 install future-fstrings
pip3.7 install casadi>=3.5.1
pip3.7 install setuptools
sudo apt-get install python3.7-tk

```

## To Use:
```
  git clone https://github.com/ivanacollg/MPC_CollisionAvoidance.git
  cd MPC_CollisionAvoidance
  git submodule update --recursive --init
  cd catkin_ws/src/nmpc_ca/acados/
  mkdir -p build
  cd build
  cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ..
  make install 
  cd ../interfaces/acados_template/
  pip3.7 install -e .
  cd ../../../../..
  catkin_make
```
Add the path to the compiled shared libraries libacados.so, libblasfeo.so, libhpipm.so to LD_LIBRARY_PATH (default path is <acados_root/lib>) by running:
```
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
```
Tipp: you can add this line to your .bashrc/.zshrc.

Run acados example:
<acados_root>/examples/acados_python/getting_started/minimal_example_ocp.py

# Please cite 
```
@article{COLLADOGONZALEZ2024118998,
title = {Adaptive sliding mode control with nonlinear MPC-based obstacle avoidance using LiDAR for an autonomous surface vehicle under disturbances},
journal = {Ocean Engineering},
volume = {311},
pages = {118998},
year = {2024},
issn = {0029-8018},
doi = {https://doi.org/10.1016/j.oceaneng.2024.118998},
url = {https://www.sciencedirect.com/science/article/pii/S0029801824023369},
author = {Ivana Collado-Gonzalez and Alejandro Gonzalez-Garcia and Rodolfo Cuan-Urquizo and Carlos Sotelo and David Sotelo and Herman Castañeda}
}

@article{GONZALEZGARCIA2022112900,
title = {Path-following and LiDAR-based obstacle avoidance via NMPC for an autonomous surface vehicle},
journal = {Ocean Engineering},
volume = {266},
pages = {112900},
year = {2022},
issn = {0029-8018},
doi = {https://doi.org/10.1016/j.oceaneng.2022.112900},
url = {https://www.sciencedirect.com/science/article/pii/S0029801822021837},
author = {Alejandro Gonzalez-Garcia and Ivana Collado-Gonzalez and Rodolfo Cuan-Urquizo and Carlos Sotelo and David Sotelo and Herman Castañeda},
}
```
Answere yes when asked to set up Tera Render automatically
