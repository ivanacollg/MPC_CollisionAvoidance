# MPC_CollisionAvoidance

## Depends

### [Python 3.7](https://medium.com/analytics-vidhya/how-to-install-and-switch-between-different-python-versions-in-ubuntu-16-04-dc1726796b9b)
```
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt update
  sudo apt install python3.7

```
### Python deendencies
```
python3.7 -m pip install pip
pip3.7 install numpy
pip3.7 install matplotlib
pip3.7 install scipy
pip3.7 install future-fstrings
pip3.7 install casadi>=3.5.1
sudo apt-get install python3.7-tk

```

## To Use:
```
  git clone 
  git submodule update --recursive --init
  cd MPC_CollisionAvoidance/catkin_ws/src/nmpc_ca/acados/
  git submodule update --recursive --init
  mkdir -p build
  cd build
  cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ..
  make install 
  cd ../interfaces/acados_template/
  pip3.7 install -e .
```
Add the path to the compiled shared libraries libacados.so, libblasfeo.so, libhpipm.so to LD_LIBRARY_PATH (default path is <acados_root/lib>) by running:
```
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
```
Tipp: you can add this line to your .bashrc/.zshrc.

Run acados example:
<acados_root>/examples/acados_python/getting_started/minimal_example_ocp.py
Answere yes when asked to set up Tera Render automatically
