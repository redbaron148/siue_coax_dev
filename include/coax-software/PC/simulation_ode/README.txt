

Installing the exoMarsRover Sim:
> cd build
> cmake ..
> make




ODE has to support double precision math. If not already supported, ODE has to be recompiled as follows:

Compiling ODE with double precision:

./configure  --enable-double-precision
