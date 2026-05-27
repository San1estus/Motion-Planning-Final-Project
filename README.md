# Motion Planning Final Project
RRT* implementation for a Dubins car model.
## Compiling the project
To run the code download with

    git clone https://github.com/San1estus/Motion-Planning-Final-Project/
To build the project you must have OpenGL and GLFW installed. Then just run

    cmake -S . -B build
    cmake --build build
move to the build directory and run RRT_car.exe.

## Controls

- LMB: place starting node
- RMB: place goal node
- O: change to obstacle mode/return to placing
- When in obstacle mode: drag LMB to draw obstacles
- When in placing mode: press Space to run the simulation

