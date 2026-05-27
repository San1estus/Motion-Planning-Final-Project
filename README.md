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
- R: reset the playground

## Tests

https://github.com/user-attachments/assets/4975bd39-f88c-40ef-8d6c-96ad15f01c86

This example is a 3x speed

https://github.com/user-attachments/assets/9ad5b1c6-e90e-41c2-9061-7908a7f26b19
