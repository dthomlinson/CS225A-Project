# cs225a

This repository will contain the code for our CS225a Experimental Robotics project.

## Dependencies
The project depends on the sai2 libraries.

## Build and run
in the main folder make a build folder and compile from there
```
mkdir build
cd build
cmake .. && make -j4
```
## run the code
go to the bin folder and then to the folder of the application you want to run.
for Environment
```
cd bin/Environment
./simviz_spacerobotics
```

### Environment
You have 2 programs there. A visualizer and the actual homework file.
The visualizer is here to help you make sure you are doing what you think you are doing.
To run it, go to bin/CS225A-Project and run ./simviz_spacerobotics. You will see a window appear with the robot in an initial configuration.
When you run [FILE NAME] and modify the values for the joints, it will modify the position of the visualized robot as long as you publish the new joint values to redis from [FILE NAME]
```
redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY,robot->_q);
```
