# Pointing Project

## About
This project determine what a person is pointing at using a rgb-d camera.  

## Setup
### Python Venv
We use pytorch for object recognition.  To use the code you'll need to set up a venv.  The instruction for setting up the venv can be found [here](https://github.com/marikamurphy/pointing/blob/master/other/objectDetection/README.md).  
### Building source code
Go to [point_interpretation](https://github.com/marikamurphy/pointing/tree/master/src/point_interpretation) and create a directory called build.  cd into the build directory and run 
> cmake ..  
After that you can run *make* in the build directory whenever you want to compile the program.

## Running the project
Before running the main logic in point_interpretation, you need to start the server code.  Go into the venv you setup earlier (*source /path/to/venv/bin/activate*) and then run [server.py](https://github.com/marikamurphy/pointing/blob/master/other/objectDetection/server.py) with 
> python server.py  
Now you can run point_interpretation.  The executable should be in the build directory you created once you compile with *make*.
