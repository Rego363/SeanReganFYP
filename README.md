# SeanReganFYP
Final Year Project

Robots:
human is used to record data from the player while they are racing
berniw_2004 is to use the Neural Network to race

How To ...

Run C++ Model loading demo with example:
1) Go to SeanReganFYP/ModelLoading/build
2) Open terminal
3) Run command- ./modelLoader model.pt

Run Neural Network training and tests:
1) Go to SeanReganFYP/
2) Open terminal
3) Run command- Python3 PyTorchModel.py

Compile robot and run in TORCS:
1) Copy and paste robot files to $TORCS_BASE/src/drivers/
2) Open terminal
3) cd $TORCS_BASE/src/drivers/RobotName/
4) Run command- make
5) Run command- make install
6) Run command- sudo cp -a $TORCS_BASE/export/drivers/RobotName/. /usr/lib/x86_64-linux-gnu/torcs/drivers/RobotName/
