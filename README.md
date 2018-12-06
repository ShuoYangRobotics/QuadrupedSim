Two years ago after reading "Legged Robots that Balance", I started to work on a MATLAB simulation of a quadruped robot. Initially this project is put in my "toy_code" repo. This semester I will use this quadruped robot for two of my CMU course projects. So it is necessary to create a dedicated repository for it. 

[![robot](https://img.youtube.com/vi/ByDtmprLmHg/0.jpg)](https://www.youtube.com/watch?v=ByDtmprLmHg)
----------

General Principle

Simulate a walking quadruped robot is easy (But remember all simulations are wrong), first construct body and legs, then carefully understand the frame transformations among them to get forward and inverse kinematics. So the robot knows how to place its feet. 

Then find a good method to simulate the contact forces between a foot and the ground. This is proved to be very difficult because few exisiting contact models are precise enough compared to real world scenario. Besides, bad contact force model can make simulation run slower, for the contact force will make the entire simulation model "stiff". In the current implementation, I use [Simscape Multibody Contact Forces Library][1] created by Steve Miller. 

Then design a gait planner to move feet in a periodic manner. 


----------


Helper code:

 1. *forward_kinematics.m* Calculate forward kinematics of the robot leg (Each leg is a simple three-joint manipulator. From shoulder to foot tip, jonts are labelled as "shoulder", "upper" and "knee". Then in the forward kinematics they are abbreviated as "s" "u" and "k") 
 2. *inverse_kinematics.m* Calculate inverse kinematics
 3. *init_quad_new_leg.m* Initialize some parameters


**quad_XXXX.slx** will be the name of different models. I will make sure all **quad_XXXX.sx** file has a corresponding **init_quad_XXXX.m** initialization file. 

By Dec-6-2018, the best running model is 

*quad_new_leg_raibert_strategy_v2.slx*

and its init file

*init_quad_new_leg_raibert_strategy_v2.m*

 


----------
Notice:

 1. Check [this post][2] to properly use contact forces library


  [1]: https://www.mathworks.com/matlabcentral/fileexchange/47417-simscape-multibody-contact-forces-library
  [2]: https://www.mathworks.com/matlabcentral/answers/378561-rigidly-connected-port-error-with-simscape-multibody-contact-forces-library
