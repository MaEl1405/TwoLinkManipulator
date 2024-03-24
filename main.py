from TwoLinkManipulator import TwoLinkManipulator
import numpy as np

L1=1                 #length of link1
L2=2                 #length of link2
theta1=np.pi/3       #joint1 rotation   
theta2=np.pi/5       #joint2 rotation

#create robot object
robot = TwoLinkManipulator(L1,L2,theta1,theta2)
robot.motion_plot()
