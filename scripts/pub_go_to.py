#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

def main():
    pos_aruco1 = [1.0, 0.7, 1.0]
    pos_aruco2 = [-15.0, -1.3, 1.0]

    Z = 1.0

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for cf in allcfs.crazyflies:
        if cf.uri == 'udp://0.0.0.0:19850':
            cf1 = cf

    # Cf1 go_to aruco 1
    pos_1 = np.array(pos_aruco1)

    # Cf1 go_to aruco 2
    pos_2 = np.array(pos_aruco2)

    # goTo(self, goal, yaw, duration, relative=False, groupMask=0) 
    # relative= False -> relative to the world frame, 'True' is relative to the drone frame
    
    cf1.goTo(pos_1, np.pi/2, 15.0)
    print("Go To 1")

    timeHelper.sleep(40.0)

    cf1.goTo(pos_2, np.pi/2, 15.0)
    print("Go To 2")

    timeHelper.sleep(40.0)

    cf1.land(targetHeight=0.02, duration=1.0+Z)

if __name__ == '__main__':
    main()
