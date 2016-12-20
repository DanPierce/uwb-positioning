#!/usr/bin/env python
import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose2D
from timedomain_uwb.msg import P440Range
from scipy.linalg import inv,block_diag

from math import atan2

# TO DO:
#   add max time delay check
#   add 3D positioning
#   estimate velocity from range rate
#   use arrays instead of matrix
#   add max number of iterations
#   course threshold as velocity

class UwbPositioningNode:

  def __init__(self):

    # ----- Parameters
    range_topic_id = rospy.get_param('~range_topic_id','/p440_range')
    pos_topic_id = rospy.get_param('~position_topic_id','/uwb_position')

    max_delay = rospy.get_param('~max_measurement_delay',0.3)

    beaconIdString = rospy.get_param('~uwb_beacon_ids','[101,102,103]')

    beaconPosXString = rospy.get_param('~uwb_beacon_positions_x','[0,1.5,3]')
    beaconPosYString = rospy.get_param('~uwb_beacon_positions_y','[0,1.5,3]')
    beaconPosZString = rospy.get_param('~uwb_beacon_positions_z',None)


    self.beaconIdList = self.stringToList(beaconIdString,int)
    self.beaconPosXList = self.stringToList(beaconPosXString,float)
    self.beaconPosYList = self.stringToList(beaconPosYString,float)
    self.beaconPosZList = self.stringToList(beaconPosZString,float)

    # Initial position guess
    self.pos_x = rospy.get_param('~pos_x_i',-0.4)
    self.pos_y = rospy.get_param('~pos_x_i',10.25)
    self.psi = rospy.get_param('~heading_i',0.)

    self.receive_time = np.array([0.,0.,0.])
    self.Y = np.matrix((  (0.),(0.),(0.) )).transpose()

    # ----- Publishers
    self.pose_pub = rospy.Publisher(pos_topic_id, Pose2D, queue_size=1000)

    # ----- Subscribers
    uwb_sub = rospy.Subscriber('/p440_range', P440Range, self.range_callback, queue_size=1)


  def measurement_equation(self,pos,bidx):
    return pow(pow(pos[0,0]-self.beaconPosXList[bidx],2)+pow(pos[1,0]-self.beaconPosYList[bidx],2),0.5)

  def range_callback(self,msg):
    cur_time = msg.header.stamp
    cur_range = float(msg.filteredRange)/1000
    cur_rangeVariance = float(msg.filteredRangeErrEst)/1000
    cur_beaconId = msg.destId

    self.Y[self.indexFromBeaconId(cur_beaconId)]=cur_range
    
    for y in self.Y:
      if (y == 0.):
        return

    xhat = np.matrix((  (self.pos_x),(self.pos_y) )).transpose()
    yhat = np.matrix((  (0.),(0.),(0.) )).transpose()
    G = np.matrix((  (0.,0.),
                     (0.,0.),
                     (0.,0.) ))

    dxhat = np.matrix((  (100.),(100.) )).transpose()
    
    while (sum(dxhat)>0.01):
      # Calculate geometry matrix based on best estimate
      for i in range(len(self.beaconPosXList)):
        yhat[i,0] = self.measurement_equation(xhat,i)
        G[i,0]=(xhat[0,0]-self.beaconPosXList[i])/yhat[i,0]
        G[i,1]=(xhat[1,0]-self.beaconPosYList[i])/yhat[i,0]
      dy = self.Y-yhat
      dxhat = (inv(G.T*G)*G.T)*dy;
      xhat+=dxhat

    step_x = xhat[0,0]-self.pos_x
    step_y = xhat[1,0]-self.pos_y
    delta_pos = pow(pow(step_x,2)+pow(step_y,2),0.5)
    if (delta_pos>0.05):
      self.psi = atan2(step_y,step_x)

    self.pos_x = xhat[0,0]
    self.pos_y = xhat[1,0]
    
    self.Y[0,0]=0.
    self.Y[1,0]=0.
    self.Y[2,0]=0.

    pose_msg = Pose2D()
    pose_msg.x = self.pos_x
    pose_msg.y = self.pos_y
    pose_msg.theta = self.psi

    self.pose_pub.publish(pose_msg)

  def indexFromBeaconId(self,beaconId):
    for i in range(len(self.beaconIdList)):
      if (self.beaconIdList[i]==beaconId):
        return i

  def stringToList(self,in_string,mapping_type):
    out_list = None
    if in_string:
      tmp1 = in_string[1:(len(in_string)-1)]
      tmp2 = tmp1.split(',')
      out_list = map(mapping_type,tmp2)
    return out_list

def main(args):
  rospy.init_node('uwb_positioning_node')

  node = UwbPositioningNode()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 
if __name__ == '__main__':
  main(sys.argv)