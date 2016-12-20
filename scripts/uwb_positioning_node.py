#!/usr/bin/env python
import rospy
import numpy as np
# from uwb_positioning import Positioning
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose2D
from timedomain_uwb.msg import P440Range
# from math import pi,cos,sin
from scipy.linalg import inv,block_diag

rospy.init_node('uwb_positioning_node')

print "UWB STUFF"

# ----- Parameters
range_topic_id = rospy.get_param('~range_topic_id','/p440_range')
pos_topic_id = rospy.get_param('~position_topic_id','/uwb_position')

max_delay = rospy.get_param('~max_measurement_delay',0.3)

beaconIdString = rospy.get_param('~uwb_beacon_ids','[101,102,103]')

beaconPosXString = rospy.get_param('~uwb_beacon_positions_x','[0,1.5,3]')
beaconPosYString = rospy.get_param('~uwb_beacon_positions_y','[0,1.5,3]')
beaconPosZString = rospy.get_param('~uwb_beacon_positions_z',None)


global pos_x,pos_y
pos_x = rospy.get_param('~pos_x_i',-0.4)
pos_y = rospy.get_param('~pos_x_i',10.25)

# ----- Publishers
# pose_ext_pub = rospy.Publisher('/tennibot_fused_pose_extended', PoseWithCovarianceStamped, queue_size=1000)
pose_pub = rospy.Publisher(pos_topic_id, Pose2D, queue_size=1000)

# est = Positioning() # instance of Positioning class

def stringToFloatList(in_string,mapping_type):
  out_list = None
  if in_string:
    tmp1 = in_string[1:(len(in_string)-1)]
    tmp2 = tmp1.split(',')
    out_list = map(mapping_type,tmp2)
  return out_list

beaconIdList = stringToFloatList(beaconIdString,int)
beaconPosXList = stringToFloatList(beaconPosXString,float)
beaconPosYList = stringToFloatList(beaconPosYString,float)
beaconPosZList = stringToFloatList(beaconPosZString,float)

receive_time = np.array([0.,0.,0.])
Y = np.matrix((  (0.),(0.),(0.) )).transpose()
# meas.b_id_pos = np.array([beaconIdList[0],beaconPosXList[0],beaconPosYList[0]],[beaconIdList[1],beaconPosXList[1],beaconPosYList[1]],[beaconIdList[2],beaconPosXList[2],beaconPosYList[2]])

def indexFromBeaconId(beaconId):
  i=0
  for b_id in beaconIdList:
    if (b_id==beaconId):
      return i
    i+=1

# def addRangeMeasurement(rangeVal,rangeVar,beaconId,time):
#   Y[indexFromBeaconId(beaconId)]==rangeVal

def measurementEquation(pos,bidx):
  return pow(pow(pos[0,0]-beaconPosXList[bidx],2)+pow(pos[1,0]-beaconPosYList[bidx],2),0.5)

def rangeCallback(msg):
  global pos_x,pos_y

  cur_time = msg.header.stamp
  cur_range = float(msg.filteredRange)/1000
  cur_rangeVariance = float(msg.filteredRangeErrEst)/1000
  cur_beaconId = msg.destId

  Y[indexFromBeaconId(cur_beaconId)]=cur_range
  
  for y in Y:
    if (y == 0.):
      return

  xhat = np.matrix((  (pos_x),(pos_y) )).transpose()
  yhat = np.matrix((  (0.),(0.),(0.) )).transpose()
  G = np.matrix((  (0.,0.),
                   (0.,0.),
                   (0.,0.) ))

  dxhat = np.matrix((  (100.),(100.) )).transpose()
  
  while (sum(dxhat)>0.01):
    # Calculate geometry matrix based on best estimate
    i=0
    for tmp in beaconPosXList:
      yhat[i,0] = measurementEquation(xhat,i)
      G[i,0]=(xhat[0,0]-beaconPosXList[i])/yhat[i,0]
      G[i,1]=(xhat[1,0]-beaconPosYList[i])/yhat[i,0]
      i+=1
    dy = Y-yhat
    dxhat = (inv(G.T*G)*G.T)*dy;
    xhat+=dxhat

  pos_x = xhat[0,0]
  pos_y = xhat[1,0]
  Y[0,0]=0.
  Y[1,0]=0.
  Y[2,0]=0.

  pose_msg = Pose2D()
  pose_msg.x = xhat[0]
  pose_msg.y = xhat[1]
  pose_msg.theta = 0.

  pose_pub.publish(pose_msg)

uwb_range_sub = rospy.Subscriber('/p440_range', P440Range, rangeCallback, queue_size=1000)

rospy.spin()
