#!/usr/bin/env python

# Credit to: https://github.com/hasauino/rrt_exploration.git
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from pimouse_slam.msg import PointArray
from numpy import array
from functions import robot,informationGain,discount
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalData=OccupancyGrid()
globalmaps=[]

def callBack(data):
    global frontiers
    frontiers=[]
    for point in data.points:
        frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
    global frontiers,mapData,globalData,globalmaps
    rospy.init_node('assigner', anonymous=False)

    # fetching all parameters
    map_topic= rospy.get_param('~map_topic','map')
    info_radius= rospy.get_param('~info_radius',1.0) #this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_multiplier=rospy.get_param('~info_multiplier',3.0)
    hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0) #at least as much as the laser scanner range
    hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0) #bigger than 1 (biase robot to continue exploring current region)
    frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points') 
    namespace = rospy.get_param('~namespace','')
    namespace_init_count = rospy.get_param('namespace_init_count',1)
    delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
    rateHz = rospy.get_param('~rate',100)

    rate = rospy.Rate(rateHz)
#-------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------

# wait if no frontier is received yet
    while len(frontiers)<1:
        pass
    centroids=copy(frontiers)
#wait if map is not received yet
    while (len(mapData.data)<1):
        pass
    robots=[]
    robots.append(robot(namespace))
    robots[0].sendGoal(robots[0].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids=copy(frontiers)

#-------------------------------------------------------------------------
#Get information gain for each frontier point
        infoGain=[]
        for ip in range(0,len(centroids)):
            infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------
#get dicount and update informationGain
        infoGain=discount(mapData,robots[0].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------
        revenue_record=[]
        centroid_record=[]
        id_record=[]

        for ip in range(0,len(centroids)):
            cost=norm(robots[0].getPosition()-centroids[ip])
            information_gain=infoGain[ip]
            if (norm(robots[0].getPosition()-centroids[ip])<=hysteresis_radius):

                information_gain*=hysteresis_gain
            revenue=information_gain*info_multiplier-cost
            revenue_record.append(revenue)
            centroid_record.append(centroids[ip])
            id_record.append(0)

        rospy.loginfo("revenue record: "+str(revenue_record))
        rospy.loginfo("centroid record: "+str(centroid_record))
        rospy.loginfo("robot IDs record: "+str(id_record))

#-------------------------------------------------------------------------
        if (len(id_record)>0):
            winner_id=revenue_record.index(max(revenue_record))
            robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
            rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))   
            rospy.sleep(delay_after_assignement)
#-------------------------------------------------------------------------
        rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass

