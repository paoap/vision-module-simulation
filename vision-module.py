#!/usr/bin/env python

from settings import settings

import numpy as np
import pandas as pd

import os

import rospy                                # ros library for publishing and subscribing
from std_msgs.msg import Int16MultiArray       # ros library for string type of msgs


def detection(K, fps, v, Xi, lcr, W, ww):
    time_factor = 1
    dt = 1./fps
    time_waiting = dt/time_factor

    '''
    the reference to get each tested value is tk, which is the absolute time elapsed
    1. Create a matrix to store the values
    first column, time
    second column, right most hill
    third column, with weeds or without weeds
    fourth column, if with weeds, calculate the frame coordinate
    2. -1 value means none
    '''
    my_data = np.full((int(K),5),"",object)
    my_data = pd.DataFrame(data=my_data)

    dt = 1./fps
    time_waiting = dt/time_factor

    '''
    the reference to get each tested value is tk, which is the absolute time elapsed
    1. Created a dataframe to store the values
    2. "" value means none
    '''
    my_data = np.full((int(K)+1,5),"",object)
    my_data = pd.DataFrame(data=my_data)

    for k in np.arange(int(K)+1):

        # time elapsed per kth frame, add 1 since np.arange starts at 0
        tk = k/fps  # s
        my_data.iloc[k,0] = tk
        #print("{}th frame.".format(K))

        # global distance of frame origin per kth frame
        distance_origin_kth_frame = v*tk + W  # m
        # find plant hill indices that are within the camera frame for each frame step
        hills_ik = np.where((Xi > distance_origin_kth_frame-W) & (Xi <= distance_origin_kth_frame))
        hills_ik = np.squeeze(hills_ik)
        
        if np.size(hills_ik)>0:
                        
            #my_data.iloc[K,1] = Xi[hills_ik]
            my_data.iloc[k,1] = hills_ik

            # get bounded hill with weeds            
            with_weeds_index = np.intersect1d(ww,hills_ik)
            
            if np.size(with_weeds_index)>0:
                my_data.iloc[k,3] = with_weeds_index
                xik = (distance_origin_kth_frame - Xi[with_weeds_index])/lcr     # px
                my_data.iloc[k,4] = xik.astype(int)
            else:
                my_data.iloc[k,3] = [-1]
                my_data.iloc[k,4] = [-1]


            # get bounded hill without weeds
            without_weeds_index = np.setdiff1d(hills_ik, with_weeds_index)
            
            if np.size(without_weeds_index)>0:
                my_data.iloc[k,2] = without_weeds_index
            else:
                my_data.iloc[k,2] = [-1]


        else:
            my_data.iloc[k,1] = [-1]
            my_data.iloc[k,2] = [-1]
            my_data.iloc[k,3] = [-1]
            my_data.iloc[k,4] = [-1]

    return my_data, time_waiting

def calculate_detection_rate(detection_data, true_values):

    # calculate overall detection rate
    N = detection_data.shape[0]
    
    # populate detected hills
    detected_hills = []
    for n in np.arange(N):
        detected_hills = np.append(detected_hills,detection_data.iloc[n])

    # delete duplicates and -1
    detected_hills = np.unique(detected_hills)
    detected_hills = detected_hills[detected_hills!=-1]
    
    # compare numpy arrays
    undetected_hills = np.setdiff1d(true_values,detected_hills)
    
    detection_rate = (1.-(float(np.size(undetected_hills))/float(np.size(true_values))))*100.
    
    return detection_rate

def publish_detection(coordinates): # procedure for the main function
    
    # creates a chatter topic, that can hold 10 strings at a time
    pub = rospy.Publisher('coordinates_px', Int16MultiArray, queue_size=1)

    # initialize a talker node
    rospy.init_node('vision-module')
    
    # rate at which this node will publish or check msgs in the container
    rate = rospy.Rate(1) #hz
    
    # loop the node
    while not rospy.is_shutdown():
        for k in np.arange(coordinates.shape[0]-1):

            my_coordinate = np.array(coordinates.iloc[k]).astype(int)
            my_coordinate = Int16MultiArray(data=my_coordinate)

            rospy.loginfo(my_coordinate)
            pub.publish(my_coordinate)

            rate.sleep()

            if rospy.is_shutdown():
                break

def save(data):
    script_dir = os.path.dirname(os.path.realpath(__file__)) + "/results-camera-1.csv"
    data.to_csv (script_dir, index = False, header=True)
    print ("Results saved as {}".format(script_dir))

if __name__ == '__main__':
    
    try:
       
        my_settings = settings()
        my_data, time_waiting = detection(
            my_settings.K,
            my_settings.fps,
            my_settings.v,
            my_settings.Xi,
            my_settings.lcr,
            my_settings.W,
            my_settings.ww)


        my_data.columns = ["Time, s",
            "Hills within the frame",
            "Without Weeds Index",
            "With Weeds Index",
            "Weeds Distance from frame origin, px"]

        print(my_data)
        print("{} frames at {}s per frame".format(my_settings.K,time_waiting))
        print("No. of Hills = {}".format(my_settings.nh))
        #print("hills with weeds = {}".format(my_settings.ww))

        coordinates = my_data.iloc[:,4]
        freq = 1./time_waiting
        

        true_hills = np.arange(my_settings.nh)
        total_dect_rate = calculate_detection_rate(my_data.iloc[:,1],true_hills)
        print("hills detection rate = {}%".format(total_dect_rate))

        true_wow = np.setdiff1d(true_hills,my_settings.ww)
        if np.size(true_wow)>0:
            wow_dect_rate = calculate_detection_rate(my_data.iloc[:,2],true_wow)
            print("Without weeds detection rate = {}%".format(wow_dect_rate))

        if np.size(my_settings.ww)>0:
            ww_dect_rate = calculate_detection_rate(my_data.iloc[:,3],my_settings.ww)
            print("With weeds detection rate = {}%".format(ww_dect_rate))

        save(my_data)
        
        #publish_detection(coordinates)
    

    except rospy.ROSInterruptException:
        
        # when interrupted such as pressing ctrl+C
        pass
