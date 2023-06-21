import rosbag
import numpy as np

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag("/home/marvl/Packages/tartanvo/results/Oxford/10-29/10-29-tartanvo_pose-tstamped-run1.bag")
topic = "tartanvo_pose"

poses = []
for topic, msg, t in bag.read_messages(topics=topic):
    time = int(msg.header.frame_id[:-4])
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    qw = msg.pose.orientation.w
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    poses.append([0,0,0,0,time,x,y,z,qx,qy,qz,qw])

# print(poses)
np.savetxt("/home/marvl/Packages/tartanvo/results/Oxford/10-29/run1.csv", 
           np.array(poses), delimiter=',', fmt=["%10.20f","%10.20f","%10.20f","%10.20f","%5.15i","%10.20f","%10.20f","%10.20f","%10.20f","%10.20f","%10.20f","%10.20f"],
           header="a,b,c,d,.header.frame_id,.pose.position.x,.pose.position.y,.pose.position.z,.pose.orientation.x,.pose.orientation.y,.pose.orientation.z,.pose.orientation.w")