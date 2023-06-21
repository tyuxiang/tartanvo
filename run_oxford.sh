# Arguments:
# 1: Dataset: [oxford, 4seasons, singapore]
# 2: Sequence: [10-29, 05-29, 11-25, ...]
# 3: Experiment name: [original]


# Run all the processes simultaneously
roscore & python tartanvo_node.py & rosrun rviz rviz -d /home/marvl/Packages/tartanvo/$1.rviz & rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100 & rosbag record tartanvo_pose -O /home/marvl/Implementation/testing-suite/Methods/tartanvo/results/$1/$2/$1-$2-$3.bag /topic __name:=my_bag & python publish_image_from_folder.py --base_img_folder /mnt/drive1/marvl/data/robotcar/2015-10-29-12-18-17/stereo/left_undistort --skipped_frames 37300 --rate 16 --dataset $1

# )
# python publish_image_from_folder.py --base_img_folder /mnt/drive1/marvl/data/robotcar/2015-10-29-12-18-17/stereo/left_undistort --skipped_frames 1080 --rate 16 --dataset oxford
rosnode kill /my_bag
sleep 2

# Convert rosbag to csv
cd /home/marvl/Implementation/testing-suite/Methods/tartanvo/results/$1/$2
rosrun rosbag_to_csv rosbag_to_csv.py
# cd /home/marvl/Implementation/testing-suite/Methods/tartanvo

# # Evaluate
# python eval_tartanvo.py --base_path five --est_path /home/marvl/Packages/tartanvo/results/Oxford/10-29/10-29-tartanvo_pose-tstamped-run1tartanvo_pose.csv --gt_path /mnt/drive1/marvl/data/robotcar/2015-10-29-12-18-17/stereo_gt_10-29-v3.out --gt_ts_path /mnt/drive1/marvl/data/robotcar/2015-10-29-12-18-17/gt_ts_10-29.out --save_plot_path /home/marvl/Packages/tartanvo/results/Oxford/10-29/run1_plot.png --save_file /home/marvl/Packages/tartanvo/results/Oxford/10-29/run1_eval_result.txt --timestamp_divide_by_digits 6 --skipped_frames 1080 --trim_to_frame -1

jobs -p | xargs kill