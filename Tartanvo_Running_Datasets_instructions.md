## TartanVO

Move everything in the repo under an `src` folder. `src` is siblings to `build`, `devel` when the `catkin_build` is run.

The modifications are inside the `datasets` branch of the tartanvo fork under marcelprasetyo's GitHub account:
https://github.com/marcelprasetyo/tartanvo/tree/datasets

To see the diff in files between this and original, click:

https://github.com/castacks/tartanvo/compare/main...marcelprasetyo:tartanvo:datasets?diff=split

This modification does not require the use of timestamp file. It will take the timestamp from the filename, therefore the filenames of the image files need to at least be uniquely identifiable from one another. Ideally they are in the original timestamp filename format.

### Modified files

These modifications are essential

`modified:   publish_image_from_folder.py`
- adds timestamp into the intermediate message header

`modified:   tartanvo_node.py`
- this modification adds the timestamp added into the intermediate message header above into the resulting output message


In `publish_image_from_folder.py`,


```py
parser.add_argument("--base_img_folder", help="base folder in Rain_Datasets for Oxford dataset")
parser.add_argument("--skipped_frames", help="how many frames to be skipped")
parser.add_argument("--rate", help="how many images published per second (Hertz)")
parser.add_argument("--dataset", help="oxford, 4seasons, dso")
```

The above code block describes the arguments.
- `base_img_folder` describes the name of the sequence's folder. See below. `base_img_folder` is `base_folder` below. You can change `data/Oxford/` below to the appropriate dataset path. I followed the TartanVO format of using the `data` folder inside tartanvo, so I created a symlink called `'Oxford'` that points to the `'Rain_Datasets'` folder path.
    - linking `data` folder to dataset folder
    ```sh
        $ cd data
        $ ln -s ~/Desktop/Datasets/4Seasons 4seasons
        $ ls
        4seasons    Oxford      KITTI
        $ ls 4seasons
        10-07   12-09
    ```
- example, base_img_folder will be `10-29-undistort`. The code snippet below shows the way the --dataset argument works alongside with the dataset path and base_img_folder.

```py
if args.dataset == 'oxford':
    img_file_name = os.path.join('data/Oxford/', base_folder, 'stereo/left_undistort')
elif args.dataset == '4seasons':
    img_file_name = os.path.join('data/Oxford/', base_folder, 'undistorted_images/cam0')
elif args.dataset == 'dso':
    img_file_name = os.path.join('data/Oxford/', base_folder, 'left_undistort')
```

- `skipped_frames` - skip how many frames at the start?
- `rate` - FPS of input stream for tartanvo. It shouldn't be too fast that tartanVO can't handle. 10FPS is safe but may be slow. You can experiment yourself
- `dataset` - used to choose the path and folder formats as seen in above code block. 

### New files:

The below files are not really used.

- PYTHON-USES-VENV-P38.txt
- README-modified.md
- oxford.rviz
- oxford.yaml

### Sophus

If needed, in the CMakeLists.txt file, add these lines if there are problems with Sophus:
```cmake
SET(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")
SET(Sophus_INCLUDE_DIRS "/usr/local/include/sophus")
```


## Install rosbag_to_csv to extract the recorded rosbags into csv files

Download Rosbag_to_csv module and put the folder under `src`
link: https://github.com/AtsushiSakai/rosbag_to_csv

make sure to rerun catkin_build or catkin make (whichever was used)

## Running

Comment/uncomment the appropriate intrinsics settings under `publish_image_from_folder.py` for different datasets.

0. Source the TartanVO ROS workspace for each terminal
```sh
$ source devel/setup.bash
```

1. Open a ROS core:
```sh
$ roscore
```

2.  Run the TartanVONode. Activate your tartanvo environment (conda, venv or otherwise)
```sh
$ cd src/tartanvo
$ conda activate tartanvo
$ python tartanvo_node.py
```

3. Record rosbag wherever convenient to store it, e.g. `./results/Oxford/10-29/run1`, and record the topic `tartanvo_pose`, which after using the above modified files for tartanvo_node and publish_image_from_folder, will record the timestamp (filename) of the images:

```sh
$ cd src/tartanvo/results/Oxford/10-29/run1
$ rosbag record tartanvo_pose -O 10-29-tartanvo_pose-tstamped-run1.bag
```

4. (optional) Run RVIZ. Just use the oxford.rviz file
```sh
$ cd src/tartanvo
$ rosrun rviz rviz -d oxford.rviz
```

5. Publish the images, e.g. run the following example
```sh
$ cd src/tartanvo
$ python publish_image_from_folder.py --base_img_folder /home/marvl/Packages/tartanvo/data/EuRoC_V102/mav0/cam0/data --skipped_frames 1080 --rate 14 --dataset oxford

python publish_image_from_folder.py --base_img_folder /mnt/drive1/marvl/data/robotcar/2015-10-29-12-18-17/stereo/left_undistort --skipped_frames 1080 --rate 14 --dataset oxford
```

If RVIZ is run, you should see the trajectory being plotted with arrows

6. When done, stop the rosbag recording (Ctrl + C) (also the other programs started above), and then run `rosbag_to_csv`, and when prompted, choose the appropriate rosbag file. If the rosbag doesn't have any available topics to convert, either the rosbag recording command was wrong, the wrong bag file was chosen, or the topic was not published `(tartanvo_node.py` was not running, or `publish_image_from_folder.py` had an error)

```sh
$ rosbag record tartanvo_pose filename.bag
$ rosrun rosbag_to_csv rosbag_to_csv.py
```


## Evaluation

These files are contained in the repository:

```
tartanvo
	eval-cmd.sh
	eval_tartanvo.py
```

These can be moved to the `results` folder or otherwise. Run it on the `.csv` file extracted from the final step in the section above. If you so wish, you can manually inspect the .csv file to see that the timestamps, format and values are correct.

The `eval-cmd.sh` file contains the commands used for evaluation. You can modify it accordingly and copy and paste it into the terminal to run the evaluation

```sh
python eval_tartanvo.py \
--base_path five \
--est_path five/five-run1-tstampedtartanvo_pose.csv \
--gt_path /home/username/Desktop/Oxford-dataset/Rain_Datasets/five/stereo_gt_nocrop_dso_five.out \
--gt_ts_path /home/username/Desktop/Oxford-dataset/Rain_Datasets/five/gt_ts_nocrop_dso_five.out \
--save_plot_path five/five-run1_plot.png \
--save_file five/five-run1_eval_result.txt \
--timestamp_divide_by_digits 9 \
--skipped_frames 0 \
--trim_to_frame -1
```