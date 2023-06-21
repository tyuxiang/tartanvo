#  """Save trajectory (absolute poses) as KITTI odometry file format

#     Args:
#         txt (str): pose text file path
#         poses (dict): poses, each pose is a [4x4] array
#         format (str): trajectory format [kitti, tum]. 
#             - **kitti**: timestamp [12 parameters]; 
#             - **tum**: timestamp tx ty tz qx qy qz qw
#     """
#     with open(txt, 'w') as f:
#         for i in poses:
#             pose = poses[i]
#             if format == 'kitti':
#                 pose = pose.flatten()[:12]
#                 line_to_write = str(i) + " "
#                 line_to_write += " ".join([str(j) for j in pose])
#             elif format == 'tum':
#                 qw, qx, qy, qz = rot2quat(pose[:3, :3])
#                 tx, ty, tz = pose[:3, 3]
#                 line_to_write = " ".join([
#                                     str(i), 
#                                     str(tx), str(ty), str(tz),
#                                     str(qx), str(qy), str(qz), str(qw)]
#                                     )
#             f.writelines(line_to_write+"\n")
#     print("Trajectory saved.")


import evo
from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync
import evo.main_ape as main_ape
from evo.core.metrics import PoseRelation
import argparse
import glob
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# from eval_helper import rot2quat
# from eval_helper import mat2euler
# from eval_helper import euler2quat


# from evo.tools import file_interface





def main(args):
    # Project onto XY plane

    traj_est = np.loadtxt(args.est_path, dtype='str', delimiter=',', usecols=[4,5,6,7,8,9,10,11])
    # remove first line header info
    traj_est = traj_est[0:,:]
    traj_est[:,0] = np.char.strip(traj_est[:,0], '.png')
    traj_est = traj_est.astype('float')

    #trim rows to a certain sequence number
    
    abs_trim = None
    rel_trim = None
    
    if (( int(args.trim_to_frame or -1) > 0) or ( int(args.trim_to_frame_rel or -1) > 0) ):
        # print('first ' + str( int(args.trim_to_frame or -1) > 0) )
        # print('second ' + str( int(args.trim_to_frame_rel or -1) > 0) )
        if (args.trim_to_frame != None and int(args.trim_to_frame) > 0):
            trim_to = int(args.trim_to_frame) - int(args.skipped_frames)
            abs_trim = int(args.trim_to_frame)
            rel_trim = trim_to
        elif (args.trim_to_frame_rel != None and int(args.trim_to_frame_rel) > 0):
            trim_to = int(args.trim_to_frame_rel)
            abs_trim = trim_to + int(args.skipped_frames)
            rel_trim = trim_to
        traj_est = traj_est[:trim_to,:]

    #   FIRST TIMESTAMP IS 1 NOT 0! timestamp begins with 1 not 0


    # traj_est = np.loadtxt(args.est_path)

    # void VoNode::tracePoses(const SE3& T_w_f, const double timestamp)
    # {
    #   Quaterniond q(T_w_f.unit_quaternion());
    #   Vector3d p(T_w_f.translation());
    #   //trace_est_pose_.precision(15);
    #   trace_est_pose_.setf(std::ios::scientific, std::ios::floatfield );
    #   trace_est_pose_ << timestamp << " ";

    #   trace_est_pose_ATE.setf(std::ios::scientific, std::ios::floatfield );
    #   trace_est_pose_ATE << timestamp << " ";

    #   Matrix3d Rot_Matrix(T_w_f.rotation_matrix());
    #   trace_est_pose_ << Rot_Matrix(0,0) << " " << Rot_Matrix(0,1) << " " << Rot_Matrix(0,2) << " " << p.x()<<" "
    #                   << Rot_Matrix(1,0) << " " << Rot_Matrix(1,1) << " " << Rot_Matrix(1,2) << " " << p.y()<<" "
    #                   << Rot_Matrix(2,0) << " " << Rot_Matrix(2,1) << " " << Rot_Matrix(2,2) << " " << p.z()<< std::endl;


    #   trace_est_pose_ATE << p.x() << " " << p.y() << " " << p.z() << " "
    #                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    # }

    # Y is the height axis. turn it to: x,y,z,qx,qy,qz,qw  format, but remove timestamp

    # throw the first row (header / 0th row)
    # traj_est = traj_est[1:,:]
    # extract relevant columns : into this format : ts(sequential id). tx ty tz qx qy qz qw
    # traj_est = (traj_est)[:,[1, 7, 8, 9, 10, 11]]

    # Separtae timestamps from pose
    tstamps = traj_est[:,0]
    traj_est = (traj_est)[:,1:]

    # Add skipped frames to timestamp array
    # tstamps += int(args.skipped_frames) -1         # -1 is because the first timestamp is 1 rather than 0.

    print('tstamps: ' + str(tstamps[0]) + ' until ' + str(tstamps[-1]) )

    #rearrange the format of T00 T01 T02 T03 T10 T11 T12 T13 T20 T21 T22 T23 , into T00 T01 T02 T10 T11 T12 T20 T21 T22 T03 T13 T23
    # traj_est = (traj_est)[:,[0,1,2,4,5,6,8,9,10,3,7,11]]

    # ALREADY IN TUM FORMAT

    # np.savetxt('testdump.txt', traj_est, fmt= '%.8e')

    # new_traj_est = []
    # turn 12 parameters 3x4 matrix into 7dof
    # for i in range(len(traj_est[:,0])):
    #     qw, qx, qy, qz = rot2quat(traj_est[i,:9])
    #     tx, ty, tz = traj_est[i,9:]
    #     new_traj_est.append([tx, ty, tz, qx, qy, qz, qw])

    # new_traj_est = np.array(new_traj_est)

    # print(new_traj_est)
    # np.savetxt('testdump.txt', new_traj_est, fmt= '%.9e')
    # return

    # traj_est = new_traj_est


    # Turn image sequences timestamp format into actual timestamp
    # corr = {}
    # ts1 = np.loadtxt(os.path.join (args.base_path, 'correspondence.txt'), delimiter=' ', usecols=[0], dtype=int)
    # ts2 = np.loadtxt(os.path.join(args.base_path, 'correspondence.txt'), delimiter=' ', usecols=[1], dtype=str)
    # # ts_corr = np.vstack((ts1, ts2)).T
    # # ts_corr = np.loadtxt('oxford/10-29/correspondence.txt', delimiter=' ',)
    # ts2 = np.char.strip(ts2, ".png")
    # pairs = zip(ts1, ts2)
    # ts_corr = { t[0]:t[1] for t in pairs}
    # # print(ts_corr)
    # for i in range(len(tstamps)):
    #     tstamps[i] = ts_corr[tstamps[i]]
    # # print(tstamps)


    # for i in range(len(ts_corr[:,0])):
    #     print(ts_corr[i][1])
    #     corr[ts_corr[i][0]] = ts_corr[i][1][:-4]



        # [2,0,1,5,3,4,6]
    digits = int(args.timestamp_divide_by_digits)
    if digits == 6:
        div = 1e6
    elif digits == 9:
        div = 1e9
    else:
        div = 10**digits
    
    tstamps /= div
    # tstamps /= 1e6
    # traj_est = (traj_est)[:,[1,2,3,4,5,6,7]]
    # traj_est = (traj_est)[:,[2,0,1,5,3,4,6]]  # turn x y z into z x y , since the later (xyz)z is height, shift the y into the right side, therefore zxy
    # traj_est = (traj_est)[:,[2,0,1,5,3,4,6]]  # turn z x y into y z x , revolve it around multiple times until it's right
    traj_est[:,2:5] = 0 # Flatten the Estimated Trajectory
    traj_est = PoseTrajectory3D(
    positions_xyz=traj_est[:,:3],
    orientations_quat_wxyz=traj_est[:,[6,3,4,5]], #Changes the format to [[qw1,qx1,qy1,qz1], ...]
    timestamps=np.array(tstamps))

    traj_ref = np.loadtxt(args.gt_path, delimiter=',')
    traj_ref[:,2:5] = 0 # Flatten the ground truth
    gt_tstamps = np.loadtxt(args.gt_ts_path, delimiter=',')
    traj_ref = PoseTrajectory3D(
        positions_xyz=traj_ref[:,:3],
        orientations_quat_wxyz=traj_ref[:,[6,3,4,5]], #Changes the format to [[qw1,qx1,qy1,qz1], ...]
        timestamps=gt_tstamps)

    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.03)

    result = main_ape.ape(traj_ref, traj_est, est_name='traj_est',
    pose_relation=PoseRelation.translation_part, align=True, correct_scale=True) # Performs alignment and scaling
    

    print("Estimated Trajectory's result:")
    # print(result.__dict__['stats']['rmse'])
    print(result)
    res_array = []
    for (key, val) in result.__dict__['stats'].items():
        res_array.append(str(key) + ' : ' + str(val))
        pass


    #record arguments used in eval
    res_array.append('--------ARGS--------')
    for arg in vars(args):
        res_array.append(arg + ' : ' + str(getattr(args, arg)))

    if (abs_trim != None and rel_trim != None):
        res_array.append('Absolute trim frame number: ' + str(abs_trim))
        res_array.append('Relative trim frame number: ' + str(rel_trim))

    res_array = np.array(res_array)
    # print(res_array)
    # print(res_array.size)
    np.savetxt(args.save_file, res_array, fmt='%s')

    # To plot results and save results
    plt.figure(0, figsize=(16, 12), dpi=80)
    plt.xlabel('Northing (m)', fontsize=30)
    plt.ylabel('Easting (m)', fontsize=30)
    ax = plt.gca()
    ax.xaxis.offsetText.set_fontsize(24)
    ax.yaxis.offsetText.set_fontsize(24)
    plt.xticks(fontsize=24)
    plt.yticks(fontsize=24)
    plt.plot(traj_ref.positions_xyz[:,0], traj_ref.positions_xyz[:,1],color='g',label='GT')
    plt.plot(traj_est.positions_xyz[:,0], traj_est.positions_xyz[:,1],color='black',label='Est')
    plt.legend(loc='best',fontsize=30)
    plt.savefig(args.save_plot_path)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--base_path", help="base path to est output / correspondence")
    parser.add_argument("--est_path", help="path to est output")
    parser.add_argument("--skipped_frames", help="how many frames were skipped")
    parser.add_argument("--trim_to_frame", help="trim the data to a certain frame sequence number")
    parser.add_argument("--trim_to_frame_rel", help="trim the data to a certain frame sequence number relative to skipped frames")
    parser.add_argument("--gt_path", help="path to gt")
    parser.add_argument("--gt_ts_path", help="path to gt timestamps")
    parser.add_argument("--save_plot_path", help="path to save plot")
    parser.add_argument("--save_file", help="name of saved file")
    parser.add_argument("--timestamp_divide_by_digits", help="divide trajectory timestamp by how many digits? Compare gt timestamp and trajectory timestamp number of digits", const=6, nargs='?')

    args = parser.parse_args()

    # images_list = sorted(glob.glob(os.path.join(args.data_path, 'stereo/left_undistort/*.png')))
    # tstamps = [float(x.split('/')[-1][:-4])/1e6 for x in images_list][args.t0:args.t1]

    # Set it back to GT coordinate frame
    # Estimated Trajectory should be in the format of a 2D numpy array of: [[x1,y1,z1,qx1,qy1,qz1,qw1], [x2,y2,z2,qx2,qy2,qz2,qw2], ...]
    # traj_est = traj_est[:,[2,0,1,5,3,4,6]] # Use this line of code if the format of the data is in [[y1,z1,x1,qy1,qz1,qx1,qw1],...]


    main(args)