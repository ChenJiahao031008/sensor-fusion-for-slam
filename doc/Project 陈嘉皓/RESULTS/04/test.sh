# evo_traj kitti optimized.txt --ref=laser_odom.txt --plot --plot_mode xy
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xy -a
evo_ape kitti ground_truth.txt optimized.txt -r full --plot --plot_mode xy -a
