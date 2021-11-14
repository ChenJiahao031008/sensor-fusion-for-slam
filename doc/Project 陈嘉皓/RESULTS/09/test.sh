# evo_traj kitti laser_odom.txt --ref=ground_truth.txt --plot --plot_mode xy
# evo_traj kitti optimized.txt --ref=laser_odom.txt --plot --plot_mode xy
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xy -a
evo_ape kitti ground_truth.txt optimized.txt -r full --plot --plot_mode xy -a

# evo_ape kitti ground_truth.txt laser.txt -r full --plot --plot_mode xy
# evo_ape kitti ground_truth.txt fused.txt -r full --plot --plot_mode xy
