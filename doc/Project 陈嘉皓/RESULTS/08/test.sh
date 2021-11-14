evo_traj kitti fused.txt --ref=laser.txt --plot --plot_mode xy
evo_ape kitti ground_truth.txt laser.txt -r full --plot --plot_mode xy
evo_ape kitti ground_truth.txt fused.txt -r full --plot --plot_mode xy
