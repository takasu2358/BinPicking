import os
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

start = time.time()
print("***************Start processes!***************")

###########################パスの取得###################################
root_dir = os.environ['HOME']
download_dir = os.path.join(root_dir, "ダウンロード")
grasp_dir = os.path.join(download_dir, "binpicking_git")
data_dir = os.path.join(download_dir, "data_folder")
ply_path = os.path.join(data_dir, "ply/out1.ply")
motionfile_path = os.path.join(data_dir, "motionfile/motionfile.dat")
sys.path.append(str(grasp_dir))
########################################################################

pcd = o3d.io.read_point_cloud(ply_path)
pc = pcd.points

############################深度画像への変換############################
import depth_csv
depth, pcd_matrix_index_image, pcd_matrix, MAX_DEPTH = depth_csv.main(pc)
########################################################################

############################把持位置の取得##############################
import grasp_position
grasp_positions = grasp_position.main(depth, pcd_matrix_index_image, pcd_matrix, MAX_DEPTH)
########################################################################

###########################Motionfileの作成#############################
import generate_motionfile
goal_position = [0, 0.55, 0.6]
motionfile_success = generate_motionfile.GenerateMotionfile(motionfile_path, grasp_positions,  goal_position, action = 0)
########################################################################

all_elapesd_time = time.time() - start
print("Time costs for all process is ", all_elapesd_time)