import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cv2
import time
from PIL import Image
import sys
import pathlib
import os
sys.dont_write_bytecode = True

current_folder_path = pathlib.Path(__file__).resolve().parent
data_folder_path = (current_folder_path / ".." / "data_folder").resolve()

num = 3
WINDOW_WIDTH = 420
WINDOW_HEIGHT = 330
left = -100
right = 320
up = 160
bottom = -170
high = 1300
low = 800

def pointcloud_process(pcd_matrix, flat):
    pcd_matrix2 = pcd_matrix[(pcd_matrix[:,0]>left)&(pcd_matrix[:,0]< right) & (pcd_matrix[:, 1]>bottom)&(pcd_matrix[:, 1]<up)&(pcd_matrix[:, 2]>low)&(pcd_matrix[:, 2]<high)]
    pcd_matrix3 = pcd_matrix2.copy()
    pcd_matrix2[:, 0] += -left
    pcd_matrix2[:, 1] += -bottom

    depth, matrix_image = xyz2matrix(pcd_matrix2)

    depth = depth - flat
    depth[depth<3] = 0
    depth[0:20] = 0
    depth[:, 400:420] = 0
    depth[:, 0:20] = 0
    # depth[depth>200] = 0

    return depth, matrix_image, pcd_matrix3

def PtoD(depth):
    MAX_DEPTH = depth.max()
    if MAX_DEPTH > 100:
        MAX_DEPTH = 100
    MIN_DEPTH = 0
    # MAX_DEPTH = 100

    for i, row in enumerate(depth):
        for j, z in enumerate(row):
            if z <= MAX_DEPTH:
                depth[i][j] = 255*(z/MAX_DEPTH)
            elif z > MAX_DEPTH:
                depth[i][j] = 255
    
    return depth, MAX_DEPTH

def xyz2matrix(matrix):
    depth = np.zeros((WINDOW_HEIGHT*num, WINDOW_WIDTH*num))
    matrix_image = np.zeros((WINDOW_HEIGHT*num, WINDOW_WIDTH*num))
    matrix[:, 0:2] *= num
    for i, xyz in enumerate(matrix):
        x = int(xyz[1])
        y = int(xyz[0])
        depth[x, y] = high - xyz[2]
        matrix_image[x, y] = i
    depth, matrix_image = max_pooling(depth, matrix_image)

    return depth, matrix_image

def max_pooling(depth, matrix_image):
    small_depth = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH))
    matrix_small_image = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH))
    for x in range(WINDOW_HEIGHT):
        for y in range(WINDOW_WIDTH):
            kernel = depth[x*3:x*3+3, y*3:y*3+3]
            max_index = np.argmax(kernel)
            q, mod = divmod(max_index, 3)
            small_depth[x][y] = kernel[q][mod]
            matrix_small_image[x][y] = matrix_image[x*3+q][y*3+mod]
    
    return small_depth, matrix_small_image
 
def main(pc):
    start = time.time()

    flatpath = data_folder_path / "csv" / "flat.csv"
    flat_list = pd.read_csv(flatpath, header=None).values
    pcd_matrix = np.asanyarray(pc)

    # t1 = time.time() - start
    # print(t1)

    depth, matrix_image, matrix = pointcloud_process(pcd_matrix, flat_list)

    # t2 = time.time() - start - t1
    # print(t2)

    depth, MAX_DEPTH = PtoD(depth)
    depth[depth < 0] = 0

    # t3 = time.time() - start - t2 - t1
    # print(t3)

    elapsed_time = time.time() - start#処理の終了時間を取得
    print("Converting PC to depth image is succeeded!")
    print("Run time costs to convert PC to depth is {}\n".format(elapsed_time))

    return depth, matrix_image, matrix, MAX_DEPTH

if __name__ == "__main__":
    try:
        mode = int(sys.argv[1])
    except IndexError:
        mode = 0   

    if mode:
        print("Photograph PointCloud by PhoXi")
        root_dir = os.environ['HOME']
        robot_dir = os.path.join(root_dir, "bin-picking-robot")
        ply_path = os.path.join(data_folder_path, "ply/out.ply")
        sys.path.append(str(robot_dir))      
        from bpbot.driver.phoxi import phoxi_client as pclt
        pxc = pclt.PhxClient(host="127.0.0.1:18300")
        pxc.triggerframe()
        pc = pxc.getpcd()
        pxc.saveply(ply_path) 

    import open3d as o3d
    filepath = data_folder_path / "ply" / "out.ply"
    pcd = o3d.io.read_point_cloud(str(filepath))
    pc = pcd.points

    # pcd = o3d.geometry.PointCloud()
    # pcd.points.append(pc)
    # o3d.visualization.draw_geometries([pcd])

    depth, _, _, _ = main(pc)

    plt.imshow(depth)
    plt.show()
