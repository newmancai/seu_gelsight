#!/home/lab407/anaconda3/envs/grasp python
import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import pyrealsense2 as rs
import cv2
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image
import scipy.io as scio

import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data(data_dir):
    # load data
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    # print(depth)
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
    meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = meta['factor_depth']

    # generate cloud
    camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask1= (depth < 950) & (depth >500)
    # print(mask1)
    mask = (workspace_mask &mask1 )
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]
    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]
    index = []
    index_to_remove = 0
    # print(gg[:1])
    a = np.mat([[0.4606873,  0.2928247, -0.8378668,1.295676718906534],[ 0.8781856, -0.0135262,  0.4781287,-0.1974501057596964],[ 0.1286748, -0.9560705, -0.2633858,0.19624172000174564],[0,0,0,1]])
    # print(a)
    for i in gg:
        b = np.append(i.translation, 1)
        c = b.reshape(-1,1)
        d = a * c
        index_to_remove += 1
        if d[2] > -0.18:
            print(d)
            index.append(index_to_remove)
            break

    # print(c)
    # print(np.linalg.inv(a))  np.linalg.inv(a)

    #center_point = first_grasp.center  # 中心点
    #orientation = first_grasp.angle  # 姿态（以弧度表示）
    #print(center_point)
    #print(orientation)
    grippers = gg[index].to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])

def demo(data_dir):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir)
    gg = get_grasps(net, end_points)
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    vis_grasps(gg, cloud)

def input():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config object to configure the pipeline
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)

    # Start the pipeline
    pipeline.start(config)
    depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    print("Depth Scale (Factor Depth):", depth_scale)
    align = rs.align(rs.stream.color)  # Create align object for depth-color alignment

    try:
        while True:
            # Wait for a coherent pair of frames: color and depth
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            if not aligned_frames:
                continue  # If alignment fails, go back to the beginning of the loop

            color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not aligned_depth_frame:
                continue

            # Convert aligned_depth_frame and color_frame to numpy arrays
            aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Display the aligned depth image
            aligned_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image, alpha=0.3),
                                                       cv2.COLORMAP_JET)
            cv2.imshow("Aligned Depth colormap", aligned_depth_colormap)

            cv2.imshow("Aligned Depth Image", aligned_depth_image)
            cv2.imwrite('/home/lab407/pic/depth.png', aligned_depth_image)

            # Display the color image
            cv2.imshow("Color Image", color_image)
            cv2.imwrite('/home/lab407/pic/color.png', color_image)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the pipeline and close all windows
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__=='__main__':
    input()
    data_dir = ('/home/lab407/pic')
    #data_dir = ('/home/lab407/graspnet-baseline/doc/example_data')
    demo(data_dir)