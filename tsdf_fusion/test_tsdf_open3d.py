import os, glob, cv2
import open3d as o3d
import numpy as np

data_foler = "data/"

# create a directory to save captured images and robot poses
def create_data_directory(dir):
    dir = os.path.join(dir, data_foler)
    try:
        if not(os.path.isdir(dir)):
            os.makedirs(dir)
    except OSError as e:
        print("Can't make the directory: %s" % dir)
        raise
    return dir

data_directory = create_data_directory(os.path.dirname(os.path.realpath(__file__)))

if __name__ == "__main__":

    # Detect how many frames in data directory
    n_imgs = len(glob.glob1(data_directory, "*.jpg"))

    # Load camera intrinsics
    calib_file = os.path.join(data_directory, '../camera_params.yaml')
    calib_file = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    camera_params = calib_file.getNode("intrinsic").mat()
    camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(640, 480, camera_params[0,0], camera_params[1,1], camera_params[0,2], camera_params[1,2])

    volume = o3d.pipelines.integration.UniformTSDFVolume(
        length=10,
        resolution=512,
        sdf_trunc=0.1,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        # origin=np.array([772, -8, 420])
    )

    # Loop through RGB-D images and fuse them together
    for i in range(n_imgs):
        print("Fusing frame %d/%d"%(i+1, n_imgs))

        # Read RGB-D image and camera pose
        color_image = o3d.io.read_image("data/frame-%06d.color.jpg"%(i))
        depth_image = o3d.io.read_image("data/frame-%06d.depth.png"%(i))
        cam_pose = np.loadtxt("data/frame-%06d.pose.txt"%(i))
        cam_pose[:3,3] =cam_pose[:3,3] * 0.001
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, depth_trunc=5.0, convert_rgb_to_intensity=False)
        volume.integrate(
            rgbd,
            camera_intrinsics,
            np.linalg.inv(cam_pose),
        )

    print("Extract triangle mesh")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])

    print("Extract voxel-aligned debugging point cloud")
    voxel_pcd = volume.extract_voxel_point_cloud()
    o3d.visualization.draw_geometries([voxel_pcd])

    print("Extract voxel-aligned debugging voxel grid")
    voxel_grid = volume.extract_voxel_grid()
    o3d.visualization.draw_geometries([voxel_grid])

    print("Extract point cloud")
    pcd = volume.extract_point_cloud()
    o3d.visualization.draw_geometries([pcd])