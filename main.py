import cv2
import numpy as np
import math
import os
import glob
import open3d as o3d
import backend
import calibration
from rembg import remove


def read_images(folder_path):
    print("Reading images")
    images = []
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.png')))  # Change the extension as needed
    for file_path in file_list:
        img = cv2.imread(file_path,cv2.IMREAD_GRAYSCALE)
        images.append(img)
    print("Finished!")
    return images
    
def read_images_removebg(folder_path):
    print("Reading images")
    images = []
    extensions = ['png', 'jpg', 'jpeg']
    file_list = sorted([file for ext in extensions for file in glob.glob(os.path.join(folder_path, f'*.{ext}'))]) 
    for file_path in file_list:
        img = cv2.imread(file_path)
        output = remove(img)
        img = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
        images.append(img)
    print("Finished!")
    return images

def calculate_dense_optical_flow(prev_frame, current_frame):
    flow = cv2.calcOpticalFlowFarneback(prev_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    return flow

def get_depth_data(images,baseline,focal_length):
    print("Starting process to get depth data :)")
    point_3d_not_rotated = []
    height, width = images[0].shape
    for i in range(len(images)):
        print("Image",i+1)
        frame1 = images[i]
        if(i == len(images)-1):
            frame2 = images[0]
        else:
            frame2 = images[i+1]

        optical_flow = calculate_dense_optical_flow(frame1, frame2)
        h, w = optical_flow.shape[:2]
        step = 2
        depthData = []
        for y in range(0, h, step):
            for x in range(0, w, step):
                dx, dy = optical_flow[y, x]
                p = math.sqrt(dx**2 + dy**2)
                if(p!=0):
                    z = baseline*focal_length / p
                    pixel_to_meter = 1000
                    resolution = height/width
                    if(z<5000):
                        depthData.append([[x,y*resolution,z/pixel_to_meter]])
        # for i in range(len(depthData)):
        #     cv2.circle(frame1,(int(depthData[i][0][0]),int(depthData[i][0][1])), 5, (255, 255, 255), 5)
        # cv2.imshow("p",frame1)
        # cv2.waitKey(0)
        point_3d_not_rotated.append(depthData)
    print("Process Finished!")
    return point_3d_not_rotated

def rotation_of_axes_about_z(points_3d_not_rotated,degrees):
    print("Rotating about z axis")
    points_3d_rotated = []
    count = 0
    for i in points_3d_not_rotated:
        theta = math.radians(degrees)*count   # 5 degrees
        cos = math.cos(theta)
        sin = math.sin(theta)
        for j in i:
            x = j[0][0]
            y = j[0][1]
            z = j[0][2]
            x_rotated = x*cos + z*sin
            z_rotated = -x*sin + z*cos
            points_3d_rotated.append([x_rotated,y,z_rotated])
        count+=1
    print("Process Finished")
    return points_3d_rotated

def create_mesh(points_3d):
    print("Creating Mesh")
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)

    # Estimate normals for the point cloud
    pcd.estimate_normals()

    # Create a mesh using Poisson surface reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

    # Remove low-density triangles
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    print("Finalizing!!")

    return mesh

def visualize_mesh(mesh):
    # Visualize the mesh
    o3d.visualization.draw_geometries([mesh])


if __name__ == "__main__":

    #Contants
    
    images_folder = './images6'

    # This function downloads the images from cloudinary to the folder
    # backend.get_image_from_cloudinary(images_folder)
      
    # Baseline = distance from the camera to the object in m
    baseline = 1       
    focal_length = 3.50168727e+03

    # Calibration for camera 
    # calib_folder = './Chessboard9x6'
    # calibcamera = read_images(calib_folder)
    # camera_matrix, dist_coeffs = calibration.calibrate_camera(calibcamera)
    # focal_length = camera_matrix[0][0]


    # Main process start here .... 
    images = read_images(images_folder)
    points_3d_rotated = np.array(rotation_of_axes_about_z(get_depth_data(images,baseline,focal_length),5))
    reconstructed_mesh = create_mesh(points_3d_rotated)

    visualize_mesh(reconstructed_mesh)
    
    # Save as STL
    # o3d.io.write_triangle_mesh("bottle_mesh.stl", smoothed_mesh)
    # o3d.io.write_triangle_mesh("mesh1.stl", reconstructed_mesh)

    # Upload to Cloudinary
    # file_path = 'LinuxLogo.jpg'
    # result = backend.upload_to_cloudinary(file_path)
    # print(result)



