import cv2
import numpy as np
import math
import os
import glob
import open3d as o3d
from scipy.stats import mode
from rembg import remove
import numpy as np


def filter_3d_coordinates(points, error):
    # Calculate the mode for each dimension
    mode_z = mode(points[:, 2]).mode

    # Filter points within the specified range of the mode
    filtered_points = []
    for point in points:
        x, y, z = point
        if abs(z - mode_z) <= error:
            filtered_points.append(point)

    return np.array(filtered_points)

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
    cnt = 1
    for file_path in file_list:
        print("Image",cnt)
        img = cv2.imread(file_path)
        # img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        output = remove(img)
        img = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
        images.append(output)
        cnt+=1
    print("Finished!")
    return images

def calculate_dense_optical_flow(prev_frame, current_frame):
    flow = cv2.calcOpticalFlowFarneback(prev_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    return flow

def focal_length_in_mm(focal_length_pixels, sensor_size_mm, sensor_resolution_pixels):
    return (focal_length_pixels * sensor_size_mm) / sensor_resolution_pixels

def get_depth_data(images,baseline,focal_length,pixel_to_millimeter):
    print("Starting process to get depth data :)")
    point_3d_not_rotated = []
    for i in range(len(images)):
        print("Image",i+1)
        frame1 = images[i]
        if(i == len(images)-1):
            frame2 = images[0]
        else:
            frame2 = images[i+1]

        optical_flow = calculate_dense_optical_flow(frame1, frame2)  

        h, w = optical_flow.shape[:2]
        step = 10
        depthData = []
        for y in range(0, h, step):
            for x in range(0, w, step):
                dx, dy = optical_flow[y, x]
                p = math.sqrt(dx**2 + dy**2)
                if(p!=0):
                    z = baseline*focal_length / p*pixel_to_millimeter   # Triangulation
                    if(z <= 1000):
                        depthData.append([[x*pixel_to_millimeter,y*pixel_to_millimeter,z]])

        point_3d_not_rotated.append(depthData)
    print("Process Finished!")
    return point_3d_not_rotated

def rotation_of_axes_about_y(points_3d_not_rotated,degrees):
    print("Rotating about y axis")
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
    points_3d_rotated = np.array(points_3d_rotated)
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

def process(images,baseline,focal_length,pixel_to_millimeter,error,degrees):
    po = get_depth_data(images,baseline,focal_length,pixel_to_millimeter)
    coordinates = []
    
    for i in po:
        for j in i:
            x = j[0][0]
            y = j[0][1]
            z = j[0][2] 
            coordinates.append([x,y,z])
    
    # Testing in 2D 
    # plot_2d_points(np.array(coordinates))
    coordinates = np.array(coordinates)

    
    # change origin 
    average_z = np.mean(coordinates[:, 2])

    for point in coordinates:
        point[2] -= average_z
    

    
    corrected_coordinates = filter_3d_coordinates(coordinates,error)
    
    count=0
    new=[] #filtered in po shape
    
    mode_z = mode(coordinates[:, 2]).mode
    for i in po:
        part=[]
        for j in i:
            z = coordinates[count][2]
            if abs(z - mode_z) <= error:
                part.append([[j[0][0]*1.2,j[0][1]*5,z]])      
            j[0][2] = coordinates[count][2]
            count+=1
        new.append(part)

    
    # draw about origin
    average_z = np.mean(corrected_coordinates[:, 2])             

    for i in new:
        for j in i:
            j[0][2] -= average_z
    

    points_rotated = rotation_of_axes_about_y(new,degrees)
    return points_rotated
'''
def save_mesh(pointcloud,file_path):
    #tessalation 
    reconstructed_mesh = create_mesh(pointcloud)         
    reconstructed_mesh.compute_vertex_normals()
    visualize_mesh(reconstructed_mesh)               

    
    # Save as STL
    # o3d.io.write_triangle_mesh("bottle_mesh.stl", smoothed_mesh)
    o3d.io.write_triangle_mesh(file_path, reconstructed_mesh)
'''


def save_mesh(pointcloud, file_path, smooth=True, solidity=True):
    # Create mesh
    reconstructed_mesh = create_mesh(pointcloud)
    
    # Compute vertex normals
    # reconstructed_mesh.compute_vertex_normals()
    
    # Visualize mesh before modifications
    # visualize_mesh(reconstructed_mesh)
    
    # Smoothing the mesh if requested
    if smooth:
        reconstructed_mesh = reconstructed_mesh.filter_smooth_laplacian(100)
        
    #     # Visualize mesh after smoothing
    #     # visualize_mesh(reconstructed_mesh)
    reconstructed_mesh.compute_vertex_normals()
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = reconstructed_mesh.vertices
    # point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=20000, max_nn=1000))

    # # Solidifying the mesh if requested
    # if solidity:
    #     reconstructed_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud,
    #                                                                                           o3d.utility.DoubleVector([0.02, 0.04]))
                                                                                              
    #     # Visualize mesh after solidifying
    #     visualize_mesh(reconstructed_mesh)
    # visualize_mesh(reconstructed_mesh)
    # Save the modified mesh
    o3d.io.write_triangle_mesh(file_path, reconstructed_mesh)


    
    



