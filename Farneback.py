import cv2
import numpy as np
import math
import os
import glob
import open3d as o3d
from rembg import remove

    

def calculate_dense_optical_flow(prev_frame, current_frame):
    # Calculate dense optical flow using Farneback method
    flow = cv2.calcOpticalFlowFarneback(prev_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    return flow

def read_images(folder_path):
    print("Checkpoint 1")
    images = []
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.png')))  # Change the extension as needed
    for file_path in file_list:
        img = cv2.imread(file_path,cv2.IMREAD_GRAYSCALE)
        images.append(img)
    return images

def read_images_removebg(folder_path):
    print("Checkpoint 1")
    images = []
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.png')))  # Change the extension as needed
    for file_path in file_list:
        img = cv2.imread(file_path)
        output = remove(img)
        img = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
        images.append(img)
    return images

def calibrate_camera(images):
    # Assuming all images have the same size
    image_size = images[0].shape[:2]

    # Define chessboard pattern parameters (modify based on your calibration pattern)
    pattern_size = (8, 5)  # Number of inner corners in your calibration pattern
    obj_points = []  # 3D points in the real world
    img_points = []  # 2D points in the image plane

    # Generate calibration pattern coordinates
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    i= 1
    for img in images:
        i+=1
        print(i)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    # Perform camera calibration
    _, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    return camera_matrix, dist_coeffs

def get_depth_data(images,baseline,focal_length):
    print("Checkpoint 2")
    point_3d_not_rotated = []
    height, width = images[0].shape
    for i in range(len(images)):
        print("C",i)
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
    return point_3d_not_rotated

def rotation_of_axes_about_z(points_3d_not_rotated):
    print("Checkpoint 3")
    points_3d_rotated = []
    count = 0
    for i in points_3d_not_rotated:
        theta = 0.0872665*count   # 5 degrees
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
    return points_3d_rotated

def create_mesh(points_3d):
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

    return mesh

def visualize_mesh(mesh):
    # Visualize the mesh
    o3d.visualization.draw_geometries([mesh])



if __name__ == "__main__":
    baseline = 1
    focal_length = 3.50168727e+03
    # camera_matrix, dist_coeffs = calibrate_camera(calibcamera)
    images_folder = 'F:\OpenCV Workshop-2023\images6'
    images = read_images(images_folder)
    points_3d_not_rotated =get_depth_data(images,baseline,focal_length)
    points_3d_rotated = rotation_of_axes_about_z(points_3d_not_rotated)
    points_3d_rotated = np.array(points_3d_rotated)
    reconstructed_mesh = create_mesh(points_3d_rotated)

    # Perform mesh smoothing
    smoothed_mesh = reconstructed_mesh.filter_smooth_taubin(number_of_iterations=2)
    # smoothed_mesh.compute_triangle_normals()
    # Fill gaps in the mesh
    # filled_mesh = smoothed_mesh.fill_holes()
    visualize_mesh(smoothed_mesh)
    
    #Save as STL
    # o3d.io.write_triangle_mesh("bottle_mesh.stl", smoothed_mesh)
    # o3d.io.write_triangle_mesh("mesh1.stl", reconstructed_mesh)



#     cv2.arrowedLine(flow_visualization, (x, y), (int(x + dx), int(y + dy)), (255, 255, 255), 1)
# # Display the frames and optical flow visualization
# cv2.imshow("Current Frame", frame2)
# cv2.imshow("Optical Flow", flow_visualization)
# frame1 = cv2.resize(frame1, (800, 1024))
# cv2.imshow("p",frame1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
