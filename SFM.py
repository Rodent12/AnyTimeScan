import cv2
import numpy as np
import open3d as o3d
import math

# Function to read images from a folder
def read_images(folder_path):
    images = []
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.jpg')))  # Change the extension as needed
    for file_path in file_list:
        img = cv2.imread(file_path)
        images.append(img)
    return images

def estimate_camera_pose(pts1, pts2, K):
    # Essential Matrix Calculation
    E, _ = cv2.findEssentialMat(pts1, pts2, K)

    # Recovering the pose from the essential matrix
    _, R, t, _ = cv2.recoverPose(E, pts1, pts2, K)

    return R, t

# Function to perform camera calibration
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

# Function to perform SFM-based 3D reconstruction
def sfm_reconstruction(images, camera_matrix):
    # Feature detector and matcher
    # sift = cv2.SIFT_create()
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # 3D point cloud and 2D image points
    points_3d = []
    points_2d = []

    # Use the first image as the reference
    ref_img = images[0]
    ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
    ref_kp, ref_des = orb.detectAndCompute(ref_gray, None)

    for i in range(1, len(images)-1):
        # Process each subsequent image
        try:
            curr_img = images[i]
            curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
            curr_kp, curr_des = orb.detectAndCompute(curr_gray, None)

            next_img = images[i+1]
            next_gray = cv2.cvtColor(next_img, cv2.COLOR_BGR2GRAY)
            next_kp, next_des = orb.detectAndCompute(next_gray, None)

            # Match features between reference and current images
            matches = bf.match(ref_des, curr_des)
            matches = sorted(matches, key=lambda x: x.distance)

            # Match features between current and next images
            next_matches = bf.match(curr_des, next_des)
            next_matches = sorted(next_matches, key=lambda x: x.distance)

            # Get matched keypoints between reference and current images
            ref_pts = np.float32([ref_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            curr_pts1 = np.float32([curr_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            # Get matched keypoints between current and next images
            curr_pts2 = np.float32([curr_kp[m.trainIdx].pt for m in next_matches]).reshape(-1, 1, 2)
            next_pts = np.float32([next_kp[m.queryIdx].pt for m in next_matches]).reshape(-1, 1, 2)

            print("Hello", i)
            # Get Rotation matrix and Translation vector
            R1, t1 = estimate_camera_pose(ref_pts, curr_pts1, camera_matrix)
            R2, t2 = estimate_camera_pose(curr_pts2, next_pts, camera_matrix)
            
            U = np.eye(3,4)
            T = np.eye(3,4)
        
            for i in range(3):                      
                for j in range(3):
                    U[i][j] = R1[i][j]
                    T[i][j] = R2[i][j]
                U[i][3] = t1[i]
                T[i][3] = t2[i]

            #Get Projection Matrix
            P1 = np.dot(camera_matrix,U)
            P2 = np.dot(camera_matrix,T)

            # Triangulate 3D points using the matched keypoints and camera matrices
            #points_4d = cv2.triangulatePoints(projection_matrix1,projection_matrix2,referencePts,currentPts)
            
            points_4d = cv2.triangulatePoints(P1, P2, ref_pts, curr_pts1)
            points_4d /= points_4d[3, :]
            
            # # Add the triangulated points to the 3D point cloud
            points_3d.extend(points_4d[:3, :].T)
            
            # Add corresponding 2D points to the list
            points_2d.extend(curr_pts1.reshape(-1, 2))
            

            # Update reference image and keypoints for the next iteration
            ref_img = curr_img
            ref_gray = curr_gray
            ref_kp, ref_des = curr_kp, curr_des
        except:
            print("Error in feature matching for images")

    # Convert lists to NumPy arrays
    points_3d = np.array(points_3d)
    points_2d = np.array(points_2d)

    return points_3d, points_2d

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




# Main functions    
if __name__ == "__main__":
    import os
    import glob

    # Path to the folder containing images
    # calibcamera_folder = 'F:\OpenCV Workshop-2023\Chessboard9x6'
    images_folder = 'F:\OpenCV Workshop-2023\images3'

    # Read images from the folder
    # calibcamera = read_images(calibcamera_folder)
    images = read_images(images_folder)

    # Camera calibration
    # camera_matrix, dist_coeffs = calibrate_camera(calibcamera)

    camera_matrix = [[3.50168727e+03,0.00000000e+00,1.70020546e+03],[0.00000000e+00,3.51462760e+03,2.34410132e+03],[0.00000000e+00,0.00000000e+00,1.00000000e+00]]
    camera_matrix = np.array(camera_matrix)

    # SFM-based 3D reconstruction
    points_3d, points_2d = sfm_reconstruction(images, camera_matrix)

    # Construct the mesh
    reconstructed_mesh = create_mesh(points_3d)

    # Visualize the mesh
    visualize_mesh(reconstructed_mesh)

