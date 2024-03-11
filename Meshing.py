import cv2
import numpy as np
import open3d as o3d

# Function to read images from a folder
def read_images(folder_path):
    images = []
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.jpeg')))  # Change the extension as needed
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
def sfm_reconstruction(images, camera_matrix, dist_coeffs):
    # Feature detector and matcher
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # 3D point cloud and 2D image points
    points_3d = []
    points_2d = []

    # Use the first image as the reference
    ref_img = images[0]
    ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
    ref_kp, ref_des = orb.detectAndCompute(ref_gray, None)
    
    for i in range(len(images) - 1):
        for j in range(i + 1, len(images)):
            ref_img = images[i]
            curr_img = images[j]

            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)

            try:
                ref_kp, ref_des = orb.detectAndCompute(ref_gray, None)
                curr_kp, curr_des = orb.detectAndCompute(curr_gray, None)

                # Match features between reference and current images
                matches = bf.match(ref_des, curr_des)
                matches = sorted(matches, key=lambda x: x.distance)

                # Get matched keypoints
                ref_pts = np.float32([ref_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                curr_pts = np.float32([curr_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                # Get Rotation matrix and Translation vector
                R, t = estimate_camera_pose(ref_pts, curr_pts, camera_matrix)

                if R is not None and t is not None:
                    U = np.eye(3, 4)
                    T = np.eye(3, 4)

                    for p in range(3):
                        for q in range(3):
                            U[p][q] = R[p][q]
                            if j == 2:
                                T[p][q] = R[p][q] + (0.04*j)%1 # considering 10 degrees rotation+0.055
                                continue
                            T[p][q] = R[p][q]

                    for p in range(3):
                        U[p][3] = t[p]
                        T[p][3] = t[p]

                    # Get Projection Matrix
                    P1 = np.dot(camera_matrix, U)
                    P2 = np.dot(camera_matrix, T)

                    # Triangulate 3D points using the matched keypoints and camera matrices
                    points_4d = cv2.triangulatePoints(P1, P2, ref_pts, curr_pts)
                    points_4d /= points_4d[3, :]

                    # Add the triangulated points to the 3D point cloud
                    points_3d.extend(points_4d[:3, :].T)
                    # Add corresponding 2D points to the list
                    points_2d.extend(curr_pts.reshape(-1, 2))

            except cv2.error as e:
                print(f"Error in sfm_reconstruction for images {i} and {j}")

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
    images_folder = 'F:\OpenCV Workshop-2023\images4'

    # Read images from the folder
    # calibcamera = read_images(calibcamera_folder)
    images = read_images(images_folder)

    # Camera calibration
    # camera_matrix, dist_coeffs = calibrate_camera(calibcamera)

    camera_matrix = [[3.50168727e+03,0.00000000e+00,1.70020546e+03],[0.00000000e+00,3.51462760e+03,2.34410132e+03],[0.00000000e+00,0.00000000e+00,1.00000000e+00]]
    camera_matrix = np.array(camera_matrix)
    dist_coeffs = []

    # SFM-based 3D reconstruction
    points_3d, points_2d = sfm_reconstruction(images, camera_matrix, dist_coeffs)

    print(points_3d.shape)
    
    reconstructed_mesh = create_mesh(points_3d)

    # Visualize the mesh
    visualize_mesh(reconstructed_mesh)

    #Save as STL
    o3d.io.write_triangle_mesh("mesh1.stl", reconstructed_mesh)

    # print(points_3d)
    # print(points_2d)

    # Further processing or visualization can be done with the obtained 3D points
    # For example, you can use a library like Mayavi or matplotlib for visualization.
