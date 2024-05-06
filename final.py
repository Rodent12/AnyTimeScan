import calibration
import backend
import parse
import main
import numpy as np


if __name__ == "__main__":

    #folder paths
    calib_folder = "./Chessboard9x6"
    images_folder = "./images9"

    # Download from Cloudinary
    # Get degrees and project_name
    # project_name,number_of_images_in_one_rotation = parse.parser()

    # backend.get_image_from_cloudinary(images_folder,project_name)
    # backend.get_image_from_cloudinary(calib_folder,project_name_1)

    

    #constants
    baseline = 50  
    sensor_size_mm = 7.7 
    error = 5.25 #5.5
    # degrees = 360/int(number_of_images_in_one_rotation)
    degrees = 10
    stl_file_path = "mesh2.stl"

    # Calibration
    # calibcamera = main.read_images(calib_folder)
    # camera_matrix, dist_coeffs = calibration.calibrate_camera(calibcamera)
    # focal_length_pixels = camera_matrix[0][0]/100
    focal_length_pixels = 3.50168727e+03

    # images = main.read_images_removebg(images_folder)
    images = main.read_images(images_folder)
    height,width = images[0].shape
    sensor_resolution_pixels = np.sqrt(width**2 + height**2)    
    pixel_to_millimeter = sensor_size_mm/sensor_resolution_pixels
    focal_length = focal_length_pixels*pixel_to_millimeter   #35mm smartphone camera 
    

    pointcloud = main.process(images,baseline,focal_length,pixel_to_millimeter,error,degrees)
    
    print(len(pointcloud))
    # main.visualize_mesh(main.create_mesh(pointcloud))
    main.save_mesh(pointcloud,stl_file_path)

    # Upload to Cloudinary
    # result = backend.upload_to_cloudinary(stl_file_path)
    # print(result)

'''
       focal_length = camera_matrix[0][0]
       focal_length = camera_matrix[0][0]/100
       focal_length_in_pixels = camera_matrix[0][0]/100
       focal_length_in mm = focal_length_in_pixels*pixel_to_millimeter    {This has been the one which is giving the best results}  
'''