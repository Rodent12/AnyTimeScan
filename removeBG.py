from rembg import remove
import cv2
import glob
import os

def removeBG(folder_path):
    count = 0
    file_list = sorted(glob.glob(os.path.join(folder_path, '*.jpg')))  # Change the extension as needed
    for file_path in file_list:
        count +=1
        output_path = folder_path +'\pole'+ str(count) + '.jpg'
        img = cv2.imread(file_path)
        output = remove(img)
        cv2.imwrite(output_path, output)
        


removeBG("output_frames")

