import numpy as np
import cv2 as cv
import glob
import json
import math
import os

from quartonian_handler import QuartonianHandler

def get_tranformation_matrix_from_transform(trans: list, rotation_matrix) -> list:
        
    transformation_matrix = np.identity(4)

    for i in range(3):
        for j in range(3):
            transformation_matrix[i][j] = rotation_matrix[i][j]

        transformation_matrix[i][3] = trans[i]
    
    return transformation_matrix

rows = 8
columns = 8

square_length_m = 0.0255

file_path = "/home/psxls7/catkin_ws/src/robotic_view_capture/view_capture_scripts/logs/robot2_checkerboard_calibration_4/images/rgb/"
log_path = "logs"
show_imgs = False

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros(((rows-1)*(rows-1), 3), np.float32)

objp[:,:2] = np.mgrid[0:(rows-1),0:(rows-1)].T.reshape(-1,2)

objp = objp[:,:] * square_length_m

objpoints = [] 
imgpoints = [] 
images = glob.glob(file_path + '*.png')

images.sort()

print("Starting calibration")

if len(images) == 0:
    raise Exception("No calibration images found, please ensure that the file path is correct")

if len(images) < 10:
    raise Exception("Please include at least 10 images or more for proper calibration")

invalid_imgs = []
file_names = []

for i, path_name in enumerate(images):
    file_name = path_name[len(file_path):]

    print("Loading image: ", file_name)
    
    img = cv.imread(path_name)
    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

    if show_imgs:
        cv.imshow(file_name, gray)
        cv.waitKey(0)
    
    ret, corners = cv.findChessboardCorners(gray, (rows-1, columns-1), None)

    print("Board found" if ret else "Board not found!")
    
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, (rows-1, columns-1), corners2, ret)

        if show_imgs:
            cv.imshow(file_name + " with corners", img)
            cv.waitKey(0)

        file_names.append(file_name)
        
    else:
        invalid_imgs.append(file_name)

    print()

cv.destroyAllWindows()

print(len(invalid_imgs), "/", len(images), " images were invalid", sep="")

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

q = QuartonianHandler()

print("Camera successfully calibrated" if ret else "Failed to calibrate camera")

if ret:
    calibration_data = {
        "mtx": np.around(mtx, decimals=14).tolist(),
        "dist": np.around(dist, decimals=14).tolist(), 
    }

    with open(os.path.join(log_path, "camera_cal.json"), "w") as calibration_file:
        json.dump(calibration_data, calibration_file, indent=4)

    transforms = {"frames":[]}

    for i, euler_rotation in enumerate(rvecs):
        rotation_matrix = q.convert_euler_to_rotation_matrix(euler_rotation)

        # TODO: CHANGE 
        transform = get_tranformation_matrix_from_transform(tvecs[i], rotation_matrix)

        view_data = {"file_path": file_names[i], 
                     "transform_matrix": np.around(transform, decimals=14).tolist()}

        transforms["frames"].append(view_data)
    
    with open(os.path.join(log_path, "camera_transforms.json"), "w") as transform_file:
        json.dump(transforms, transform_file, indent=4)

    """with open(os.path.join(log_path, "camera_rvecs.json"), "w") as rvecs_file:
        rvecs_dict = {}
        for i, vec in enumerate(rvecs):
            rvecs_dict[file_names[i]] = vec.tolist()
        json.dump(rvecs_dict, rvecs_file, indent=4)"""

    print("Finished camera calibration")
