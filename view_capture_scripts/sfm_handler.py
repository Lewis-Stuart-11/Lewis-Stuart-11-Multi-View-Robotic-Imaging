import os
import subprocess
import configargparse
import json
import numpy as np
import time
import sqlite3 as sq
import shutil
from math import sqrt

from abc import ABC, abstractmethod

class SfMHandler(ABC):

    def __init__(self):
        self.img_name_pair_path = None
        self.keys_for_file_names = None
        self.img_dir = None

    @abstractmethod
    def convert_transform_to_sfm_pose(transform: list):
        pass

    @abstractmethod
    def convert_sfm_pose_to_nerf(rot: list, trans: list):
        pass
    
    @abstractmethod
    def convert_and_add_transforms_and_cam(self, transform_frames: list, camera_info = {}):
        pass

    @abstractmethod
    def compute_features(self, feature_mode: str="HIGH"):
        pass

    @abstractmethod
    def generate_img_name_matches_linear(self):
        pass

    @abstractmethod
    def compute_feature_matches(self):
        pass

    @abstractmethod
    def perform_structure_from_motion(self):
        pass

    def set_img_name_pair_path(self, new_img_name_pairs_path):
        self.img_name_pair_path = new_img_name_pairs_path

    def generate_img_name_matches_linear(self, img_pair_range=2):
        if self.keys_for_file_names is None or self.keys_for_file_names == {}:
            file_names_ordered = [f for f in os.listdir(self.img_dir) if os.path.isfile(os.path.join(self.img_dir, f)) and f.endswith(".png")]
        else:
            file_names_ordered = list(self.keys_for_file_names.keys())

        file_names_ordered.sort()

        with open(self.img_name_pair_path, "w") as img_pair_file:
            for img_name in file_names_ordered:

                file_name_idx = file_names_ordered.index(img_name)

                for i in range(-img_pair_range, img_pair_range+1):
                    if i == 0:
                        continue
                    
                    adjacent_file_name = file_names_ordered[(file_name_idx + i) % len(file_names_ordered)]
                    
                    img_pair_file.write(str(img_name) + " " + str(adjacent_file_name) + "\n")

    def generate_img_key_matches_adjacent(self, img_adjacencies):
        with open(self.img_name_pair_path, "w") as img_pair_file:
              for current_img, adjacent_imgs in img_adjacencies.items():
                   for adjacent_img in adjacent_imgs:
                        img_pair_file.write(str(current_img) + " " + str(adjacent_img) + "\n")


class Colmap_Handler(SfMHandler):

    def __init__(self, experiment_name, log_dir, img_dir="images/rgb", mask_dir=None):

        self.colmap_binary = "colmap"
        
        self.img_dir = img_dir
        self.mask_dir = mask_dir

        # On Windows, if FFmpeg isn't found, try automatically downloading it from the internet
        if os.name == "nt" and os.system(f"where {self.colmap_binary} >nul 2>nul") != 0:
            colmap_glob = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "external", "colmap", "*", "COLMAP.bat")
            candidates = glob(colmap_glob)
            if not candidates:
                print("COLMAP not found. Attempting to download COLMAP from the internet.")
                err = os.system((os.path.join(log_dir, "download_colmap.bat")))
                if err:
                    print("UNABLE TO INSTALL COLMAP")
                    return
                candidates = glob(colmap_glob)
            if candidates:
                self.colmap_binary = candidates[0]
        
        self.experiment_dir = os.path.join(log_dir, experiment_name)

        self.num_adjust_iters = 4
        self.adjust_iter = 0

        self.setup_directory()

        self.keys_for_file_names = {}

    def setup_colmap_subdirs(self):
        self.sfm_data_dir = os.path.join(self.experiment_dir, f"colmap_{self.adjust_iter}")
        self.sparse_dir = os.path.join(self.sfm_data_dir , "sparse")
        self.dense_dir = os.path.join(self.sfm_data_dir , "dense")
        self.adjusted_dir = os.path.join(self.sfm_data_dir , "adjusted", "bin")
        self.adjusted_txt_dir = os.path.join(self.sfm_data_dir , "adjusted", "txt")
        self.reconstructed_dir = os.path.join(self.sfm_data_dir , "reconstructed")
        self.db = os.path.join(self.sfm_data_dir, "colmap.db")

        if not os.path.exists(self.sfm_data_dir):
            os.mkdir(self.sfm_data_dir)

        if not os.path.exists(self.sparse_dir):
            os.mkdir(self.sparse_dir)

        if not os.path.exists(self.dense_dir):
            os.mkdir(self.dense_dir)

        if not os.path.exists(os.path.join(self.sfm_data_dir , "adjusted")):
            os.mkdir(os.path.join(self.sfm_data_dir , "adjusted"))

        if not os.path.exists(self.adjusted_dir):
            os.mkdir(self.adjusted_dir)
        
        if not os.path.exists(self.adjusted_txt_dir):
            os.mkdir(self.adjusted_txt_dir)

        if not os.path.exists(self.reconstructed_dir):
            os.mkdir(self.reconstructed_dir)

        self.img_name_pair_path = os.path.join(self.sfm_data_dir, "imageNamePairs.txt")

    def setup_directory(self):
        self.img_dir = os.path.join(self.experiment_dir, self.img_dir)
        self.mask_dir = os.path.join(self.experiment_dir, self.mask_dir) if self.mask_dir is not None else None
        
        self.setup_colmap_subdirs()

        self.conn = sq.connect(self.db, detect_types=sq.PARSE_DECLTYPES | sq.PARSE_COLNAMES)
        self.cur = self.conn.cursor()

    def qvec2rotmat(qvec):
        return np.array([
		[
			1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
			2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
			2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]
		], [
			2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
			1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
			2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]
		], [
			2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
			2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
			1 - 2 * qvec[1]**2 - 2 * qvec[2]**2
		]
	])


    def rotation_matrix_to_quartonian_safe(R):
        tr = R[0][0] + R[1][1] + R[2][2]

        if (tr > 0):
                S = sqrt(tr+1.0) * 2
                qw = 0.25 * S
                qx = (R[2][1] - R[1][2]) / S
                qy = (R[0][2] - R[2][0]) / S
                qz = (R[1][0] - R[0][1]) / S

        elif ((R[0][0] > R[1][1])&(R[0][0] > R[2][2])):
                S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
                qw = (R[2][1] - R[1][2]) / S
                qx = 0.25 * S
                qy = (R[0][1] + R[1][0]) / S
                qz = (R[0][2] + R[2][0]) / S 

        elif R[1][1] > R[2][2]:
                S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
                qw = (R[0][2] - R[2][0]) / S
                qx = (R[0][1] + R[1][0]) / S
                qy = 0.25 * S
                qz = (R[1][2] + R[2][1]) / S

        else:
                S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
                qw = (R[1][0] - R[0][1]) / S
                qx = (R[0][2] + R[2][0]) / S
                qy = (R[1][2] + R[2][1]) / S
                qz = 0.25 * S

        return [qx, qy, qz, qw]

    def convert_transform_to_sfm_pose(self, transform):
        flip_mat = np.array([
                                [1, 0, 0, 0],
                                [0, -1, 0, 0],
                                [0, 0, -1, 0],
                                [0, 0, 0, 1]
                           ])

        c2w = np.matmul(transform, flip_mat)

        c2w_back = np.linalg.inv(c2w)

        rotation_matrix = [[0 for i in range(3)] for j in range(3)]
        translation_vec = [0 for i in range(3)]

        for i in range(3):
                for j in range(3):
                        rotation_matrix[i][j] = c2w_back[i][j]
                
                translation_vec[i] = c2w_back[i][3]

        quart_back = Colmap_Handler.rotation_matrix_to_quartonian_safe(rotation_matrix)

        quart_back_conv = [0 for i in range(4)]

        quart_back_conv[0] = quart_back[3]
        quart_back_conv[1] = quart_back[0]
        quart_back_conv[2] = quart_back[1]
        quart_back_conv[3] = quart_back[2]

        return quart_back_conv, translation_vec
        
    def convert_sfm_pose_to_nerf(transform):
        c2w = np.linalg.inv(transform)

        flip_mat = np.array([
                                [1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 0, 0, 1]
                                ])

        return np.matmul(c2w, flip_mat)

    def get_colmap_transforms(self, file_path):
        colmap_transforms = {}

        i = 0

        bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
        
        with open(file_path, "r") as colmap_file:
            for line in colmap_file:
                line = line.strip()
                if len(line) != 0 and line[0] == "#":
                    continue
                i = i + 1

                if len(line) == 0:
                    continue

                if  i % 2 == 1:
                    elems=line.split(" ")

                    name = str(elems[9])

                    #if name.find(".png") == -1:
                    #    print(f"ERRONEOUS NAME {name}")
                        

                    image_id = str(elems[0])

                    qvec = np.array(tuple(map(float, elems[1:5])))
                    tvec = np.array(tuple(map(float, elems[5:8])))

                    R = Colmap_Handler.qvec2rotmat(-qvec)
                    t = tvec.reshape([3,1])

                    c2w = np.concatenate([np.concatenate([R, t], 1), bottom], 0)
                    
                    c2w_flipped = Colmap_Handler.convert_sfm_pose_to_nerf(c2w)
                        
                    colmap_transforms[name] = c2w_flipped.tolist()

        return colmap_transforms

    def add_cam(self, cam_id, camera_info, output_dir):
        
        camera_line = f"{cam_id} OPENCV {camera_info['w']} {camera_info['h']} {camera_info['fl_x']} {camera_info['fl_y']} {camera_info['cx']} {camera_info['cy']} {camera_info['k1']} {camera_info['k2']} {camera_info['p1']} {camera_info['p2']}\n"

        camera_file = os.path.join(output_dir, "cameras.txt")

        with open(camera_file, "w" if not os.path.exists(camera_file) or cam_id==1 else "a") as cameras:
            cameras.writelines(camera_line)
                    
        camera_info_bytes = np.array([camera_info['fl_x'], camera_info['fl_y'], camera_info['cx'], camera_info['cy'], 
                                      camera_info['k1'], camera_info['k2'], camera_info['p1'], camera_info['p2']]).tobytes()

        model = 4

        if self.cur.execute(f"SELECT * FROM cameras WHERE camera_id='{cam_id}';").fetchone() is None:
            self.cur.execute(f"INSERT INTO cameras(camera_id, model, width, height, params, prior_focal_length) VALUES({cam_id}, {model}, {camera_info['w']}, {camera_info['h']}, ?, 0)", (camera_info_bytes,))
            self.conn.commit() 
        else:
            self.cur.execute(f"UPDATE cameras SET model={model}, width={camera_info['w']}, height={camera_info['h']}, params=?, prior_focal_length=0 WHERE camera_id={cam_id}", (camera_info_bytes,))
            self.conn.commit()

    def update_image_camera_ids(self, frames, camera_id):
        for frame in frames:

            image_id = self.cur.execute("SELECT * FROM images WHERE name='" + frame + "';").fetchone()

            if image_id is None:
                raise Exception(f"Could not find image {frame} in transform")

            image_id = image_id[0]

            self.keys_for_file_names[image_id] = frame

            prior_image_str = f"UPDATE images SET camera_id = {camera_id} WHERE image_id = {int(image_id)}"

            self.cur.execute(prior_image_str)

            self.conn.commit()

    def convert_and_add_transforms_and_cam(self, transform_frames, output_dir, camera_info = {}, image_cams={}):

        self.cur.execute(f"""DELETE FROM cameras WHERE camera_id > {len(camera_info)}""")

        self.conn.commit()

        for cam_id, cam in camera_info.items():
            self.add_cam(cam_id, cam, output_dir)

        _ = open(os.path.join(output_dir,"points3D.txt"), "w")

        transforms_quart = {}

        for file_path, transform_matrix in transform_frames.items():

            quart, trans = self.convert_transform_to_sfm_pose(np.array(transform_matrix))

            transforms_quart[file_path[file_path.rfind("/")+1:]] = quart + trans

        for fname, transform in transforms_quart.items():

            image_id = self.cur.execute("SELECT * FROM images WHERE name='" + fname + "';").fetchone()

            if image_id is None:
                raise Exception(f"Could not find image {fname} in transform")

            image_id = image_id[0]

            self.keys_for_file_names[image_id] = fname

            prior_image_str = f"""UPDATE images SET prior_qw = {transform[3]}, prior_qx = {transform[0]},
                            prior_qy = {transform[1]}, prior_qz = {transform[2]}, prior_tx = {transform[4]},
                            prior_ty = {transform[5]}, prior_tz = {transform[6]}"""

            if len(image_cams) != 0:
                prior_image_str += f", camera_id = {image_cams[fname]}"

            prior_image_str += f" WHERE image_id = {int(image_id)}"

            self.cur.execute(prior_image_str)

            self.conn.commit()

        with open(os.path.join(output_dir,"images.txt"), "w") as colmap_file:
            for image_id, fname in self.keys_for_file_names.items():
                transform = transforms_quart[fname]

                transformation = [str(x) for x in transform]

                transformation_str = " ".join(transformation)
                
                file_str = str(image_id) + " " + transformation_str 
                
                if len(image_cams) != 0:
                    file_str += f" {image_cams[fname]} "
                else:
                    file_str += " 1 "
                    
                file_str += fname + "\n" + "\n"

                colmap_file.writelines(file_str)
        
    def compute_features(self, feature_mode="HIGH", image_list = None, multiple_cams=False): 
        compute_features_cmd = f"{self.colmap_binary} feature_extractor --SiftExtraction.estimate_affine_shape=true --SiftExtraction.domain_size_pooling=true \
                                                 --database_path {self.db} --image_path {self.img_dir} --ImageReader.camera_model OPENCV"

        if self.mask_dir is not None:
           compute_features_cmd +=  f" --ImageReader.mask_path {self.mask_dir}"

        if image_list is not None:
            compute_features_cmd += f" --image_list_path {image_list}"

        #if not multiple_cams:
        #compute_features_cmd += " --ImageReader.single_camera 1"

        return os.system(compute_features_cmd)

    def compute_feature_matches(self):
        if os.path.isfile(self.img_name_pair_path):
            return os.system(f"{self.colmap_binary} matches_importer  --database_path {self.db} --match_list_path {self.img_name_pair_path} --match_type pairs")
        
        return os.system(f"{self.colmap_binary} exhaustive_matcher  --database_path {self.db}")
    
    def triangulate_points(self, input_path, output_path):
        return os.system(f"{self.colmap_binary} point_triangulator --database_path {self.db} --image_path {self.img_dir} --input_path {input_path} --output_path {output_path}")
    
    def bundle_adjustment(self, input_path, output_path):
        return os.system(f"{self.colmap_binary} bundle_adjuster --input_path {input_path} --output_path {output_path} --BundleAdjustment.refine_principal_point 0 \
                                            --BundleAdjustment.refine_focal_length 0 --BundleAdjustment.refine_extra_params 0 --BundleAdjustment.refine_extrinsics 1 \
                                            --BundleAdjustment.max_num_iterations 150 --BundleAdjustment.max_linear_solver_iterations 1000")
    
    def convert_model_to_txt(self, input_path, output_path):
        return os.system(f"{self.colmap_binary} model_converter --input_path {input_path} --output_path {output_path} --output_type TXT")

    def undistort_images(self, input_path, output_path):
        return os.system(f"{self.colmap_binary} image_undistorter --image_path {self.img_dir} --input_path {input_path} \
                                              --output_path {output_path} --output_type COLMAP")

    def register_new_images(self, input_dir, output_dir):
        return os.system(f"colmap image_registrator --database_path {self.db} --input_path {input_dir} --output_path {output_dir}")

    def perform_structure_from_motion(self, input_dir, recon_dir, ba_dir, output_dir, convert_model=True):
        if self.triangulate_points(input_dir, recon_dir):
            return None

        if self.bundle_adjustment(recon_dir, ba_dir):
            return None

        if convert_model:
            if self.convert_model_to_txt(ba_dir, output_dir):
                return None
            
            return self.get_colmap_transforms(os.path.join(output_dir, "images.txt"))

        return True

    def generate_coloured_pointcloud(self):
        err = os.system(f"{self.colmap_binary} patch_match_stereo --workspace_path {self.dense_dir}")
        if err:
            return err

        return os.system(f"{self.colmap_binary} stereo_fusion --workspace_path {self.dense_dir} -output_path {self.dense_dir}/sparse/fused.ply")

    def extend_reconstruction(self, input_dir, output_dir):
        if self.register_new_images(input_dir, output_dir):
            return True

        if self.bundle_adjustment(output_dir, output_dir):
            return True

    def generate_img_list(self, frames, cam_id):
        image_list_path = os.path.join(self.sfm_data_dir, f"{cam_id}_image_list.txt")
            
        with open(image_list_path, "w") as image_list_txt:
            for frame in frames:
                image_list_txt.writelines(frame + "\n")

        return image_list_path
    
    def delete_dir_contents(self, dir, delete_txt=True, delete_bin=True):
        try:
            files = os.listdir(directory_path)
            
            for file in files:
                file_path = os.path.join(directory_path, file)
                
                if delete_txt and file.endswith('.txt'):
                    os.remove(file_path)
                elif delete_bin and file.endswith('.bin'):
                    os.remove(file_path)
        except Exception as e:
            return False
    
    def normalise_transforms(self, original_transforms, adjusted_transforms):
        original_centre_attention = np.array([0.0,0.0,0.0])

        for file_name, transform_matrix in original_transforms.items():
            translation = np.array(transform_matrix)[:3, 3]
            original_centre_attention += translation

        original_centre_attention /= len(original_transforms)

        current_origin = np.array([0.0,0.0,0.0])

        for file_name, transform_matrix in adjusted_transforms.items():
            translation = np.array(transform_matrix)[:3, 3]
            current_origin += translation

        current_origin /= len(adjusted_transforms)

        print("Original Origin: ", original_centre_attention)
        print("Adjusted Origin: ", current_origin)
        print("Difference: ", original_centre_attention-current_origin)

        original_avg_radius = 0.

        for file_name, transform_matrix in original_transforms.items():
            translation = np.array(transform_matrix)[:3, 3] - original_centre_attention

            original_avg_radius += np.linalg.norm(translation)

        original_avg_radius /= len(original_transforms)

        avg_radius = 0.

        for file_name, transform_matrix in adjusted_transforms.items():
            transform = np.array(transform_matrix)

            transform[:3, 3] -= current_origin

            avg_radius += np.linalg.norm(transform[:3, 3])

            transform_matrix = transform.tolist()

        avg_radius /= len(adjusted_transforms)

        radius_scale = avg_radius/original_avg_radius
        print(f"Original Radius: {original_avg_radius}")
        print("Adjusted Radius: ", avg_radius)
        print("Radius Scale: ", radius_scale)

        is_valid = True
        if radius_scale < 0.1 or radius_scale > 3:
            print("WARNING: THESE TRANSFORMS ARE NOT VALID")
            is_valid = False

        for file_name, transform_matrix in adjusted_transforms.items():
            transform = np.array(transform_matrix)

            transform[:3, 3] /= radius_scale

            transform[:3, 3] += original_centre_attention

            transform_matrix = transform.tolist()

        return adjusted_transforms, is_valid

    def adjust_transforms_using_registration(self, all_transform_frames, camera_info = {}, sfm_settings={}, image_cams={}):
        transforms_per_cam = {}

        for frame, cam_id in image_cams.items():
            if cam_id not in transforms_per_cam.keys():
                transforms_per_cam[cam_id] = [frame]
            else:
                transforms_per_cam[cam_id].append(frame)

        inital_reconstruction = True

        prev_dir = None

        for cam_id, frames in transforms_per_cam.items():

            image_list_path = self.generate_img_list(frames, cam_id)

            current_dir = os.path.join(self.sfm_data_dir, f"sparse_{cam_id}")

            if not os.path.exists(current_dir):
                os.mkdir(current_dir)

            print(f"Adding new cameras for group: {cam_id}")

            print(f"1. Computing features")
            self.compute_features(image_list=image_list_path, multiple_cams=True if len(camera_info) > 1 else False)

            print()
            print("2. Matching Features")
            self.compute_feature_matches()

            if prev_dir is None:
                initial_transforms = {}

                for file_path, transform in all_transform_frames.items():
                    if file_path[file_path.rfind("/")+1:] in frames:
                        initial_transforms[file_path] = transform

                print()
                print("3. Adding Camera and Pose Priors")
                self.convert_and_add_transforms_and_cam(initial_transforms, current_dir, camera_info=camera_info, image_cams=image_cams)

                print()
                print("4. Calculating new camera poses")
                self.perform_structure_from_motion(current_dir, self.reconstructed_dir, current_dir, None, convert_model=False)

                self.delete_dir_contents(current_dir, delete_bin=False)
            else:

                self.update_image_camera_ids(frames, cam_id)

                for i in range(self.num_adjust_iters):
                    print()
                    print(f"3 #{i}. Extending Reconstruction")
                    self.extend_reconstruction(prev_dir, current_dir)

                    self.delete_dir_contents(current_dir, delete_bin=False)

                    if i != num_registration_attempts-1:
                        prev_dir = current_dir

                        current_dir = current_dir + "_1"

                        os.mkdir(current_dir)

            prev_dir = current_dir

        if self.convert_model_to_txt(current_dir, self.adjusted_txt_dir):
            return None

        new_transforms = self.get_colmap_transforms(os.path.join(self.adjusted_txt_dir, "images.txt"))

        if "undistort_imgs" not in sfm_settings or sfm_settings["undistort_imgs"]:
            print()
            print("5. Undistorting Images")

            start_time = time.time()
            if self.undistort_images(self.adjusted_txt_dir, self.dense_dir):
                print("UNABLE TO UNDISTORT IMAGES")
            print("Time taken: ", round(time.time()-start_time, 2))

            print()
            print("6. Generating point cloud")

            start_time = time.time()
            err = self.generate_coloured_pointcloud()
            if err:
                print("UNABLE TO GENERATE POINT CLOUD")
            print("Time taken: ", round(time.time()-start_time, 2))

        os.rename(self.sfm_data_dir, os.path.join(self.experiment_dir,  "colmap"))

        return new_transforms

    def adjust_transforms_using_global_ba(self, transform_frames, camera_info = {}, sfm_settings={}, image_cams=None):

        print("1. Computing features")

        start_time = time.time()
        if self.compute_features(feature_mode=sfm_settings["feature_mode"], multiple_cams=True if len(camera_info) > 1 else False):
            print("UNABLE TO EXRACT FEATURES")
            return None
        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("2. Matching Features")

        start_time = time.time()
        if self.compute_feature_matches():
            print("UNABLE TO MATCH FEATURES")
            return None
        print("Time taken: ", round(time.time()-start_time, 2))

        original_transforms = transform_frames.copy()

        for i in range(self.num_adjust_iters):
            self.adjust_iter = i
            self.setup_colmap_subdirs()

            if i != 0:
                shutil.copyfile(os.path.join(self.experiment_dir,  f"colmap_{i-1}", "colmap.db"), os.path.join(self.experiment_dir,  f"colmap_{i}", "colmap.db"))
                shutil.copyfile(os.path.join(self.experiment_dir,  f"colmap_{i-1}", "imageNamePairs.txt"), self.img_name_pair_path)

                shutil.rmtree(prev_colmap_path)

            print()
            print("3. Adding Camera and Pose Priors")
            start_time = time.time()
            self.convert_and_add_transforms_and_cam(transform_frames, self.sparse_dir, camera_info=camera_info, image_cams=image_cams)
            print("Time taken: ", round(time.time()-start_time, 2))

            print()
            print("4. Calculating new camera poses")

            start_time = time.time()
            new_transforms = self.perform_structure_from_motion(self.sparse_dir, self.reconstructed_dir, self.adjusted_dir, self.adjusted_txt_dir)

            if new_transforms is None:
                print("UNABLE TO CALCULATE NEW POSES")
                return None
            print("Time taken: ", round(time.time()-start_time, 2))

            new_transforms, is_valid = self.normalise_transforms(original_transforms, new_transforms)

            if not is_valid:
                return None

            prev_colmap_path = self.sfm_data_dir

            transform_frames = new_transforms

        if "undistort_imgs" not in sfm_settings or sfm_settings["undistort_imgs"]:
            print()
            print("5. Undistorting Images")

            start_time = time.time()
            err = self.undistort_images(self.adjusted_dir, self.dense_dir)
            if err:
                print("UNABLE TO UNDISTORT IMAGES")
            print("Time taken: ", round(time.time()-start_time, 2))

            print()
            print("6. Generating point cloud")

            start_time = time.time()
            err = self.generate_coloured_pointcloud()
            if err:
                print("UNABLE TO GENERATE POINT CLOUD")
            print("Time taken: ", round(time.time()-start_time, 2))

        shutil.rmtree(self.sparse_dir)

        os.rename(prev_colmap_path, os.path.join(self.experiment_dir,  "colmap"))

        return new_transforms

"""class OpenMVG_Handler(SfMHandler):

    def __init__(self, experiment_name, log_dir, img_dir="images/rgb", mask_dir = None):

        super()

        self.openMVG_path = os.path.join(log_dir, "openmvg_path.txt")

        self.experiment_dir = os.path.join(log_dir, experiment_name)
        self.img_dir = img_dir

        if not os.path.exists(self.openMVG_path):
            raise Exception("Cannot find OpenMVG path in file: " + self.openMVG_path)
        
        with open(os.path.join(log_dir, "openmvg_path.txt")) as openMVG_path_file:
            self.OPENMVG_SFM_BIN  = os.path.expanduser(str(openMVG_path_file.read()).strip())

        if not os.path.exists(self.OPENMVG_SFM_BIN):
            raise Exception("Given OpenMVG file path does not exist: " + self.OPENMVG_SFM_BIN)
        
        self.sensor_width_camera_database = os.path.join(self.OPENMVG_SFM_BIN, "sensor_width_camera_database.txt")
        #os.path.join(log_dir, "sensor_width_camera_database.txt")

        #if not os.path.exists(self.sensor_width_camera_database):
        #    raise Exception("Could not find sensor width camera database in log directory")

        self.setup_directory()

        self.file_names_for_key = {}
        self.keys_for_file_names = {}

        self.adjust_focal_length = True
        self.adjust_principal_point = True
        self.adjust_distortion = True

        self.generate_sfm_file()


    def setup_directory(self):
        self.img_dir = os.path.join(self.experiment_dir, self.img_dir)
        self.sfm_data_dir = os.path.join(self.experiment_dir, "openMV")
        self.matches_dir = os.path.join(self.sfm_data_dir , "matches")
        self.output_dir = os.path.join(self.sfm_data_dir , "output")

        self.img_name_pair_path = os.path.join(self.sfm_data_dir, "imageNamePairs.txt")
        self.img_pair_path = os.path.join(self.sfm_data_dir, "imageKeyPairs.txt")

        if not os.path.exists(self.sfm_data_dir):
            os.mkdir(self.sfm_data_dir)

        if not os.path.exists(self.matches_dir):
            os.mkdir(self.matches_dir)

        if not os.path.exists(self.output_dir):
            os.mkdir(self.output_dir)

        self.sfm_file_path = os.path.join(self.matches_dir,"sfm_data.json")


    
    def convert_transform_to_sfm_pose(self, transform):

        flip_mat = np.array([
                            [1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]
                    ])
        
        c2w = np.matmul(transform, flip_mat)
        
        c2w_back = np.linalg.inv(c2w)
        
        rot, tvec = convert_transform_to_rot_and_trans(c2w_back)
        
        tvec = np.array([-x for x in tvec])
        
        rot_inv = np.linalg.inv(np.array(rot))
        
        center = multiply_matrix_by_point(rot_inv, tvec)
        
        return rot, center
    
    def convert_sfm_pose_to_nerf(self, rot, center):
        bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
        
        flip_mat = np.array([
                            [1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]
                    ])
        
        tvec = multiply_matrix_by_point(rot, center)
        
        tvec = np.array([-x for x in tvec])
        
        R = np.array(rot)
        
        t = tvec.reshape([3,1])
        m = np.concatenate([np.concatenate([R, t], 1), bottom], 0)
        c2w = np.linalg.inv(m)
        
        c2w =  np.matmul(c2w, flip_mat)
        
        return c2w

    def generate_sfm_file(self):
        pIntrisics = subprocess.Popen([os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  
                                       "-i", self.img_dir, "-o", self.matches_dir, "-d", self.sensor_width_camera_database, 
                                       "-f 100"])
        pIntrisics.wait()

        time.sleep(0.1)

        with open(self.sfm_file_path, "r") as sfm_file:
            sfm_data = json.load(sfm_file)

        for view in sfm_data["views"]:
            self.file_names_for_key[str(view["key"])]  = view["value"]["ptr_wrapper"]["data"]["filename"]
            self.keys_for_file_names[view["value"]["ptr_wrapper"]["data"]["filename"]] = str(view["key"])

    

    def convert_and_add_transforms_and_cam(self, transform_frames, output_dir, camera_info = {}):

        with open(self.sfm_file_path, "r") as sfm_file:
            sfm_data = json.load(sfm_file)

        pose_priors = {}
        self.img_sharpness = {}

        for frame in transform_frames:

            fname = frame["file_path"]
            transform = frame["transform_matrix"]

            if "sharpness" in frame:
                self.img_sharpness[fname] = frame["sharpness"]

            rotation, center = self.convert_transform_to_sfm_pose(np.array(transform))

            pose = {"rotation": rotation, "center": center}

            pose_priors[fname[fname.rfind("/")+1:]] = pose

        for view in sfm_data["views"]:
            file_name = view["value"]["ptr_wrapper"]["data"]["filename"]

            if file_name in pose_priors:
                pose = pose_priors[file_name[file_name.rfind("/")+1:]]

                view["value"]["ptr_wrapper"]["data"]["use_pose_center_prior"] = True
                view["value"]["ptr_wrapper"]["data"]["center_weight"] = [1.0, 1.0, 1.0]
                view["value"]["ptr_wrapper"]["data"]["center"] = pose["center"]

                view["value"]["ptr_wrapper"]["data"]["use_pose_rotation_prior"] = True
                view["value"]["ptr_wrapper"]["data"]["rotation_weight"] = 1.0
                view["value"]["ptr_wrapper"]["data"]["rotation"] = pose["rotation"]

        extrinsics = []

        for fname, pose in pose_priors.items():
            extrinsics.append({"key": int(self.keys_for_file_names[fname]), "value": pose})

        sfm_data["extrinsics"] = extrinsics

        if camera_info != None and camera_info != {}:
            intrinsics = sfm_data["intrinsics"][0]["value"]["ptr_wrapper"]["data"]

            if "w" in camera_info and "h" in camera_info:
                intrinsics["width"] = int(camera_info["w"])
                intrinsics["height"] = int(camera_info["h"])

            if "fl_x" in camera_info:
                intrinsics["focal_length"] = float(camera_info["fl_x"])

                self.adjust_focal_length = False

            if "cx" in camera_info and "cy" in camera_info:
                intrinsics["principal_point"] = [float(camera_info["cx"]), float(camera_info["cy"])]

                self.adjust_principal_point = False

            if "k1" in camera_info and "k2" in camera_info and "k3" in camera_info:
                intrinsics["disto_k3"] = [float(camera_info["k1"]), float(camera_info["k2"]), float(camera_info["k3"])]

                self.adjust_distortion = False
        
        with open(self.sfm_file_path, "w") as sfm_file:
            json.dump(sfm_data, sfm_file, indent=4)

    def compute_features(self, feature_mode: str="HIGH"):
        avaliable_modes = ["NORMAL", "HIGH", "ULTRA"]

        if feature_mode not in avaliable_modes:
            raise Exception(f"Feature mode must match one of these options: {avaliable_modes}")

        pFeatures = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  
                                       "-i", self.sfm_file_path, "-o", self.matches_dir, "-m", "SIFT", "-p", feature_mode] )
        pFeatures.wait()
        
    def convert_img_name_matches_to_img_key_matches(self):
        with open(self.img_name_pair_path, "r") as img_name_file:
            with open(self.img_pair_path, "w") as img_key_file:
                for line in img_name_file.readlines():
                    image_1, image_2 = line.strip().split(" ")

                    key_1 = self.keys_for_file_names[image_1]
                    key_2 = self.keys_for_file_names[image_2]

                    img_key_file.write(str(key_1) + " " + str(key_2) + "\n")
                        
    def compute_feature_matches(self):
        pMatches = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),  
                                      "-i", self.sfm_file_path, "-p", self.img_pair_path, 
                                      "-o", self.matches_dir + "/matches.putative.bin" ] )
        pMatches.wait()

        pFiltering = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_GeometricFilter"), 
                                        "-i", self.sfm_file_path, "-m", self.matches_dir+"/matches.putative.bin" , 
                                        "-g" , "f" , "-o" , self.matches_dir+"/matches.f.bin" ] )
        pFiltering.wait()

    def perform_structure_from_motion(self, sfm_type: str="INCREMENTALV2"):
        avaliable_types = ["GLOBAL", "INCREMENTAL", "INCREMENTALV2", "RECONSTRUCTION"]

        if sfm_type not in avaliable_types:
            raise Exception("SFM type must match one of the following options: {avaliable_types}")
        
        intrinsic_properties = ""

        if self.adjust_focal_length:
             intrinsic_properties += "ADJUST_FOCAL_LENGTH|"
        if self.adjust_principal_point:
             intrinsic_properties += "ADJUST_PRINCIPAL_POINT|"
        if self.adjust_distortion:
             intrinsic_properties += "ADJUST_DISTORTION|"

        if intrinsic_properties == "":
             intrinsic_properties = "NONE"
        else:
             intrinsic_properties = intrinsic_properties[:-1]

        if sfm_type == "INCREMENTALV2":
                pRecons = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_SfM"), "-P", 
                                             "--sfm_engine", "INCREMENTALV2", "--input_file", self.sfm_file_path, 
                                             "--match_dir", self.matches_dir, "--output_dir", self.output_dir,
                                             "-S", "EXISTING_POSE", 
                                             "-f", intrinsic_properties] )
                                             
        elif sfm_type == "INCREMENTAL":
                pRecons = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_SfM"), "-P", 
                                             "--sfm_engine", "INCREMENTAL", "--input_file", self.sfm_file_path, 
                                             "--match_dir", self.matches_dir, "--output_dir", self.output_dir,
                                             "-a", "0001.png" , "-b", "0002.png", 
                                             "-f", intrinsic_properties] )
                
        elif sfm_type == "GLOBAL":
            pRecons = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_SfM"), "-P", 
                                             "--sfm_engine", "GLOBAL", "--input_file", self.sfm_file_path, 
                                             "--match_dir", self.matches_dir, "--output_dir", self.output_dir,
                                             "-f", intrinsic_properties] )

        else:
            pRecons = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_ComputeStructureFromKnownPoses"), 
                                         "--input_file", self.sfm_file_path, "-m", self.matches_dir, 
                                         "-o", self.output_dir + "/adjusted_poses.json", "-b", 
                                         "-p",  self.img_pair_path] )
    
        pRecons.wait()

        if sfm_type != "RECONSTRUCTION":
            pConvert = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"), 
                                        "-i", self.output_dir + "/sfm_data.bin", "-o", 
                                        self.output_dir + "/adjusted_poses.json", "-E", "-V"])
            pConvert.wait()

        new_transforms = []

        time.sleep(0.1)

        with open(self.output_dir + "/adjusted_poses.json", "r") as adjusted_poses_file:
            mvg_poses = json.load(adjusted_poses_file)

        for frame in mvg_poses["extrinsics"]:
            rot = frame["value"]["rotation"]
            center = frame["value"]["center"]
                
            converted_pose = self.convert_sfm_pose_to_nerf(rot, center)

            new_frame = {"file_path": self.file_names_for_key[str(frame["key"])],
                         "transform_matrix": converted_pose.tolist()}
            
            if new_frame["file_path"] in self.img_sharpness:
                 new_frame["sharpness"] = self.img_sharpness[new_frame["file_path"]]

            new_transforms.append(new_frame)

        return new_transforms
    
    def generate_coloured_pointcloud(self):
        pCloud = subprocess.Popen( [os.path.join(self.OPENMVG_SFM_BIN, "openMVG_main_ComputeSfM_DataColor"),  
                                    "-i", self.output_dir+"/sfm_data.bin", 
                                    "-o", os.path.join(self.output_dir,"colorized.ply")] )
        pCloud.wait()
         
    
    def adjust_transforms(self, transform_frames, camera_info = {}, sfm_settings={}):

        print("1. Generating SfM File")

        start_time = time.time()
        self.convert_and_add_transforms_and_cam(transform_frames, None, camera_info=camera_info)
        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("2. Converting Img Name Pairs to Key Pairs")

        start_time = time.time()
        self.convert_img_name_matches_to_img_key_matches()
        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("3. Computing Features")

        start_time = time.time()
        self.compute_features(feature_mode=sfm_settings["feature_mode"])
        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("4. Matching features")

        start_time = time.time()
        self.compute_feature_matches()
        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("5. Calculating camera poses")
        
        start_time = time.time()
        new_transforms = self.perform_structure_from_motion(sfm_type=sfm_settings["sfm_type"])
        converted_transforms = [{"file_path": "rgb/"+name, "transform": transform} for name, transform in new_transforms.items()]
        print(converted_transforms)

        print("Time taken: ", round(time.time()-start_time, 2))

        print()
        print("6. Generating point cloud")

        start_time = time.time()
        self.generate_coloured_pointcloud()
        print("Time taken: ", round(time.time()-start_time, 2))

        return converted_transforms"""

def get_sfm_handler(sfm_type, experiment_name, log_dir, img_dir=None, mask_dir=None):
    if sfm_type == "COLMAP":
        return Colmap_Handler(experiment_name, log_dir, img_dir=img_dir, mask_dir=mask_dir)
        
        """elif sfm_type == "OPENMVG":
        return OpenMVG_Handler(experiment_name, log_dir, img_dir=img_dir)"""
        
        """ ADD YOUR METHOD HERE """
        
    else:
        raise Exception(f"Unknown SFM handler type: {sfm_type}")


def config_parser():
    parser = configargparse.ArgumentParser()

    parser.add_argument("--config", is_config_file=True, help="Path to configuration file containing a list of configuration arguments")
    parser.add_argument("--log_dir", required=True, type=str)
    parser.add_argument("--experiment_name", required=True, type=str)
    parser.add_argument("--camera_file_path", type=str, default=None)

    parser.add_argument("--sfm_package", type=str, default="COLMAP")

    parser.add_argument("--sfm_type", type=str, default="INCREMENTALV2")
    parser.add_argument("--feature_mode", type=str, default="HIGH")
    parser.add_argument("--use_mask", type=bool, default=False)

    parser.add_argument("--img_pair_path", type=str, default=None)
     
    parser.add_argument("--override_existing_sfm", type=bool, default=False)
    
    return parser.parse_args()

if __name__ == "__main__":

    args = config_parser()

    experiment_dir = os.path.join(args.log_dir, args.experiment_name)

    mask_dir = os.path.join(experiment_dir, "images", "mask") if args.use_mask else None

    transforms_train_file = os.path.join(experiment_dir, "transforms", "original", "transforms.json")

    with open(transforms_train_file, "r") as transform_file:
        transform_json = json.load(transform_file)

    camera_info = {}

    if "fl_x" in transform_json:
        camera_info["fl_x"] = float(transform_json["fl_x"])

    if "fl_y" in transform_json:
        camera_info["fl_y"] = float(transform_json["fl_y"])

    if "w" in transform_json and "h" in transform_json:
        camera_info["w"] = int(transform_json["w"])
        camera_info["h"] = int(transform_json["h"])
    
    if "cx" in transform_json and "cy" in transform_json:
        camera_info["cx"] = float(transform_json["cx"])
        camera_info["cy"] = float(transform_json["cy"])
    
    if "k1" in transform_json and "k2" in transform_json and "k3" in transform_json:
        camera_info["k1"] = float(transform_json["k1"])
        camera_info["k2"] = float(transform_json["k2"])
        camera_info["k3"] = float(transform_json["k3"])

    if "p1" in transform_json and "p2" in transform_json:
        camera_info["p1"] = float(transform_json["p1"])
        camera_info["p2"] = float(transform_json["p2"])

    sfm_settings = {}

    sfm_settings["sfm_type"] = args.sfm_type
    sfm_settings["feature_mode"] = args.feature_mode

    default_img_name_pair = os.path.join(experiment_dir, "colmap", "imageNamePairs.txt")

    if args.sfm_package == "COLMAP":
        sfm_handler = Colmap_Handler(args.experiment_name, args.log_dir,  mask_dir=mask_dir)
    elif args.sfm_package == "OPENMVG":
        sfm_handler = OpenMVG_Handler(args.experiment_name, args.log_dir)
    else:
         print(f"SfM Package {args.sfm_package} is not currently supported")

    if args.img_pair_path == None and not os.path.isfile(default_img_name_pair):
        print("Creating image match file")

        sfm_handler.generate_img_name_matches_linear()

    elif os.path.isfile(default_img_name_pair):
        print(f"Using image match file: {default_img_name_pair}")

        sfm_handler.set_img_name_pair_path(default_img_name_pair)
    else:
        if os.path.isfile(args.img_pair_path):
            print(f"Using image match file: {args.img_pair_path}")

            sfm_handler.set_img_name_pair_path(args.img_pair_path)
        else:
            print(f"WARNING: image match file {args.img_pair_path} is invalid")

            exit(0)

    if os.path.isdir(os.path.join(experiment_dir, "colmap")) and args.override_existing_sfm and False:
        if os.path.exists(sfm_handler.sparse_dir):
            shutil.rmtree(sfm_handler.sparse_dir)

        if os.path.exists(sfm_handler.dense_dir):
            shutil.rmtree(sfm_handler.dense_dir)

        if os.path.exists(os.path.join(sfm_handler.sfm_data_dir , "adjusted")):
            shutil.rmtree(os.path.join(sfm_handler.sfm_data_dir , "adjusted"))

        if os.path.exists(sfm_handler.reconstructed_dir):
            shutil.rmtree(sfm_handler.reconstructed_dir)

        os.remove(sfm_handler.db)

        sfm_handler.setup_directory()
    
    if os.path.isdir(os.path.join(experiment_dir, "openMV")) and args.override_existing_sfm:
        if not os.path.exists(sfm_handler.matches_dir):
            os.mkdir(sfm_handler.matches_dir)

        if not os.path.exists(sfm_handler.output_dir):
            os.mkdir(sfm_handler.output_dir)

        sfm_handler.setup_directory()

    frame_data = {}

    for frame in transform_json["frames"]:
        new_frame_data = {
            "transform_matrix": frame["transform_matrix"],
            "file_path": frame["file_path"]
        }

        if "mask_path" in frame:
            new_frame_data["mask_path"] = frame["mask_path"]

        frame_data[frame["file_path"]] = new_frame_data

    new_transforms = sfm_handler.adjust_transforms(transform_json["frames"], camera_info={"1":camera_info}, 
                                                    sfm_settings=sfm_settings)
    
    transform_data = {**camera_info, **new_transforms}

    with open(os.path.join(args.log_dir, args.experiment_name, "transforms", "adjusted", "transforms.json"), "w") as save_file:
        json.dump(transform_data, save_file, indent=4)


