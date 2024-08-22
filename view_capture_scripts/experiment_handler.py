import json
import os
import shutil
import cv2

import distutils.dir_util 

class ExperimentHandler():
    def __init__(self, log_dir, experiment_name):
        # Experiment directory structure
        self.img_dir = "images"
        self.transform_dir = "transforms"
        self.exports_dir = "exports"

        self.rgb_img_dir = os.path.join(self.img_dir, "rgb")
        self.depth_img_dir = os.path.join(self.img_dir, "depth")
        self.mask_img_dir = os.path.join(self.img_dir, "mask")
        self.segmented_img_dir = os.path.join(self.img_dir, "segmented")
        self.undistored_img_dir = os.path.join(self.img_dir, "undistorted")
        self.undistored_segmented_img_dir = os.path.join(self.img_dir, "undistorted_segmented")

        self.original_transform_dir = os.path.join(self.transform_dir, "original")
        self.adjusted_transform_dir = os.path.join(self.transform_dir, "adjusted")
        self.depth_transform_dir = os.path.join(self.transform_dir, "depth")
        self.mask_transform_dir = os.path.join(self.transform_dir, "mask")
        self.segmented_transform_dir = os.path.join(self.transform_dir, "segmented")
        self.depth_mask_transform_dir = os.path.join(self.transform_dir, "depth_mask")
        self.undistorted_transform_dir = os.path.join(self.transform_dir, "undistorted")

        self.pc_export_dir = os.path.join(self.exports_dir, "pointcloud")
        self.checkpoint_exports_dir = os.path.join(self.exports_dir, "checkpoint")
        self.render_exports_dir = os.path.join(self.exports_dir, "render")

        self.experiment_name = experiment_name
        self.log_dir = log_dir

        self.experiment_dir_path = os.path.join(self.log_dir, self.experiment_name)

    def create_new_dir(self, args):

        self.experiment_dir_path = os.path.join(self.log_dir, self.experiment_name)

        # Create new directory for saving experiment information
        os.makedirs(self.experiment_dir_path)

        # Create top level directories
        os.makedirs(os.path.join(self.experiment_dir_path, self.img_dir))
        os.makedirs(os.path.join(self.experiment_dir_path, self.transform_dir))
        os.makedirs(os.path.join(self.experiment_dir_path, self.exports_dir))

        # Create directories for different images
        os.makedirs(os.path.join(self.experiment_dir_path, self.rgb_img_dir))
        if args.capture_depth:
            os.makedirs(os.path.join(self.experiment_dir_path, self.depth_img_dir))
        if args.segment_method is not None:
            os.makedirs(os.path.join(self.experiment_dir_path, self.mask_img_dir))
            os.makedirs(os.path.join(self.experiment_dir_path, self.segmented_img_dir))
        if args.adjust_transforms and args.undistort_imgs:
            os.makedirs(os.path.join(self.experiment_dir_path, self.undistored_img_dir))

        # Create directories for different transforms
        os.makedirs(os.path.join(self.experiment_dir_path, self.original_transform_dir))
        if args.adjust_transforms:
            os.makedirs(os.path.join(self.experiment_dir_path, self.adjusted_transform_dir))
            if args.undistort_imgs:
                os.makedirs(os.path.join(self.experiment_dir_path, self.undistorted_transform_dir))
        if args.capture_depth:
            os.makedirs(os.path.join(self.experiment_dir_path, self.depth_transform_dir))
        if args.segment_method is not None:
            os.makedirs(os.path.join(self.experiment_dir_path, self.mask_transform_dir))
            os.makedirs(os.path.join(self.experiment_dir_path, self.segmented_transform_dir))
            if args.capture_depth:
                os.makedirs(os.path.join(self.experiment_dir_path, self.depth_mask_transform_dir))
            if args.adjust_transforms and args.undistort_imgs:
                os.makedirs(os.path.join(self.experiment_dir_path, self.undistored_segmented_img_dir))

        # Create directories for different experiment exports
        os.makedirs(os.path.join(self.experiment_dir_path, self.pc_export_dir))
        os.makedirs(os.path.join(self.experiment_dir_path, self.checkpoint_exports_dir))
        os.makedirs(os.path.join(self.experiment_dir_path, self.render_exports_dir))

        self.set_experiment_config(args)

    def get_experiment_dir(self):
        return self.experiment_dir_path

    def set_experiment_name(self, experiment_name):
        self.experiment_name = experiment_name

    def does_experiment_exist(self):
        return os.path.exists(self.experiment_dir_path)

    def get_experiment_config(self):

        try:
            with open(os.path.join(self.experiment_dir_path, "config.txt"), 'r') as f:
                config_data = json.load(f)
        except Exception:
            return None

        return config_data

    def set_experiment_config(self, args):
        with open(f'{os.path.join(self.experiment_dir_path, "config.txt")}', 'w') as file:
            json.dump(args.__dict__, file, indent=2)
    
    def remove_current_experiment(self):
        shutil.rmtree(self.experiment_dir_path)

    def get_rgb_dir(self):
        return self.rgb_img_dir
    
    def get_depth_dir(self):
        return self.depth_img_dir
    
    def get_mask_dir(self):
        return self.mask_img_dir
    
    def get_segmented_dir(self):
        return self.segmented_img_dir

    def config_gaussian_dir(self, sfm_dense_dir):
        if not os.path.exists(sfm_dense_dir):
            return False

        sfm_undistored_imgs_dir = os.path.join(self.experiment_dir_path, sfm_dense_dir, "images")
        sfm_undistored_transform_dir = os.path.join(self.experiment_dir_path, sfm_dense_dir, "sparse")

        undistorted_transform_dir = os.path.join(self.experiment_dir_path, self.undistorted_transform_dir)

        distutils.dir_util.copy_tree(sfm_undistored_imgs_dir, os.path.join(self.experiment_dir_path, self.undistored_img_dir))

        if not os.path.exists(os.path.join(undistorted_transform_dir, "sparse")):
            os.mkdir(os.path.join(undistorted_transform_dir, "sparse"))

        if not os.path.exists(os.path.join(undistorted_transform_dir, "sparse", "0")):
            os.mkdir(os.path.join(undistorted_transform_dir, "sparse", "0"))

        distutils.dir_util.copy_tree(sfm_undistored_transform_dir, os.path.join(undistorted_transform_dir, "sparse", "0"))

    def get_transform_dir(self, transform_type):
        if transform_type == "original":
            return self.original_transform_dir
        elif transform_type == "adjusted":
            return self.adjusted_transform_dir
        elif transform_type == "segmented":
            return self.segmented_transform_dir
        elif transform_type == "mask":
            return self.mask_transform_dir
        elif transform_type == "depth":
            return self.depth_transform_dir
        elif transform_type == "depth_mask":
            return self.depth_mask_transform_dir
        elif transform_type == "undistorted":
            return self.undistorted_transform_dir

        raise Exception(f"Transform directory with type {transform_type} is not defined")
    
    def get_img_dir(self, img_type):
        if img_type == "rgb":
            return self.rgb_img_dir
        elif img_type == "depth":
            return self.depth_img_dir
        elif img_type == "mask":
            return self.mask_img_dir
        elif img_type == "segmented":
            return self.segmented_img_dir
        elif img_type == "undistorted":
            return self.undistored_img_dir
        elif img_type == "undistorted_segmented":
            return self.undistored_segmented_img_dir

        raise Exception(f"Img directory with type {img_type} is not defined")

    def write_img(self, img_name: str, img_data, img_type: str):

        img_dir = self.get_img_dir(img_type)

        return cv2.imwrite(os.path.join(self.experiment_dir_path, img_dir, 
                               img_name), img_data)

    def read_img(self, img_name: str, img_type: str):
        img_dir = self.get_img_dir(img_type)

        return cv2.imread(os.path.join(self.experiment_dir_path, img_dir, img_name))

    def get_relative_file_paths(self, img_name, img_type, transform_type):

        transform_dir = self.get_transform_dir(transform_type)

        img_dir = self.get_img_dir(img_type)

        return os.path.relpath(os.path.join(img_dir, img_name), transform_dir)

    def write_transforms_file(self, transforms_json, transform_type, file_name):
        transform_dir = self.get_transform_dir(transform_type)

        with open(os.path.join(self.experiment_dir_path, transform_dir, file_name), "w") as save_file:
                json.dump(transforms_json, save_file, indent=4)
