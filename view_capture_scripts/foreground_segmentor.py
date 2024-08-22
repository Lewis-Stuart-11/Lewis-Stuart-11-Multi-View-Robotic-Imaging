import cv2
import numpy as np
import os
import json

from abc import ABC, abstractmethod

class SegmentBackground(ABC):
	@abstractmethod
	def segment_foreground(self, rgb_img, depth_img=None):
		return None


class SegmentForegroundWhiteRemoval(SegmentBackground):
	def __init__(self, threshold=[102, 102, 97], min_noise_area = 10000):
		self.threshold = np.array(threshold)
		self.min_noise_area = min_noise_area

	def find_largest_connected_component(self, mask):
		num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))

		sorted_indices = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1] + 1

		# Iterate through connected components and find the largest one covering the center
		for i in sorted_indices:  # Start from 1 to skip the background component

			# Create a new mask with only the current connected component
			current_mask = (labels == i).astype(np.uint8)

			# Check if the current component covers the center of the image
			
			if self.covers_center(current_mask):
				return current_mask * 255

				# If no component covers the center, return an empty mask
		return (labels == 1).astype(np.uint8) * 255

	def covers_center(self, mask):
		# Assuming self is an instance of your class
		h, w = mask.shape[:2]
		middle_start = int(0.425 * w)
		middle_end = int(0.575 * w)

		# Check if any pixel in the middle 10% of the image is within the connected component
		middle_region = mask[:, middle_start:middle_end]
		return np.any(middle_region)

	def remove_small_noise(self, mask, min_noise_area):
		num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))

		# Identify the indices of noise components based on their area
		noise_indices = np.where(stats[:, cv2.CC_STAT_AREA] < min_noise_area)[0]

		# Set the pixels corresponding to noise components to 0
		for index in noise_indices:
			mask[labels == index] = 0

		return mask

	def segment_foreground(self, rgb_img, depth_img=None, mask_img=None):

		# Default white background with dimensions of original image
		white_background = np.ones_like(rgb_img) * 255

		if mask_img is None:

			# Remove all pixels above a certain colour intensity threhold (white pixels)
			mask = cv2.inRange(rgb_img, self.threshold, np.array([255, 255, 255]))

			# Remove small amounts of noise 
			kernel = np.ones((5,5), np.uint8)
			mask = 255 - mask

			mask = cv2.dilate(mask, kernel, iterations=5)

			# Remove noise in the image (clusters with area less than the set minimum)
			mask_cleaned = self.remove_small_noise(mask.copy(), self.min_noise_area)

			# Segment largest cluster in the image (main object)
			mask_largest_component = self.find_largest_connected_component(mask)

			mask_1 = cv2.erode(mask_largest_component, kernel, iterations=5)

			rgb_img_seg_1 = cv2.bitwise_and(rgb_img, rgb_img, mask=mask_1)

			mask_2 = cv2.inRange(rgb_img_seg_1, self.threshold, np.array([255, 255, 255]))

			mask_2 = cv2.bitwise_not(mask_2)

			mask_final = cv2.bitwise_and(mask_1, mask_2)

			kernel = np.ones((2,2), np.uint8)
			mask_final = cv2.erode(mask_final, kernel, iterations=1)
		else:
			mask_final = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)

		# Apply mask
		rgb_img = cv2.bitwise_and(rgb_img, rgb_img, mask=mask_final)
		
		# Set removed background to the colour white
		inverted_mask = cv2.bitwise_not(mask_final)
		white_background = cv2.bitwise_and(white_background, white_background, mask=inverted_mask)
		rgb_img = cv2.add(rgb_img, white_background)

		# Add mask as alpha channel
		rgb_img = np.dstack((rgb_img, mask_final))

		return rgb_img, mask_final

class SegmentForegroundDepth(SegmentBackground):
	def __init__(self, cutoff_ratio=0.4, bin_size=8, num_peaks=3, 
					   min_threshold=10, max_threshold = 200):
		
		self.min_threshold = min_threshold
		self.max_threshold = max_threshold

		self.bin_size = bin_size
		self.num_peaks = num_peaks

		self.cutoff_ratio = cutoff_ratio

	def clean_depth_mask(self, depth_mask):
		kernel = np.ones((5,5), np.uint8)
		depth_mask = cv2.dilate(depth_mask, kernel, iterations=5)

		depth_mask = cv2.erode(depth_mask, kernel, iterations=5)

		kernel = np.ones((7,7), np.uint8)

		depth_mask = cv2.erode(depth_mask, kernel, iterations=8)
		depth_mask = cv2.dilate(depth_mask, kernel, iterations=8)

		kernel = np.ones((5,5), np.uint8)

		depth_mask = cv2.dilate(depth_mask, kernel, iterations=3)

		return depth_mask

	def determine_threshold(self, depth_img):

		hist = np.histogram(depth_img.flatten(), bins = [i for i in range(0, 255, self.bin_size)])[0]

		"""import matplotlib.pyplot as plt

		plt.hist(depth_img.flatten(), bins = [i for i in range(0, 255, bin_size)])

		plt.savefig("/home/psxls7/catkin_ws/src/robotic_view_capture/view_capture_scripts/" + str(self.temp) + ".png")

		plt.close()"""

		gradients = np.gradient(hist)

		min_indices = np.sort(gradients.argsort()[:self.num_peaks+1])[1:self.num_peaks+1]

		largest_difference = np.sort(np.gradient(min_indices).argsort()[-2:])

		foreground_intensity = min_indices[largest_difference[0]] 
		background_intensity = min_indices[largest_difference[1]]

		calculated_threshold =  (foreground_intensity + ((background_intensity-foreground_intensity) * self.cutoff_ratio)) * self.bin_size

		return min(max(calculated_threshold, self.min_threshold), self.max_threshold)

	def segment_foreground(self, rgb_img, depth_img=None):

		if depth_img is None:
			raise Exception("Depth image cannot be None for foreground segmentation")

		threshold = self.determine_threshold(depth_img)

		binary_depth = np.uint8(np.where(depth_img < threshold, 255, 0))

		clean_binary_depth = self.clean_depth_mask(binary_depth)

		foreground_img = np.dstack((rgb_img, clean_binary_depth))

		return foreground_img, clean_binary_depth


""" YOUR CUSTOM CLASS HERE  """


def get_foreground_segmentor(foreground_method):
	if foreground_method == "white_removal":
		return SegmentForegroundWhiteRemoval()

	elif foreground_method == "depth":
		return SegmentForegroundDepth()

		""" ADD YOUR METHOD HERE """
		
	else:
		raise Exception(f"Unknown foreground segmentation type: {foreground_method}")