from cv2 import cv2
import os
import numpy as np
from sklearn.model_selection import train_test_split
import h5py
import H5pyHelper
from datetime import datetime


class ImageManager:
    def __init__(self):
        self.set_default()

    def set_default(self):
        self.features = []
        self.labels = []
        self.i = 0
        self.batches = 5000

    def start_menu(self):
        self.directory = "/home/sorozco0612/dev/alan_bot/data_processing/road_data/"

        self.create_dataset(self.directory)

        if not (self.features == []) and not (self.labels == []):
            x_train, x_test, y_train, y_test = train_test_split(
                np.array(self.features), np.array(self.labels)
            )
            # self.append_to_dataset(temp_filename)
            H5pyHelper.append_to_dataset(
                os.path.join(self.directory, "data.h5"),
                x_train,
                "x_train",
            )
            H5pyHelper.append_to_dataset(
                os.path.join(self.directory, "data.h5"),
                y_train,
                "y_train",
            )
            H5pyHelper.append_to_dataset(
                os.path.join(self.directory, "data.h5"),
                x_test,
                "x_test",
            )
            H5pyHelper.append_to_dataset(
                os.path.join(self.directory, "data.h5"),
                y_test,
                "y_test",
            )

        seed = datetime.now()
        H5pyHelper.shuffle_dataset(
            os.path.join(self.directory, "data.h5"), "x_train", seed
        )
        H5pyHelper.shuffle_dataset(
            os.path.join(self.directory, "data.h5"), "y_train", seed
        )

    def create_dataset(self, search_directory):
        label_file = None

        # get label file
        if os.path.exists(search_directory + "labels.txt"):
            label_file = open(search_directory + "labels.txt")
            labels = label_file.readlines()

        # find all .jpg files within a given folder
        for component in os.listdir(search_directory):

            path = search_directory + component

            # recursively find .jpg files in sub directories
            if os.path.isdir(path):
                print(path)
                self.create_dataset(path + "/")

            elif component.endswith(".jpg"):

                clean = ImageManager.clean_image(path)

                if label_file:

                    if self.i == self.batches:
                        x_train, x_test, y_train, y_test = train_test_split(
                            np.array(self.features), np.array(self.labels)
                        )
                        # self.append_to_dataset(temp_filename)
                        H5pyHelper.append_to_dataset(
                            os.path.join(self.directory, "data.h5"),
                            x_train,
                            "x_train",
                        )
                        H5pyHelper.append_to_dataset(
                            os.path.join(self.directory, "data.h5"),
                            y_train,
                            "y_train",
                        )
                        H5pyHelper.append_to_dataset(
                            os.path.join(self.directory, "data.h5"),
                            x_test,
                            "x_test",
                        )
                        H5pyHelper.append_to_dataset(
                            os.path.join(self.directory, "data.h5"),
                            y_test,
                            "y_test",
                        )
                        self.i = 0
                        self.features = []
                        self.labels = []

                    # input cleaned image into dataset
                    self.features.append(clean)

                    # get label from file
                    label = labels[int(component.strip(".jpg"))]
                    label = label.strip("\n")
                    label = float(label)

                    self.i += 1

                    # store label into dataset
                    self.labels.append(label)

    @staticmethod
    def clean_image(image_path):
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        image = cv2.GaussianBlur(image, (3, 3), 0)
        scaled = cv2.resize(image, (200, 66))

        # scaled = ImageManager.scale_image(image)
        scaled_pixels = ImageManager.scale_pixels(scaled)

        # HSV provides more color contrast with yellow lines.
        # It was alot easier to isolate the yellow lines in HSV than in BGR
        # hsv = cv2.cvtColor(scaled, cv2.COLOR_BGR2HSV)

        # hsv mask to get yellow from image in medium lighting
        # hsv_thresh = cv2.inRange(
        #    hsv, np.array([0, 0, 63], np.uint8), np.array([107, 243, 255], np.uint8)
        # )

        ## gaussian blue reduces noise from image. Used to keep the most prominent edges from an image
        # hsv_blur = cv2.GaussianBlur(hsv_thresh, (13, 13), 0)

        ## edge detection
        ## canny = cv2.Canny(hsv_blur, 246, 255)
        # canny = cv2.Canny(hsv_blur, 0, 255)

        # region = ImageManager.region_of_interest(scaled)

        # give single color channel. Needed for 2DConv
        # region = region.reshape(list(region.shape) + [1])

        return scaled_pixels

    @staticmethod
    def region_of_interest(img):

        height, width, channels = img.shape

        vertices = np.array(
            [
                [
                    (0, height),
                    (0, height * 3 / 4),
                    (width / 5, height / 3),
                    (width * 4 / 5, height / 3),
                    (width, height * 3 / 4),
                    (width, height),
                ]
            ],
            np.int32,
        )

        # create empty mas
        mask = np.zeros_like(img)

        # mask only works for binary images
        match_mask_color = (255, 255, 255)

        # filly polygon with white given vertices
        cv2.fillPoly(mask, vertices, match_mask_color)

        # only take pixels where both the mask and image are white
        return cv2.bitwise_and(img, mask)

    @staticmethod
    def scale_image(image, scale=60):

        if scale == 100:
            return image

        # calculate the 50 percent of original dimensions
        width = int(image.shape[1] * scale / 100)
        height = int(image.shape[0] * scale / 100)

        # resize image
        return cv2.resize(image, (width, height))

    @staticmethod
    def scale_pixels(image):
        normalized = ImageManager.normalize_pixels(image)
        centered = ImageManager.center_pixels(normalized)
        return centered

    @staticmethod
    def normalize_pixels(image):
        pixels = image.astype("float32")
        pixels /= 255.0
        return pixels

    @staticmethod
    def center_pixels(image):
        pixels = image - image.mean()
        return pixels


if __name__ == "__main__":
    manager = ImageManager()
    manager.start_menu()
