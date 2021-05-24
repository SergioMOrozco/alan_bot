from cv2 import cv2
import os
import numpy as np
from sklearn.model_selection import train_test_split
import h5py

filename = "data.h5"


class ImageManager:
    def __init__(self):
        self.features = []
        self.labels = []
        self.i = 0
        self.batches = 500

    def start_menu(self):

        self.directory = input(
            "Please input a directory containg .jpg files that you would like cleaned and made into a dataset: "
        )

        self.f = None

        self.create_dataset(self.directory)

        if not (self.features == []) and not (self.labels == []):
            self.batches = len(self.features)
            self.append_to_dataset()

        # self.save_dataset(directory)
        # self.mini_test(directory)

    def mini_test(self, directory):
        features = np.load(os.path.join(directory, "features.npy"))
        labels = np.load(os.path.join(directory, "labels.npy"))

        cv2.imshow("image", features[0])
        print(labels[0])
        cv2.waitKey(1000)

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
                self.create_dataset(path + "/")

            elif component.endswith(".jpg"):

                clean = ImageManager.clean_image(path)

                if label_file:

                    if self.i == self.batches:
                        self.i = 0
                        self.append_to_dataset()

                    # show image to user
                    # cv2.imshow("image", clean)
                    # cv2.waitKey(30)

                    # input cleaned image into dataset
                    self.features.append(clean)

                    # get label from file
                    label = labels[int(component.strip(".jpg"))]
                    label = label.strip("\n")
                    label = label.split(",")
                    label = [float(item) for item in label]

                    self.i += 1

                    # store label into dataset
                    self.labels.append([label[0], label[1]])

    def append_to_dataset(self):
        x_train, x_test, y_train, y_test = train_test_split(
            np.array(self.features), np.array(self.labels)
        )

        print("Batch: ", self.batches)
        if self.f is None:
            self.f = h5py.File(os.path.join(self.directory, filename), "w")
            train_x_data = self.f.create_dataset(
                "x_train",
                data=x_train,
                compression="gzip",
                chunks=True,
                maxshape=(None, 432, 614, 3),
            )
            train_y_data = self.f.create_dataset(
                "y_train",
                data=y_train,
                compression="gzip",
                chunks=True,
                maxshape=(None, 2),
            )
            test_x_data = self.f.create_dataset(
                "x_test",
                data=x_test,
                compression="gzip",
                chunks=True,
                maxshape=(None, 432, 614, 3),
            )
            test_y_data = self.f.create_dataset(
                "y_test",
                data=y_test,
                compression="gzip",
                chunks=True,
                maxshape=(None, 2),
            )
            print("'x_train' chunk has shape:{}".format(self.f["x_train"].shape))
            print("'y_train' chunk has shape:{}".format(self.f["y_train"].shape))
            print("'x_test' chunk has shape:{}".format(self.f["x_test"].shape))
            print("'y_test' chunk has shape:{}".format(self.f["y_test"].shape))
            print("====================")
            self.f.close()

        else:
            hf = h5py.File(os.path.join(self.directory, filename), "a")

            hf["x_train"].resize(hf["x_train"].shape[0] + x_train.shape[0], axis=0)
            hf["x_train"][-x_train.shape[0] :] = x_train

            hf["y_train"].resize(hf["y_train"].shape[0] + y_train.shape[0], axis=0)
            hf["y_train"][-y_train.shape[0] :] = y_train

            hf["x_test"].resize(hf["x_test"].shape[0] + x_test.shape[0], axis=0)
            hf["x_test"][-x_test.shape[0] :] = x_test

            hf["y_test"].resize(hf["y_test"].shape[0] + y_test.shape[0], axis=0)
            hf["y_test"][-y_test.shape[0] :] = y_test

            print("'x_train' chunk has shape:{}".format(hf["x_train"].shape))
            print("'y_train' chunk has shape:{}".format(hf["y_train"].shape))
            print("'x_test' chunk has shape:{}".format(hf["x_test"].shape))
            print("'y_test' chunk has shape:{}".format(hf["y_test"].shape))
            print("====================")

            hf.close()

        self.features = []
        self.labels = []

    def save_dataset(self, directory):
        x_train, x_test, y_train, y_test = train_test_split(
            np.array(self.features), np.array(self.labels)
        )
        np.save(os.path.join(directory, "train_features"), x_train)
        np.save(os.path.join(directory, "train_labels"), y_train)
        np.save(os.path.join(directory, "test_features"), x_test)
        np.save(os.path.join(directory, "test_labels"), y_test)

    @staticmethod
    def clean_image(image_path):
        image = cv2.imread(image_path)

        scaled = ImageManager.scale_image(image)

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

        return scaled

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


if __name__ == "__main__":
    manager = ImageManager()
    manager.start_menu()
    # manager.mini_test("/home/sorozco0612/dev/alan_bot/data_processing/road_data/")
    cv2.destroyAllWindows()
