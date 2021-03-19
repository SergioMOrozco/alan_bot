from cv2 import cv2
import os
import numpy as np


class ImageManager:
    def __init__(self):
        self.features = []
        self.labels = []

    def start_menu(self):

        directory = input(
            "Please input a directory containg .jpg files that you would like cleaned and made into a dataset: "
        )
        self.create_dataset(directory)
        self.save_dataset(directory)
        self.mini_test(directory)

    def mini_test(self, directory):
        features = np.load(os.path.join(directory, "features.npy"))
        labels = np.load(os.path.join(directory, "labels.npy"))

        cv2.imshow("image", features[0].reshape(33, 47, 1))
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

                # show image to user
                cv2.imshow("image", clean)
                cv2.waitKey(30)

                # input cleaned image into dataset
                self.features.append([clean])

                if label_file:

                    # get label from file
                    label = labels[int(component.strip(".jpg"))]
                    label = label.strip("\n")
                    label = label.split(",")

                    # store label into dataset
                    self.labels.append([label[0], label[1]])

    def save_dataset(self, directory):
        np.save(os.path.join(directory, "features"), np.array(self.features))
        np.save(os.path.join(directory, "labels"), np.array(self.labels))

    @staticmethod
    def clean_image(image_path):
        image = cv2.imread(image_path)

        scaled = ImageManager.scale_image(image)

        # HSV provides more color contrast with yellow lines.
        # It was alot easier to isolate the yellow lines in HSV than in BGR
        hsv = cv2.cvtColor(scaled, cv2.COLOR_BGR2HSV)

        # hsv mask to get yellow from image in medium lighting
        hsv_thresh = cv2.inRange(
            hsv, np.array([2, 0, 94], np.uint8), np.array([24, 255, 185], np.uint8)
        )

        # gaussian blue reduces noise from image. Used to keep the most prominent edges from an image
        hsv_blur = cv2.GaussianBlur(hsv_thresh, (13, 13), 0)

        # edge detection
        canny = cv2.Canny(hsv_blur, 246, 255)

        region = ImageManager.region_of_interest(canny)

        return region

    @staticmethod
    def region_of_interest(img):

        height, width = img.shape

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
        match_mask_color = 255

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
    cv2.destroyAllWindows()
