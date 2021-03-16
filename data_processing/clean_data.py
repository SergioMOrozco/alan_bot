from cv2 import cv2
import os
import numpy as np


class DataCleaner:
    def __init__(self):
        self.scale = 60

    def clean(self, search_directory):
        for component in os.listdir(search_directory):
            path = search_directory + component
            if os.path.isdir(path):

                self.clean(path + "/")

            elif component.endswith(".jpg"):

                clean = self.clean_image(path)

                cv2.imshow("image", clean)
                cv2.waitKey(30)

    def clean_image(self, image_path):
        image = cv2.imread(image_path)

        scaled = self.scale_image(image)

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

        region = self.region_of_interest(canny)

        return region

    def region_of_interest(self, img):

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

    def scale_image(self, image):

        # calculate the 50 percent of original dimensions
        width = int(image.shape[1] * self.scale / 100)
        height = int(image.shape[0] * self.scale / 100)

        # resize image
        return cv2.resize(image, (width, height))


clean = DataCleaner()
clean.clean("/home/sorozco0612/dev/alan_bot/data_processing/")
cv2.destroyAllWindows()
