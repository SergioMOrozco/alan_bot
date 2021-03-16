from cv2 import cv2
import os
import numpy as np

cv2.namedWindow("hsv")
cv2.namedWindow("canny")


image = cv2.imread(
    "/home/sorozco0612/dev/alan_bot/data_processing/road_data/test/30.jpg"
)

# ---------------------- Calibration ------------------------

# def nothing(x):
#    pass

# sliders for blurring calibration

# cv2.createTrackbar("blur", "canny", 1, 31, nothing)

# sliders for hsv color masking calibration

# cv2.createTrackbar("h_low", "hsv", 0, 179, nothing)
# cv2.createTrackbar("h_high", "hsv", 0, 179, nothing)
# cv2.createTrackbar("s_low", "hsv", 0, 255, nothing)
# cv2.createTrackbar("s_high", "hsv", 0, 255, nothing)
# cv2.createTrackbar("v_low", "hsv", 0, 255, nothing)
# cv2.createTrackbar("v_high", "hsv", 0, 255, nothing)

# sliders for canny thresholding calibration

# low_canny = cv2.createTrackbar("Low Threshold", "canny", 0, 255, nothing)
# up_canny = cv2.createTrackbar("Upper Threshold", "canny", 0, 255, nothing)

# -----------------------------------------------------------


def region_of_interest(img, vertices):

    # create empty mas
    mask = np.zeros_like(img)

    # mask only works for binary images
    match_mask_color = 255

    # filly polygon with white given vertices
    cv2.fillPoly(mask, vertices, match_mask_color)

    # only take pixels where both the mask and image are white
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image


# HSV provides more color contrast with yellow lines.
# It was alot easier to isolate the yellow lines in HSV than in BGR
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


height = image.shape[0]
width = image.shape[1]

region_of_interest_vertices = [
    (0, height),
    (0, height * 3 / 4),
    (width / 5, height / 3),
    (width * 4 / 5, height / 3),
    (width, height * 3 / 4),
    (width, height),
]


while True:

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

    # ---------------------- Calibration ------------------------

    # sliders for hsv color masking calibration

    # h_low = cv2.getTrackbarPos("h_low", "hsv")
    # h_high = cv2.getTrackbarPos("h_high", "hsv")
    # s_low = cv2.getTrackbarPos("s_low", "hsv")
    # s_high = cv2.getTrackbarPos("s_high", "hsv")
    # v_low = cv2.getTrackbarPos("v_low", "hsv")
    # v_high = cv2.getTrackbarPos("v_high", "hsv")

    # sliders for blurring calibration

    # blur = cv2.getTrackbarPos("blur", "canny")

    # if blur % 2 == 0:
    #    blur += 1

    # sliders for canny thresholding calibration

    # lower_canny = cv2.getTrackbarPos("Low Threshold", "canny")
    # upper_canny = cv2.getTrackbarPos("Upper Threshold", "canny")

    # -----------------------------------------------------------

    # hsv mask to get yellow from image in medium lighting
    hsv_thresh = cv2.inRange(
        hsv, np.array([2, 0, 94], np.uint8), np.array([24, 255, 185], np.uint8)
    )

    # gaussian blue reduces noise from image. Used to keep the most prominent edges from an image
    hsv_blur = cv2.GaussianBlur(hsv_thresh, (13, 13), 0)

    # edge detection
    canny = cv2.Canny(hsv_blur, 246, 255)

    region = region_of_interest(
        canny, np.array([region_of_interest_vertices], np.int32)
    )

    cv2.imshow("image", image)
    cv2.imshow("region", region)

cv2.destroyAllWindows()
