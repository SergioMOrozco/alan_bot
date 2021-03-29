import numpy as np
import matplotlib
from cv2 import cv2
from matplotlib import pyplot as plt
from image_manager import ImageManager

matplotlib.use("TkAgg")


def nothing(x):
    pass


cv2.namedWindow("track_bars")

cv2.createTrackbar("h_low", "track_bars", 0, 179, nothing)
cv2.createTrackbar("h_high", "track_bars", 0, 179, nothing)
cv2.createTrackbar("s_low", "track_bars", 0, 255, nothing)
cv2.createTrackbar("s_high", "track_bars", 0, 255, nothing)
cv2.createTrackbar("v_low", "track_bars", 0, 255, nothing)
cv2.createTrackbar("v_high", "track_bars", 0, 255, nothing)
cv2.createTrackbar("canny_low", "track_bars", 0, 255, nothing)
cv2.createTrackbar("canny_high", "track_bars", 0, 255, nothing)

image = cv2.imread(
    "/home/sorozco0612/dev/alan_bot/data_processing/road_data/Tracks/TrackOneReverse/1966.jpg"
)

scaled = ImageManager.scale_image(image)

while True:

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    h_low = cv2.getTrackbarPos("h_low", "track_bars")
    s_low = cv2.getTrackbarPos("s_low", "track_bars")
    v_low = cv2.getTrackbarPos("v_low", "track_bars")

    h_high = cv2.getTrackbarPos("h_high", "track_bars")
    s_high = cv2.getTrackbarPos("s_high", "track_bars")
    v_high = cv2.getTrackbarPos("v_high", "track_bars")

    canny_low = cv2.getTrackbarPos("canny_low", "track_bars")
    canny_high = cv2.getTrackbarPos("canny_high", "track_bars")

    # HSV provides more color contrast with yellow lines.
    # It was alot easier to isolate the yellow lines in HSV than in BGR
    hsv = cv2.cvtColor(scaled, cv2.COLOR_BGR2HSV)

    # hsv mask to get yellow from image in medium lighting
    hsv_thresh = cv2.inRange(
        hsv,
        np.array([h_low, s_low, v_low], np.uint8),
        np.array([h_high, s_high, v_high], np.uint8),
    )

    # gaussian blue reduces noise from image. Used to keep the most prominent edges from an image
    hsv_blur = cv2.GaussianBlur(hsv_thresh, (13, 13), 0)

    # edge detection
    canny = cv2.Canny(hsv_blur, canny_low, canny_high)

    region = ImageManager.region_of_interest(canny)

    images = np.concatenate((hsv_thresh, hsv_blur, canny, region), axis=1)

    cv2.imshow("image", image)
    cv2.imshow("images", images)
