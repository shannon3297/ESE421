# Author: Shannon Lin Nov 2019 for ESE 421
# 
# run as "python3 coneFinding.py"
# images need to be saved in same directory as code

import cv2
import numpy as np;
import imutils
import re
import matplotlib.pyplot as plt

num_total_files = 15
file_counter = 0
all_expected_x = []
all_expected_y = []
all_actual_x = []
all_actual_y = []
# 'X2small_Y0bigR_west.jpg'
all_files = ['X1small_Y0bigL_east.jpg', 'X2small_Y0bigL_east.jpg', 'X3big_Y1smallR_north.jpg', 'X3big_Y2smallL_south.jpg', 
    'X3big_Y3smallL_south.jpg', 'X7big_Y0smallR_south.jpg', 'X5big_Y0smallR_south.jpg', 'X5big_Y1smallL_south.jpg', 'X5big_Y1smallR_south.jpg', 
    'X5big_Y2smallL_south.jpg', 'X5big_Y2smallR_south.jpg', 'X5big_Y4smallR_south.jpg', 'X5big_Y5smallR_south.jpg', 'X5big_Y6smallL_south.jpg',
    'X5small_Y1bigR_west.jpg', 'X7small_Y1bigR_west.jpg']

# iterate over all images specified
while file_counter != num_total_files:
    # read in image
    filename = all_files[file_counter]
    print("Analyzing ", filename)
    img = cv2.imread(filename)

    # split image into separate BGR channels and equalize each of them
    b, g, r = cv2.split(img)
    red = cv2.equalizeHist(r)
    green = cv2.equalizeHist(g)
    blue = cv2.equalizeHist(b)
    equalized = cv2.merge((blue, green, red))

    # find mostly red pixels, note that array is BGR
    # range based on which way it's facing
    if "north.jpg" in filename:
        lb = np.array([130, 90, 180])
        ub = np.array([165, 120, 220])
    elif ("east" in filename) | ("west" in filename):
        lb = np.array([60, 60, 250])
        ub = np.array([255, 255, 255])
    else:
        lb = np.array([50, 50, 250])
        ub = np.array([230, 230, 255])

    # find mask that has colors within lowerBound and upperBound
    mask = cv2.inRange(equalized, lb, ub)
    cv2.imshow('img', img)
    cv2.waitKey(0)
    # cv2.imshow('equalized', equalized)
    # cv2.waitKey(0)
    # cv2.imshow('mask', mask)
    # cv2.waitKey(0)

    # find contours in mask image
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours  = imutils.grab_contours(contours)
    
    # parse filename to find size of cone based on how far away it is
    size = filename.split('_')
    # determine minimum threshold of contour shape to be used later based on size of cone
    if "small" in size[1]:
        MIN_THRESH = 10
    else:
        MIN_THRESH = 150

    # outline all contours found if area of contour > MIN_THRESH
    for c in contours :
        if cv2.contourArea(c) > MIN_THRESH:
            # draw green contour around cone
            cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            base = tuple(c[c[:, :, 1].argmax()][0])
            print("pixel(x,y): ", base)
            # draw red dot at base of cone
            cv2.circle(img, base, 5, (0, 0, 255), -1)
            cv2.imshow("Cone", img)
            cv2.waitKey(0)

    # convert pixel (x,y) coordinate to image(x,y)
    # 1 pixel = 1.12um
    i_y_p = (base[0] - 240) * 1.12 * 10**-6 # half width is 240
    i_z_p = (base[1] - 135) * 1.12 * 10**-6 # half height is 135

    # compute c_x_p, c_y_p from image(x,y)
    # focal = 400*1.12um, height= 13.4cm
    h = 13.4 * 10**-2
    f = 400 * 1.12 * 10**-6
    c_x_p = h * f / i_z_p 
    c_y_p = h * i_y_p / i_z_p
    print("(c_x_p, c_y_p): ", c_x_p, ",", c_y_p)
    all_expected_x.append(c_x_p)
    all_expected_y.append(c_y_p)

    # compute actual x, y
    # extract number of tiles in x direction and what size (big vs small)
    segments = filename.split("_")
    x_dir = re.split("([0-9])", segments[0])
    small_tile = 40.5 * 10**-2
    big_tile = 92 * 10**-2
    if "big" in x_dir[2]:
        actual_x = (float(x_dir[1]) * big_tile)
    else:
        actual_x = (float(x_dir[1]) * small_tile)

    # same logic as above
    y_dir = re.split("([0-9])", segments[1])
    if "big" in y_dir[2]:
        actual_y = (float(y_dir[1]) * big_tile)
    else:
        actual_y = (float(y_dir[1]) * small_tile)

    print("(actual_x, actual_y): ", actual_x, ",", actual_y)
    all_actual_x.append(actual_x)
    all_actual_y.append(actual_y)
    file_counter = file_counter + 1
    print("\n")

# plot expected and actual (x,y)
plt.plot(all_expected_x, all_expected_y, 'ro')
plt.plot(all_actual_x, all_actual_y, 'b+')
plt.xlabel("x")
plt.ylabel("y")
plt.title('Expected locations in red circles, actuals in blue plus sign')
plt.show()