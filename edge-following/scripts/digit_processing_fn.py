# import the necessary packages
import cv2
import imutils
import numpy as np


def process_image(image):
    # convert image to grayscale, and blur it slightly
    crop_length = 30
    image = image.copy()[crop_length:, :, :]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray = gray.copy()[25:, :]
    # (image, kernel size, standard deviation of gaussian distribution (0 means it's calculated by the function))
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    blurred = cv2.blur(gray, (5,5))  # average blur instead

    # Threshold the image
    # (image, output threshold value, adaptive threshold method, threshold method, subregion pixel size, offset)
    # alternate adaptive threshold method: cv2.ADAPTIVE_THRESH_MEAN_C
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 29, 6.5)
    # further process the image with an opening to remove artifacts
    # kernelSize = (3, 3)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernelSize)
    # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # find external contours in the image
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    # clone = cv2.cvtColor(gray.copy(), cv2.COLOR_GRAY2RGB)  # Convert back to RGB
    clone = image.copy()

    # loop over the contours and collect the areas (also draw the contours onto the image copy)
    imx = len(gray[1, :])
    imy = len(gray[:, 1])
    # print(imx, imy)
    areas = np.zeros((imy, imx))

    for (i, c) in enumerate(cnts):
        # compute the area and the perimeter of the contour
        area = cv2.contourArea(c)
        # perimeter = cv2.arcLength(c, True)

        # compute the center of the contour
        m = cv2.moments(c)
        cx = int(m["m10"] / (m["m00"] + 0.0000001))
        cy = int(m["m01"] / (m["m00"] + 0.0000001))
        areas[cy, cx] = area

        # draw the contour on the copy
        cv2.drawContours(clone, [c], -1, (255, 0, 255), 1)

    # calculate the total contour areas of each defined section of the image
    # section divisions
    xcrop = 10
    x = [0, 80-xcrop, 160-xcrop, imx-xcrop]
    y = [0, 101-crop_length, 214-crop_length, imy]
    # prin(x)
    # print(y)
    # y = [0, 101, 214, imy]
    xlen = len(x) - 1
    ylen = len(y) - 1
    sections = np.zeros((ylen, xlen))
    section_areas = sections.copy()

    for i in range(ylen):
        for j in range(xlen):
            # total area of a section
            section_areas[i, j] = ((x[j + 1] - x[j]) * (y[i + 1] - y[i]))
            # total area of the contours in a section
            sections[i, j] = sum(sum(areas[y[i]:y[i + 1], x[j]:x[j + 1]]))

    # debugging code
    # print(sections)
    # print(section_areas)

    # display the normalized area that the contours take up in each section
    aug = 10  # augmentation factor (since normalized area is relatively small)
    visual = np.ones((imy, imx, 1), dtype=np.uint8)  # image array
    
    normalized_areas = np.zeros((ylen, xlen))
    for i in range(ylen):
        for j in range(xlen):
            # each square equal to area of contours divided by area of section times some augmentation factor
            normalized_areas[i, j] = sections[i, j] / section_areas[i, j]
            temp = (1-normalized_areas[i, j]) * 255 * aug
            visual[y[i]:y[i+1], x[j]:x[j+1],0] = temp
    # print(normalized_areas)

    # return thresholded image, image with contours drawn on it, "force" visual, and the normalized areas that make the
    # "force" visual
    thresh = 255 - thresh
    color = 0
    thickness = 1
    thresh = cv2.line(thresh, (x[1], y[0]), (x[1], y[ylen]), color, thickness)
    thresh = cv2.line(thresh, (x[2], y[0]), (x[2], y[ylen]), color, thickness)
    thresh = cv2.line(thresh, (x[3], y[0]), (x[3], y[ylen]), color, thickness)
    thresh = cv2.line(thresh, (x[0], y[1]), (x[xlen], y[1]), color, thickness)
    thresh = cv2.line(thresh, (x[0], y[2]), (x[xlen], y[2]), color, thickness)
    thresh = cv2.line(thresh, (x[0], y[3]), (x[xlen], y[3]), color, thickness)
    force = normalized_areas*255

    dist_fm_center = 0
    center_x = int((x[2] - x[1]) / 2 + x[1])
    center_y = int((y[2] - y[1]) / 2 + y[1])

    for (i, c) in enumerate(cnts):
        # compute the area and the perimeter of the contour
        area = cv2.contourArea(c)
        # perimeter = cv2.arcLength(c, True)

        # compute the center of the contour
        m = cv2.moments(c)
        cx = int(m["m10"] / (m["m00"] + 0.0000001))
        cy = int(m["m01"] / (m["m00"] + 0.0000001))
        areas[cy, cx] = area

        if cx > x[1] and cx < x[2] and cy > y[1] and cy < y[2] and area > 40:
            dist_fm_center += abs(cx - center_x)

    return thresh, clone, visual, force, dist_fm_center, 0


def main():
    # load the image
    imfile = 'DIGIT1.png'
    image = cv2.imread(imfile)
    thresh, contours, visual, normalized_areas = process_image(image)
    print(normalized_areas)
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.imshow("OpenCV Mean Thresh", thresh)
    cv2.waitKey(0)
    cv2.imshow("Areas", contours)
    cv2.waitKey(0)
    cv2.imshow("Average Areas", visual)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
