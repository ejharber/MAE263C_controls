# import the necessary packages
import cv2
import imutils
import numpy as np


def process_image(image):
    # convert image to grayscale, and blur it slightly
    crop_length = 25
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
    print(imx, imy)
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
    x = [0, 80, 160, imx]
    y = [0, 101-crop_length, 214-crop_length, imy]
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
    ext = 80  # size of the square that will represent each section
    aug = 10  # augmentation factor (since normalized area is relatively small)
    visual = np.ones((ylen * ext, xlen * ext, 1), dtype=np.uint8)  # image array
    normalized_areas = np.zeros((ylen, xlen))
    for i in range(ylen):
        for j in range(xlen):
            # each square equal to area of contours divided by area of section times some augmentation factor
            normalized_areas[i, j] = sections[i, j] / section_areas[i, j]
            temp = (1-normalized_areas[i, j]) * 255 * aug
            visual[i * ext:((i + 1) * ext), j * ext:((j + 1) * ext)] = temp
    # print(normalized_areas)

    # return thresholded image, image with contours drawn on it, "force" visual, and the normalized areas that make the
    # "force" visual
    thresh = 255 - thresh
    return thresh, clone, visual, normalized_areas


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
