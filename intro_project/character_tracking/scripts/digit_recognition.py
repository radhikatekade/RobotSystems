import cv2
# from process_image import get_output_image
import joblib
from skimage.feature import hog
import numpy as np

# Load the classifier
clf = joblib.load("digits_cls.pkl")

cap = cv2.VideoCapture(0)


def getOutputDigit(f):
    im = cv2.imread(f)

    # Convert to grayscale and apply Gaussian filtering
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # Threshold the image
    ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)

    # Find contours in the image
    ctrs, hier = cv2.findContours(
        im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get rectangles contains each contour
    rects = [cv2.boundingRect(ctr) for ctr in ctrs]

    # For each rectangular region, calculate HOG features and predict
    # the digit using Linear SVM.
    for rect in rects:
        # Draw the rectangles
        cv2.rectangle(im, (rect[0], rect[1]), (rect[0] +
                      rect[2], rect[1] + rect[3]), (0, 255, 0), 3)

        # Make the rectangular region around the digit
        roi = im_th[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]

        # Resize the image
        roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
        roi = cv2.dilate(roi, (3, 3))

        # Calculate the HOG features
        roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(
            14, 14), cells_per_block=(1, 1), visualize=False)

        nbr = clf.predict(np.array([roi_hog_fd], 'float64'))

        cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),
                    cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)

        # Calculate the center 
        center_x = rect[0] + (rect[2]/2)
        center_y = rect[1] + (rect[3]/2)

        return im, nbr, center_x, center_y

if __name__ == '__main__':
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        ret, frame = cap.read()
        frame_copy = frame.copy()
        cv2.imwrite('test.jpg', frame_copy)
        output_img = getOutputDigit('test.jpg')
        cv2.imshow('Detected text', output_img)

        c = cv2.waitKey(1)
        if c == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
