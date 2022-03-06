import pytesseract
import cv2
img = cv2.imread('normal_text.png')
img = cv2.resize(img, (600, 360))
print(pytesseract.image_to_boxes(img))
cv2.imshow('Result', img)
cv2.waitKey(0)
