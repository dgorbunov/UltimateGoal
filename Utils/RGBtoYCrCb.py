import cv2 as cv
import numpy as np
input = np.uint8([[[255,0,0]]])
output = cv.cvtColor(input,cv.COLOR_RGB2YCrCb)
print(output)

