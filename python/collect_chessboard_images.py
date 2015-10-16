import numpy as np
import cv2

BOARD_SHAPE = (9,6)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cv2.namedWindow("preview")
cv2.namedWindow("overlay")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, img = vc.read()
else:
    rval = False

image_num = 0
while rval:
    rval, img = vc.read()
    cv2.imshow("preview", img)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, BOARD_SHAPE)

    # If found, add object points, image points (after refining them)
    if ret == True:

        cv2.imwrite("images/board{:04d}.jpg".format(image_num), gray)
        image_num += 1

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, BOARD_SHAPE, corners2,ret)
        cv2.imshow("overlay", img)

    key = cv2.waitKey(500)
    if key == 27: # exit on ESC
        break

# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1])

# print "mtx:", mtx
# print "dist:", dist

# h,  w = img.shape[:2]
# newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

# # undistort
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# cv2.imshow("overlay", dst)

# while True:
#     key = cv2.waitKey(20)
#     if key == 27: # exit on ESC
#         break

cv2.destroyWindow("preview")
cv2.destroyWindow("overlay")

