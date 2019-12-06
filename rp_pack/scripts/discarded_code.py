# Contour extraction

blurred = cv2.dilate(cv2.erode(cv2.GaussianBlur(image/255, (3,3), 1), np.ones((5,5))), np.ones((9, 9)),5)*255

im, contours, hierarchy = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for c in contours:
    # calculate moments for each contour
    M = cv2.moments(c)
    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    # display the image
cv2.imshow("Image", image)
cv2.waitKey(0)
pdb.set_trace()



# Blob detection
self.params = cv2.SimpleBlobDetector_Params()
self.params.filterByColor = False
self.params.filterByArea = False
self.params.filterByCircularity = False,
self.params.filterByInertia = False,
self.params.filterByConvexity = False
self.detector = cv2.SimpleBlobDetector_create(self.params)

keypoints = self.detector.detect(image)
im_with_keypoints = cv2.drawKeypoints(original, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow("mask", image)
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
