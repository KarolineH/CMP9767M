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


# Grid map stuff

def init_map(self):
	# fetch map information
	map = rospy.wait_for_message('/map', OccupancyGrid, 30)
	self.map_resolution = np.around(map.info.resolution, decimals = 3) # in m per pixel
	self.map_width = map.info.width
	self.map_height = map.info.height
	self.map_origin = map.info.origin.position

	# find the list indices that are unoccupied floor space
	floor_indices = []
	for i, cell in enumerate(map.data):
		if cell == 0:
			floor_indices.append(i)

	floor_coordinates = self.indices_to_coordinates(floor_indices)

def grid_indices_to_coordinates(self, indices):
	"""Given indices of some points of interest in the row-major OccupancyGrid map, convert those into x,y coordinates"""
	# turn into np array for elementwise operations:
	indices = np.array(indices)
	column_index = indices % self.map_width
	coordinate_x = (column_index * self.map_resolution) + self.map_resolution/2 + self.map_origin.x
	coordinate_y = (indices - column_index)/self.map_width * self.map_resolution + self.map_resolution/2 + self.map_origin.y
	coord_list = zip(coordinate_x, coordinate_y)



# Tracking weeds as a pointcloud
# Discarded because it scaled like crazy

		# Initialise the to-do list of points that need to be sprayed
		self.weed_cloud = PointCloud()
		self.weed_cloud.header.frame_id = "/map"

		# Set up connections
		self.sub = rospy.Subscriber("/{}/local_weed_poses".format(self.robot_id), PointCloud, self.add_points)
		self.pub = rospy.Publisher('/WeedCloud', PointCloud, queue_size = 2)
		pop_service = rospy.Service('remove_weed', PointCloudOperation, self.remove_points)
	def add_points(self, newPoints):
		for point in newPoints.points:
			self.weed_cloud.points.append(point)
		self.pub.publish(self.weed_cloud)

	def remove_points(self, checkedPoints):
		for point in checkedPoints.points:
			while self.weed_cloud.points.count(point) > 0: # While this point is still somewhere in the list
				self.weed_cloud.points.remove(point)# remove it
		self.pub.publish(self.weed_cloud)



#Distance Matrix between all points in a point cloud
        # Compute distance matrix
        distance_matrix = np.ones((len(waypoints.points), len(waypoints.points))) * 1000
        for i in range(len(waypoints.points)):
            for j in range(len(waypoints.points)):
                if i < j:
                    x_diff = abs(waypoints.spoints[i].x - waypoints.points[j].x)
                    y_diff = abs(waypoints.points[i].y - waypoints.points[j].y)

                    distance_matrix[i,j] = (xdiff**2 + ydiff**2)**0.5 # euclidean distance
                    distance_matrix[j,i] = distance_matrix[i,j] # since the matrix is symmetrical



exploration_topics = []
while not exploration_topics:
    topics = rospy.get_published_topics()
    topic_names = [topic[0] for topic in topics]
    exploration_topics = filter(lambda t: "visualization_marker" in t, topic_names)
    rospy.sleep(0.5)
# wait until the exploration robot has started moving
rospy.wait_for_message(exploration_topics[0], MarkerArray, 1000)



		# Wait for the camera_converter node to specify camera setup details:
		#
		while not rospy.has_param('/{}/FoV_footprint'.format(self.robot_id)):
			rospy.sleep(1)
		self.min_view_dimension = min(rospy.get_param('/{}/FoV_footprint'.format(self.robot_id)))



        # the explorer robot always traverses the exploration area in lines parallel to the y-axis
        # so, wait until the explorer robot is far enough away in x-direction
        # dist = 0
        # while dist < self.min_robot_distance:
        #     relative_pose = self.tfl.transformPose("/{}/base_link".format(self.explorer_robot), explorer_initial_pose)
        #     dist = relative_pose.pose.position.x
        # rospy.loginfo("SPRAYER ROBOT: Now spraying weeds")

        # while explorer_current_goal == explorer_initial_pose:
        #     explorer_current_goal = rospy.wait_for_message("/{}/move_base/current_goal".format(self.explorer_robot), PoseStamped, 1000)
        #     rospy.sleep(1)

        # rospy.sleep(self.head_start) # ALTERNATIVE: give the explorer a timed  head start
