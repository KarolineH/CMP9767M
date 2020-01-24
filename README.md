<h1>Robot Programming Assessment Item 1</h1>
<h2>Karoline Heiwolt</h2>

FOCUS AREA: Coordination

HOW TO RUN:
  -Launch "roslaunch rp_pack assignment.launch"
  -Once everything runs, use the "Publish Point" button in the rviz interface to define the corners of a field the robots should work on
  -If the sprayer robot ever sits in one place for a long time, you can "rosservice call /next_weed" to manually send it to its next goal


SUMMARY:
In my implementation the task is divided into two sub-tasks. An explorer robot traverses the field along a path that ensures that the robot's camera has captured the entire area (coverage algorithm tuned to the 'footprint' of the image). It maintains a map of detected weeds and provides a to-do-list for the sprayer robot, who moves around the field to spray the weeds one by one, while staying out of the way of the explorer robot. This implementation also works on different farms, the user defines the field area with a few clicks in the GUI, the exploration path adapts to the new area.

NOTE:
Please note that the "heatmap_jade_devel" folder is not my original work, it is included in the source files here, because the release version did not work for me and I had to make some changes in the package to integrate it into my code.
