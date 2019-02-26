ROS node to split side-by-side stereo
=====================================

Subscribes to an image stream of side-by-side stereo where each image
message consists of a left and right image concatenated to form a single
double-wide image.  This node splits the incoming image down the middle
and republishes each half as stereo/left and stereo/right images.

This is a modified version of public domain code posted by user PeteBlackerThe3rd
in response to my question on ROS Answers:
https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/

Parameters
----------

- input\_image\_topic -- ROS topic of concantenated stereo images.
- left\_output\_image\_topic -- defaults to /stereo/left/image\_raw
- right\_output\_image\_topic -- defaults to /stereo/right/image\_raw
- output\_width -- output images are rescaled to this width. If 0 or not set, it
  will be 1/2 of the input\_image\_topic width.
- output\_height -- output images are rescaled to this height.  If 0 or not set, it
  will be the same as the input\_image\_topic height.

License
-------

MIT
