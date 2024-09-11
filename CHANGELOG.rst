^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visual_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2024-09-11)
------------------
* updated changelog
* fixed declare params
* Contributors: Thies Lennart Alff

* fixed declare params
* Contributors: Thies Lennart Alff

0.0.2 (2024-09-11)
------------------
* fixed dataclass defaults
* Contributors: Thies Lennart Alff

0.0.1 (2024-08-03)
------------------
* changed tag tf to tag marker node
* fixed pyquaternion dependency
* make debug topic private
* add px4 bridge, not tested yet
* fixed old package names after launch file/node migration
* added tag pose tf publisher
* fixed formatting
* delete missing node from cmakelist
* switch to c++ implementation of ranges node
* fixed data struct issue
* consider orientation of the tag for yaw estimation
* updated tag sizes for new alu dibond tags
* added code from mjpeg_cam
* added calibration launch file from hippo_common
* handled the case of unknown tag ids
* changed viz node
* add camer name to container name
* publish vision pose to vehicle namespace instead of camera namespace
* renamed launch file
* added virtual range sensor and finalized launch setup
* added todo marker to fix the yaw angle error
* rewritten launch file/subscriptions
* fixed wrong apriltag rotation
* added apriltag node config file
* removed image_decoder
  from this package to add it to the mjpeg_cam package
* implemented simple image decoder
* added image_decoder node
* fixed formatting
* added formatting settings
* removed unresolvable dependency
* fix last prediction time error, fix type error Duration, converting to seconds now
* use the correct frame as suggested by nathalie
* set better initial state
* updated tag poses to match real world
* added use_sim_time and removed tf_publisher
* ros2 conversion
* ...
* fixed numpy dimension bugs
* minor changes
* nice solution for ros params in dataclass
* work in progress
* work in progress
* initial commit
* Contributors: NBauschmann, Thies Lennart Alff
