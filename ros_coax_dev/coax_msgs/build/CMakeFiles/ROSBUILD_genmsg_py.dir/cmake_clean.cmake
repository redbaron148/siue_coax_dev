FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/coax_msgs/msg/__init__.py"
  "../src/coax_msgs/msg/_CoaxRawControl.py"
  "../src/coax_msgs/msg/_CoaxConstants.py"
  "../src/coax_msgs/msg/_CoaxVersion.py"
  "../src/coax_msgs/msg/_CoaxModes.py"
  "../src/coax_msgs/msg/_CoaxControlParameters.py"
  "../src/coax_msgs/msg/_Coax3DPose.py"
  "../src/coax_msgs/msg/_CoaxTrimMode.py"
  "../src/coax_msgs/msg/_CoaxKeepAlive.py"
  "../src/coax_msgs/msg/_CoaxControl.py"
  "../src/coax_msgs/msg/_CoaxContent.py"
  "../src/coax_msgs/msg/_CoaxState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
