FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/coax_msgs/srv/__init__.py"
  "../src/coax_msgs/srv/_CoaxSetControlParameters.py"
  "../src/coax_msgs/srv/_CoaxSendString.py"
  "../src/coax_msgs/srv/_CoaxSetRawControl.py"
  "../src/coax_msgs/srv/_CoaxGetTrimMode.py"
  "../src/coax_msgs/srv/_CoaxSetControl.py"
  "../src/coax_msgs/srv/_CoaxConfigureComm.py"
  "../src/coax_msgs/srv/_CoaxRequestState.py"
  "../src/coax_msgs/srv/_CoaxReachNavState.py"
  "../src/coax_msgs/srv/_Coax3DSetControlMode.py"
  "../src/coax_msgs/srv/_CoaxSetTimeout.py"
  "../src/coax_msgs/srv/_CoaxSetTrimMode.py"
  "../src/coax_msgs/srv/_CoaxReset.py"
  "../src/coax_msgs/srv/_CoaxGetSensorList.py"
  "../src/coax_msgs/srv/_CoaxSetAckMode.py"
  "../src/coax_msgs/srv/_CoaxGetControlParameters.py"
  "../src/coax_msgs/srv/_CoaxConfigureControl.py"
  "../src/coax_msgs/srv/_CoaxSetVerbose.py"
  "../src/coax_msgs/srv/_CoaxConfigureOAMode.py"
  "../src/coax_msgs/srv/_CoaxGetVersion.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
