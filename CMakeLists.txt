cmake_minimum_required(VERSION 3.0.2)
project(abc_swarm)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs dynamic_reconfigure)
catkin_python_setup()
generate_dynamic_reconfigure_options(config/tf_pid.cfg)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure)

# Original teaching nodes remain at their original paths, with no new files.
file(GLOB COURSE_NODES nodes/*.py)
catkin_install_python(PROGRAMS ${COURSE_NODES}
  scripts/leader_follower_tracker.py
  scripts/pid_tracker.py
  scripts/preset_path_tracker.py
  scripts/demo_runtime.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.rst LICENSE dependencies.repos dependencies-rmtt.repos requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_main_layout.py)
  catkin_add_nosetests(tests/test_leader_follower.py)
  catkin_add_nosetests(tests/test_preset_path.py)
  catkin_add_nosetests(tests/test_demo_entrypoints.py)
endif()
