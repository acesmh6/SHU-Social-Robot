cmake_minimum_required(VERSION 2.8.3)
project(anthony)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy std_msgs image_transport cv_bridge roslib)

add_message_files(FILES face.msg faceArray.msg touch_in.msg touch_cmd.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge roslib)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

####Executables####
add_executable(face_detector_node src/face_detector.cpp)
target_link_libraries(face_detector_node ${catkin_LIBRARIES})
add_dependencies(face_detector_node ${catkin_EXPORTED_TARGETS})

add_executable(touchscreen_reader_node src/touchscreen_reader.cpp)
target_link_libraries(touchscreen_reader_node ${catkin_LIBRARIES})
add_dependencies(touchscreen_reader_node ${catkin_EXPORTED_TARGETS})

add_executable(touch_command_node src/touch_command.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(touch_command_node ${catkin_LIBRARIES} ${QT_LIBRARIES} opencv_highgui opencv_core)
add_dependencies(touch_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


