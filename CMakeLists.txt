cmake_minimum_required(VERSION 2.6)
project (ORB_SLAM)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffriend-injection")

LIST (APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	tf
	image_transport
	cv_bridge
	rosbag_storage
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common octree)

include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIR}
	${CHOLMOD_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

add_library (orb_slam STATIC
	src/Tracking.cc
	src/LocalMapping.cc
	src/LoopClosing.cc
	src/ORBextractor.cc
	src/ORBmatcher.cc
	src/FramePublisher.cc
	src/Converter.cc
	src/MapPoint.cc
	src/KeyFrame.cc
	src/Map.cc
	src/MapPublisher.cc
	src/Optimizer.cc
	src/PnPsolver.cc
	src/Frame.cc
	src/KeyFrameDatabase.cc
	src/Sim3Solver.cc
	src/Initializer.cc
	src/debugList.cpp
)

add_executable (${PROJECT_NAME}
	src/main.cc
)

target_link_libraries (${PROJECT_NAME}
	orb_slam
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
	cholmod
	boost_serialization
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
)

add_executable (mapper
	src/mapper.cc
)

target_link_libraries (mapper
	orb_slam
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
	cholmod
	boost_serialization
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
)

add_executable (tloc
	src/tloc.cpp
)


target_link_libraries (tloc
	boost_serialization
)


add_executable (bag2video
	src/bag2video.cc
)

target_link_libraries (bag2video
	${catkin_LIBRARIES}
)


add_executable (dumpmap
	src/dumpmap.cc
)


target_link_libraries (dumpmap
	orb_slam
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
	cholmod
	boost_serialization
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
)


add_executable (localizer
	src/localizer.cc
)


target_link_libraries (localizer
	orb_slam
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
	cholmod
	boost_serialization
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
)
