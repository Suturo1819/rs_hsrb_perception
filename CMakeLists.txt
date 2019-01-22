cmake_minimum_required(VERSION 2.8.3)
project(rs_hsrb_perception)
find_package(catkin REQUIRED robosherlock suturo_perception_msgs geometry_msgs)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_hsrb_perception)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_hsrb_perception/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   CATKIN_DEPENDS suturo_perception_msgs geometry_msgs
   )
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()

# Utilities
add_library(Msgs SHARED src/MsgPublisher.cpp)
target_link_libraries(Msgs ${CATKIN_LIBRARIES})

# Annotators / Fixed Flow Types
rs_add_library(rs_NaiveShapeAnnotator src/NaiveShapeAnnotator.cpp)
target_link_libraries(rs_NaiveShapeAnnotator ${CATKIN_LIBRARIES})
rs_add_library(rs_NaiveObjectRecognition src/NaiveObjectRecognition.cpp)
target_link_libraries(rs_NaiveObjectRecognition ${CATKIN_LIBRARIES})
rs_add_library(rs_SuturoClassifier src/SuturoClassifier.cpp)
target_link_libraries(rs_SuturoClassifier ${CATKIN_LIBRARIES})
rs_add_library(rs_PositionMsg src/PositionMsg.cpp)
target_link_libraries(rs_PositionMsg Msgs)

