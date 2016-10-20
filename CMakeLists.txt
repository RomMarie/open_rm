cmake_minimum_required(VERSION 2.8.3)
project(open_rm)


find_package(catkin REQUIRED COMPONENTS roscpp
    image_geometry
    cv_bridge tf laser_geometry image_transport roslaunch gencpp dynamic_reconfigure)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5Core REQUIRED)

FILE(GLOB launchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${launchFiles})

FILE(GLOB cfgFiles "cfg/*.cfg")
add_custom_target(cfg SOURCES ${cfgFiles})

find_package( Qt4 4.5.0 COMPONENTS QtCore QtXml QtGui REQUIRED )
include(${QT_USE_FILE})

generate_dynamic_reconfigure_options( cfg/mask_to_skel.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
algebre
axemedian
blobs
geometrie
graphe
intervalles
mapping
odometrie
sigvis
transfdist
vision
visionomni
  DEPENDS OpenCV Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRECTORIES} ${OpenCV_INCLUDE_DIRS})


add_library(algebre
include/open_rm/Algebre/polynomes.h src/Algebre/polynomes.cpp
include/open_rm/Algebre/equations.h src/Algebre/equations.cpp)
target_link_libraries(algebre geometrie intervalles ${OpenCV_LIBRARIES})

add_library(axemedian
include/open_rm/AxeMedian/fdeltama.h src/AxeMedian/fdeltama.cpp
include/open_rm/AxeMedian/deltama.h src/AxeMedian/deltama.cpp
include/open_rm/AxeMedian/operations.h src/AxeMedian/operations.cpp)
target_link_libraries(axemedian transfdist ${OpenCV_LIBRARIES})

add_library(blobs include/open_rm/Blobs/blobs.h src/Blobs/blobs.cpp)
target_link_libraries(blobs ${OpenCV_LIBRARIES})

add_library(geometrie
include/open_rm/Geometrie/distances.h src/Geometrie/distances.cpp
include/open_rm/Geometrie/bezier.h src/Geometrie/bezier.cpp)
target_link_libraries(geometrie intervalles ${OpenCV_LIBRARIES})

add_library(graphe include/open_rm/Graphe/graphe.h
include/open_rm/Graphe/grapheGround.h
include/open_rm/Graphe/graphetopview.h
include/open_rm/Graphe/robotvirtuel.h
src/Graphe/robotvirtuel.cpp)
target_link_libraries(graphe ${OpenCV_LIBRARIES})

add_library(intervalles include/open_rm/Intervalles/intervalles.h src/Intervalles/intervalles.cpp)
target_link_libraries(intervalles ${OpenCV_LIBRARIES})

add_library(mapping include/open_rm/Mapping/localOccGrid.h src/Mapping/localOccGrid.cpp
include/open_rm/Mapping/localTopView.h src/Mapping/localTopView.cpp)
target_link_libraries(mapping ${OpenCV_LIBRARIES})

add_library(odometrie include/open_rm/Odometrie/compasvisuel.h src/Odometrie/compasvisuel.cpp)
target_link_libraries(mapping ${OpenCV_LIBRARIES})

add_library(sigvis
include/open_rm/SignaturesVisuelles/signaturesvisuelles.h src/SignaturesVisuelles/signaturesvisuelles.cpp)
target_link_libraries(sigvis intervalles ${OpenCV_LIBRARIES})

add_library(transfdist
include/open_rm/TransfDistance/transfdistance.h src/TransfDistance/transfdistance.cpp)
target_link_libraries(transfdist ${OpenCV_LIBRARIES})

add_library(vision include/open_rm/Vision/calibrage.h src/Vision/calibrage.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES})

add_library(visionomni
include/open_rm/VisionOmni/sphereeq.h src/VisionOmni/sphereeq.cpp
include/open_rm/VisionOmni/birdeyeview.h src/VisionOmni/birdeyeview.cpp
include/open_rm/VisionOmni/omnidma.h src/VisionOmni/omnidma.cpp
include/open_rm/VisionOmni/paramscalib.h src/VisionOmni/paramscalib.cpp)
target_link_libraries(visionomni ${QT_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud lisant un répertoire d'images et le publiant sur le réseau ROS
add_executable(rm_dir_to_image src/Noeuds/rm_dir_to_image.cpp)
target_link_libraries(rm_dir_to_image ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud publiant l'image acquise par une caméra sur le réseau ROS
add_executable(rm_cam_to_image src/Noeuds/rm_cam_to_img.cpp)
target_link_libraries(rm_cam_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud simulant une caméra virtuelle sur le réseau ROS
add_executable(rm_virtualcam_to_image src/Noeuds/rm_virtualcam_to_image.cpp)
target_link_libraries(rm_virtualcam_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud projetant un nuage de point dans l'image acquise par uen caméra
add_executable(rm_pointcloud_to_camera src/Noeuds/rm_pointcloud_to_camera.cpp   )
target_link_libraries(rm_pointcloud_to_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud lisant une image binaire et publiant son squelette sur le réseau ROS (algorithme du DMA)
add_executable(rm_mask_to_skel src/Noeuds/rm_mask_to_skel.cpp)
add_dependencies(rm_mask_to_skel ${PROJECT_NAME}_gencfg)
target_link_libraries(rm_mask_to_skel axemedian blobs ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud lisant une transformée en axe médian et publiant la forme reconstruite sur le réseau ROS
add_executable(rm_mat_to_redt src/Noeuds/rm_mat_to_redt.cpp)
add_dependencies(rm_mat_to_redt ${PROJECT_NAME}_gencfg)
target_link_libraries(rm_mat_to_redt axemedian ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud lisant un répertoire d'images et le publiant sur le réseau ROS
add_executable(rm_skel_to_graph src/Noeuds/rm_skel_to_graph.cpp)
target_link_libraries(rm_skel_to_graph axemedian blobs ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Noeud qui convertit une nappe laser (rho,theta) en un nuage de point (X,Y,Z)
add_executable(rm_laserscan_to_tf_pointcloud src/Noeuds/rm_laserscan_to_tf_pointcloud.cpp)
target_link_libraries(rm_laserscan_to_tf_pointcloud ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# Noeud qui projette un nuage de points dans l'image acquise par une caméra
add_executable(rm_laser_to_image src/Noeuds/rm_laser_to_image.cpp)
target_link_libraries(rm_laser_to_image ${catkin_LIBRARIES})


# Noeud simple exécutant une commande système
add_executable(rm_system_cmd src/Noeuds/rm_system_cmd.cpp)
target_link_libraries(rm_system_cmd ${catkin_LIBRARIES})

# Noeud pour tester les algos en cours
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node geometrie algebre ${OpenCV_LIBRARIES})
