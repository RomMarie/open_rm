cmake_minimum_required(VERSION 2.8.3)
project(open_rm)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5Core REQUIRED)

find_package( Qt4 4.5.0 COMPONENTS QtCore QtXml REQUIRED )
include(${QT_USE_FILE})

include_directories(include ${Qt5Core_INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRECTORIES} ${OpenCV_INCLUDE_DIRECTORIES})

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


add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node geometrie algebre ${OpenCV_LIBRARIES})
