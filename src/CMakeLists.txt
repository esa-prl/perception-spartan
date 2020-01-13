set(SPARTAN_SRC_DIR "/home/marta/compass/software/vomap/src")
set(SPARTAN_BUILD_DIR "/home/marta/compass/software/vomap/src/build")

include_directories("${SPARTAN_SRC_DIR}")
include_directories("${SPARTAN_SRC_DIR}/VO")
include_directories("${SPARTAN_SRC_DIR}/fundest-1.0")
include_directories("${SPARTAN_SRC_DIR}/levmar-2.6")
include_directories("${SPARTAN_SRC_DIR}/multires")
include_directories("${SPARTAN_SRC_DIR}/robreg")
include_directories("${SPARTAN_SRC_DIR}/sweep")

link_directories("${SPARTAN_SRC_DIR}/VO")
link_directories("${SPARTAN_SRC_DIR}/fundest-1.0")
link_directories("${SPARTAN_SRC_DIR}/levmar-2.6")
link_directories("${SPARTAN_SRC_DIR}/multires")
link_directories("${SPARTAN_SRC_DIR}/robreg")
link_directories("${SPARTAN_SRC_DIR}/sweep")

add_library(vo STATIC IMPORTED)
set_property(TARGET vo PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/VO/libvo.a")

add_library(fd STATIC IMPORTED)
set_property(TARGET fd PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/fundest-1.0/libfundest.a")

add_library(lm STATIC IMPORTED)
set_property(TARGET lm PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/levmar-2.6/liblevmar.a")

add_library(mr STATIC IMPORTED)
set_property(TARGET mr PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/multires/libmultires.a")

add_library(rr STATIC IMPORTED)
set_property(TARGET rr PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/robreg/librobreg.a")

add_library(sw STATIC IMPORTED)
set_property(TARGET sw PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/sweep/libsweep.a")

find_package(OpenCV REQUIRED)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#target_link_libraries(frame_test vo)
###############

rock_library(spartan
    SOURCES ImageLoader.cpp OdometryExecutor.cpp SpartanMap.cpp MappingExecutor.cpp
    HEADERS ImageLoader.hpp Config.hpp OdometryExecutor.hpp CalibInfo.hpp SpartanMap.hpp MappingExecutor.hpp
    DEPS_PKGCONFIG base-types
    LIBS vo fd lm rr mr sw ${OpenCV_LIBS} ${PCL_LIBRARIES})



#rock_executable(frame_test_bin Main.cpp
#    DEPS frame_test)

