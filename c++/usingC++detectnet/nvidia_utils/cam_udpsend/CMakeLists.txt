file(GLOB gstCameraSources *.cpp)
file(GLOB gstCameraIncludes *.h )

add_executable(cam_udpsend ${gstCameraSources})
target_link_libraries(cam_udpsend jetson-utils)

install(TARGETS cam_udpsend DESTINATION bin)

# WE HAVE TO ADD THE NEW SUBDIRECTORY @ CMakeLists of "utils" directory