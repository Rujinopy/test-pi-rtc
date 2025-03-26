# Define the project name as "capturer"
project(capturer) 

find_package(PkgConfig) # Find the PkgConfig package
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera) # Check for the libcamera package and import its target
message(STATUS "libcamera library found:") # Print a status message indicating that the libcamera library was found
message(STATUS "    version: ${LIBCAMERA_VERSION}") # Print the version of the libcamera library
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}") # Print the libraries linked with libcamera
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}") # Print the include path for libcamera

aux_source_directory(${PROJECT_SOURCE_DIR} CAPTURE_FILES) # Add all source files in the project source directory to the CAPTURE_FILES variable

add_library(${PROJECT_NAME} ${CAPTURE_FILES}) # Create a library target with the project name and source files

target_include_directories(${PROJECT_NAME} PUBLIC ${LIBCAMERA_INCLUDE_DIRS}) # Add the libcamera include directories to the target's include directories
target_link_libraries(${PROJECT_NAME} PUBLIC common ${WEBRTC_LIBRARY} pulse-simple pulse ${LIBCAMERA_LINK_LIBRARIES}) # Link the target with the common library, WebRTC library, PulseAudio libraries, and libcamera libraries
