cmake_minimum_required(VERSION 3.7)
project(follower_mpcc)

set(CMAKE_CXX_STANDARD 14)
# Release build, full optimisation
set(CMAKE_CXX_FLAGS "-O2")

# Static link the libraries under External
# The libraries can be obtained by running install.sh
include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/matplotlib)
include_directories(External/Eigen)

add_executable(follower_mpcc
        main.cpp)

# For matplotlib
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(follower_mpcc ${PYTHON_LIBRARIES})

# For HPIPM and Blasfeo
target_link_libraries(follower_mpcc
        ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)