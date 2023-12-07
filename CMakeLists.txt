cmake_minimum_required(VERSION 3.10)
project(franka_control)

# minimum C++ version
set(CMAKE_CXX_STANDARD 17)


find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(Boost 1.58.0 REQUIRED system thread)
# find_package(pinocchio REQUIRED)

include_directories( ${Boost_INCLUDE_DIR} )
# include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${Franka_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${pinocchio_INCLUDE_DIRS})
link_directories(${Franka_INCLUDE_DIRS})




#build franka_control
# define source files 
# file(GLOB_RECURSE FRANKA_CONTROL_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp )
# list(REMOVE_ITEM FRANKA_CONTROL_SRC ${CMAKE_CURRENT_LIST_DIR}/src/runners/read_states.cpp )
# add_executable(franka_control ${FRANKA_CONTROL_SRC})
# target_link_libraries(franka_control  PUBLIC Franka::Franka libzmq ${Franka} ${cppzmq} Eigen3::Eigen )

# # build read_states
# file( GLOB_RECURSE FRANKA_READ_STATES ${CMAKE_CURRENT_LIST_DIR}/src/runners/read_states.cpp )
# add_executable(read_states ${FRANKA_READ_STATES})
# target_link_libraries(read_states PUBLIC Franka::Franka libzmq ${Franka} ${cppzmq} Eigen3::Eigen )


#[[build tests]]

set(RUNNERS
    medcvr_point_collection    
    gravity_compensation
    impedance
    # nullspace_controller
)

foreach(runner ${RUNNERS})
    file(GLOB_RECURSE RUNNERS_SRC ${CMAKE_CURRENT_LIST_DIR}/runners/${runner}.cpp ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
    #list(REMOVE_ITEM RUNNERS_SRC ${CMAKE_CURRENT_LIST_DIR}/src/runners/runner.cpp ${CMAKE_CURRENT_LIST_DIR}/src/runners/read_states.cpp )
    add_executable( run_${runner} ${RUNNERS_SRC} )
    target_link_libraries(run_${runner} PUBLIC Franka::Franka  ${Franka}  ${Boost_LIBRARIES} Eigen3::Eigen Poco::Foundation)
endforeach() 

# custom library
add_library(franka_common STATIC
  src/common.cpp
)

add_library(ik STATIC
  src/ik.cpp
)


target_link_libraries(franka_common PUBLIC  ${Franka_LIBRARIES} )

# custom library (network client) 
add_library(network_client STATIC
  src/teleop/network_client.cpp
)
target_link_libraries(network_client PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})

# custom library (network server) 
add_library(network_server STATIC
  src/teleop/network_server.cpp
)
target_link_libraries(network_server PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})

# custom library (teleop follower) 
add_library(follower STATIC
  src/teleop/follower.cpp
)
target_link_libraries(follower PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} franka_common ik)

# custom library (teleop leader) 
add_library(leader STATIC
  src/teleop/leader.cpp
  
)
target_link_libraries(leader PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} franka_common)


# build network examples
set(EXTRA_NETWORK_EXAMPLES
  franka_master_client
  franka_slave_server
)

# foreach(network_ex ${EXTRA_NETWORK_EXAMPLES})
#   add_executable(${network_ex} ${CMAKE_CURRENT_LIST_DIR}/runners/extra/${network_ex}.cpp)
#   target_link_libraries(${network_ex} ${Franka_LIBRARIES} ${Boost_LIBRARIES} franka_common )
# endforeach()

add_executable(run_follower_franka ${CMAKE_CURRENT_LIST_DIR}/runners/teleop/follower_franka.cpp)
target_link_libraries( run_follower_franka 
  PUBLIC
  Franka::Franka
  follower
  network_server
  franka_common
)

add_executable(run_leader_franka ${CMAKE_CURRENT_LIST_DIR}/runners/teleop/leader_franka.cpp)
target_link_libraries( run_leader_franka 
  PUBLIC
  Franka::Franka
  leader
  network_client
  ${Franka_LIBRARIES}
)


add_executable(run_nullspace_control ${CMAKE_CURRENT_LIST_DIR}/runners/nullspace_control.cpp)
target_link_libraries( run_nullspace_control
  PUBLIC
  Franka::Franka
  ${Franka_LIBRARIES}
)
# # custom library
# add_library(examples_common STATIC
#   src/examples/examples_common.cpp
# )
# target_link_libraries(examples_common PUBLIC  ${Franka_LIBRARIES} )

# # custom library (network client) 
# add_library(network_client STATIC
#   src/teleop/network_client.cpp
# )
# target_link_libraries(network_client PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})

# # custom library (network server) 
# add_library(network_server STATIC
#   src/teleop/network_server.cpp
# )
# target_link_libraries(network_server PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})


# # custom library (teleop follower) 
# add_library(follower STATIC
# ${CMAKE_CURRENT_LIST_DIR}/src/teleop/follower.cpp
# )
# target_link_libraries(follower PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} examples_common)

# # custom library (teleop leader) 
# add_library(leader STATIC
#   src/teleop/leader.cpp
# )
# target_link_libraries(leader PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} examples_common)





