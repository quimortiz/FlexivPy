#
# Copyright(c) 2020 to 2022 ZettaScale Technology and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License v. 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
# v. 1.0 which is available at
# http://www.eclipse.org/org/documents/edl-v10.php.
#
# SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
#
cmake_minimum_required(VERSION 3.16)
project(helloworld LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT TARGET CycloneDDS-CXX::ddscxx)
  find_package(CycloneDDS-CXX REQUIRED)
endif()

find_package(Boost REQUIRED)

find_package(flexiv_rdk REQUIRED)
find_package(pinocchio REQUIRED)

find_package(yaml-cpp REQUIRED)




# if user has not defined CMAKE_BUILD_TYPE, set it to Release
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

idlcxx_generate(TARGET helloworlddata FILES HelloWorldData.idl WARNINGS no-implicit-extensibility)
idlcxx_generate(TARGET flexivdata FILES FlexivData.idl WARNINGS no-implicit-extensibility)

add_executable(main main.cpp utils.cpp)

target_compile_definitions(main PRIVATE PINOCCHIO_WITH_HPP_FCL PINOCCHIO_WITH_URDFDOM)

target_link_libraries(main CycloneDDS-CXX::ddscxx flexivdata helloworlddata flexiv::flexiv_rdk pinocchio::pinocchio yaml-cpp)



add_executable(gravity_comp gravity_comp.cpp)

target_compile_definitions(gravity_comp PRIVATE PINOCCHIO_WITH_HPP_FCL PINOCCHIO_WITH_URDFDOM)
target_link_libraries(gravity_comp CycloneDDS-CXX::ddscxx flexivdata helloworlddata flexiv::flexiv_rdk pinocchio::pinocchio)


# add flag PINOCCHIO_WITH_HPP_FCL  



# Link both executables to idl data type library and ddscxx.

# target_link_libraries(ddscxxHelloworldSubscriber CycloneDDS-CXX::ddscxx helloworlddata)
#
# # Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
# # on Azure Pipelines
# if(DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
#   if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
#     target_compile_options(ddscxxHelloworldPublisher PRIVATE -fno-analyzer)
#     target_compile_options(ddscxxHelloworldSubscriber PRIVATE -fno-analyzer)
#   endif()
# endif()
#
# set_property(TARGET ddscxxHelloworldPublisher PROPERTY CXX_STANDARD ${cyclonedds_cpp_std_to_use})
# set_property(TARGET ddscxxHelloworldSubscriber PROPERTY CXX_STANDARD ${cyclonedds_cpp_std_to_use})
