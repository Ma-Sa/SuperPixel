project(SuperPixel)

cmake_minimum_required(VERSION 2.8)

set(CUDA_PROPAGATE_HOST_FLAGS off)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()


find_package(Pangolin 0.1 QUIET)
find_package(HAL 0.1 QUIET)
find_package(OpenCV QUIET )
find_package(CUDA REQUIRED)

 if (BUILD_CUDA_LIB)

     if(CMAKE_COMPILER_IS_GNUCC)

    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-Xcompiler;-fPIC;-arch=sm_13" )

       if (NVCC_COMPILER_BINDIR)

            set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};--compiler-bindir=${NVCC_COMPILER_BINDIR}")

       endif()

     else()

       set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-arch=sm_13" )

	endif()
endif()

if(Pangolin_FOUND AND HAL_FOUND AND CUDA_FOUND)
    cuda_add_executable( SuperPixel main.cpp FastImgSeg.cpp cudaImgTrans.cu cudaSegEngine.cu cudaSegSLIC.cu cudaUtil.cu )


    include_directories( ${HAL_INCLUDE_DIRS} )
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    include_directories( ${CUDA_INCLUDE_DIRS} )
    target_link_libraries(SuperPixel ${HAL_LIBRARIES} )
    target_link_libraries(SuperPixel ${Pangolin_LIBRARIES} )
    target_link_libraries(SuperPixel ${CUDA_LIBRARIES} )

    add_dependencies( SuperPixel hal )
    add_dependencies( SuperPixel PbMsgs )
endif()
