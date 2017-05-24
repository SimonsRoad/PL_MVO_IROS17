# set project's name
PROJECT( PL_MVO_SPOS )

###############################################################################
# CMake settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

# this command finds OpenCV libraries and sets all required variables
FIND_PACKAGE(OpenCV REQUIRED)

# this command finds Eigen libraries and sets all required variables

SET(Eigen_DIR "I:\\ProgramFiles\\PCL1.7.2\\PCL1.7.2\\3rdParty\\Eigen\\eigen3")
INCLUDE_DIRECTORIES(${Eigen_DIR})

FILE(GLOB_RECURSE HDRS_FILES *.h *.hpp)
FILE(GLOB_RECURSE SRCS_FILES *.c *.cpp)

#ADD_LIBRARY(${PROJECT_NAME}  ${HDRS_FILES} ${SRCS_FILES})
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})

ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_FILES} ${HDRS_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
