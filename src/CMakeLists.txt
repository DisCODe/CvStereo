SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED COMPONENTS system thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find another necessary libraries

# # OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)

#FIND_PACKAGE(PCL 1.7.0 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)
DISCODE_FIND_DCL(CvCoreTypes)
DISCODE_FIND_DCL(CvBasic)

# Add standard libraries for this DCL: Boost & PCL
MESSAGE(STATUS "${PCL_LIBRARIES}")
SET(DisCODe_LIBRARIES ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} )

# Validate whether components are properly linked to utilized libraries.
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs") 



# ##############################################################################
# Add source directories
# ##############################################################################

# Components
ADD_SUBDIRECTORY(Components)

# Types
#ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(CvStereoConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/CvStereoConfig.cmake @ONLY)
