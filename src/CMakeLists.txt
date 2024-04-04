CMAKE_MINIMUM_REQUIRED( VERSION 3.2 ) #设定版本
# 增加一个可执行的二进制
# ADD_EXECUTABLE( main main.cpp )
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# FIND_PACKAGE(VTK REQUIRED)
# INCLUDE_DIRECTORIES(/usr/include/vtk-9.1/)
# INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})
# LINK_LIBRARIES(${VTK_INCLUDE_DIRS})
# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

ADD_EXECUTABLE( pclvs main.cpp )
TARGET_LINK_LIBRARIES( pclvs ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( filter filter.cpp )
TARGET_LINK_LIBRARIES( filter ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )    

ADD_EXECUTABLE( normalvector_estimate normalvector_estimate.cpp )
TARGET_LINK_LIBRARIES( normalvector_estimate ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  

ADD_EXECUTABLE( segementation segementation.cpp )
TARGET_LINK_LIBRARIES( segementation ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  

ADD_EXECUTABLE( myproject myproject.cpp )
TARGET_LINK_LIBRARIES( myproject ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  

ADD_EXECUTABLE( correspondence correspondence.cpp )
TARGET_LINK_LIBRARIES( correspondence ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  

ADD_EXECUTABLE( hull hull.cpp )
TARGET_LINK_LIBRARIES( hull ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  

ADD_EXECUTABLE( corrview corrview.cpp )
TARGET_LINK_LIBRARIES( corrview ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )  