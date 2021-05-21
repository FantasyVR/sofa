cmake_minimum_required(VERSION 3.1)

#replace here with your project name and version number
project(MyPlugin VERSION 0.1)

#look for all packages on which your project depends
find_package(SofaFramework REQUIRED)

set(HEADER_FILES
    initMyPlugin.h
    NSlidingConstraint.h
    NeedleContact.h
)

set(SOURCE_FILES
    initMyPlugin.cpp
    NSlidingConstraint.inl
    NSlidingConstraint.cpp
    NeedleContact.inl
    NeedleContact.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaCore SofaConstraint SofaHelper SofaDefaultType)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_MYPLUGIN")


install(TARGETS MyPlugin
        COMPONENT MyPlugin_Libraries
        EXPORT  MyPluginTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)