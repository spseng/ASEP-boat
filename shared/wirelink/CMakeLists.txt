cmake_minimum_required(VERSION 3.8)
project(wirelink LANGUAGES CXX)
 
find_package(ament_cmake REQUIRED)
 
add_library(wirelink STATIC src/parser.cpp)
target_compile_features(wirelink PUBLIC cxx_std_17)
set_target_properties(wirelink PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wirelink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
 
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS wirelink
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
 
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()