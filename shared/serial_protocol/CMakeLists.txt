cmake_minimum_required(VERSION 3.8)
project(serial_protocol LANGUAGES CXX)
 
find_package(ament_cmake REQUIRED)
 
add_library(serial_protocol STATIC src/parser.cpp)
target_compile_features(serial_protocol PUBLIC cxx_std_17)
set_target_properties(serial_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
 
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS serial_protocol
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
 
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()