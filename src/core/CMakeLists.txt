# AMCL を包む、juiz に依存しない部分
add_library(localization_amcl_core STATIC
  version.cpp
  localizer.cpp
  map_io.cpp
  stb_image_impl.cpp)
target_include_directories(localization_amcl_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# stb_image (地図の画像を読む。public domain / MIT)
target_include_directories(localization_amcl_core PRIVATE ${PROJECT_SOURCE_DIR}/third_party/stb)
target_link_libraries(localization_amcl_core PUBLIC amcl::amcl)
target_compile_definitions(localization_amcl_core PRIVATE
  LOCALIZATION_AMCL_VERSION="${PROJECT_VERSION}")
