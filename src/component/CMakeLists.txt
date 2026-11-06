# juiz のコンポーネント。.juizapp の plugins/<target>/ に、このライブラリと libjuiz_sdk を置く
add_library(localization_amcl SHARED localization_amcl_component.cpp)
# generated/: juiz_std の型の C++ ヘッダ (scripts/gen_types.sh で作る)
target_include_directories(localization_amcl PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated)
target_link_libraries(localization_amcl PRIVATE localization_amcl_core juiz_sdk::juiz_sdk)
# rpath を $ORIGIN (macOS は @loader_path) だけにし、ビルドしたライブラリの隣に libjuiz_sdk を写す
juiz_sdk_bundle(localization_amcl)
