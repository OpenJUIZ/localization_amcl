cmake_minimum_required(VERSION 3.16)
project(localization_amcl VERSION 0.1.1 LANGUAGES C CXX)

# AMCL (OpenJUIZ/amcl) で自己位置推定をする juiz のコンポーネント。計画は docs/plan.md
#
#   cmake -B build -DCMAKE_PREFIX_PATH=<juiz_sdk のアーカイブを展開したディレクトリ>
#   cmake --build build && ctest --test-dir build
#
# juiz_sdk が無いときは LOCALIZATION_AMCL_BUILD_COMPONENT=OFF で core とテストだけをビルドできる。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LOCALIZATION_AMCL_BUILD_COMPONENT "juiz のコンポーネント(プラグイン)をビルドする。juiz_sdk が要る" ON)
set(LOCALIZATION_AMCL_JUIZ_SDK_VERSION 0.2.5 CACHE STRING "使う juiz_sdk の版(juiz と完全に一致させる)")

if (MSVC)
  # ソースは UTF-8。M_PI などを使えるようにする
  add_compile_options(/utf-8)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif ()

# amcl は静的ライブラリにしてプラグインに入れる(配るのはプラグインと libjuiz_sdk だけにする)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(third_party/amcl EXCLUDE_FROM_ALL)

add_subdirectory(src/core)

if (LOCALIZATION_AMCL_BUILD_COMPONENT)
  find_package(juiz_sdk ${LOCALIZATION_AMCL_JUIZ_SDK_VERSION} EXACT CONFIG REQUIRED)
  add_subdirectory(src/component)
endif ()

include(CTest)
if (BUILD_TESTING)
  add_subdirectory(tests)
endif ()
