// stb_image の実装をこの翻訳単位にだけ置く。読むのは地図の画像だけなので、使う形式を絞る
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_PNM
#define STBI_ONLY_BMP
#define STBI_NO_STDIO_WARNINGS
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "stb_image.h"
