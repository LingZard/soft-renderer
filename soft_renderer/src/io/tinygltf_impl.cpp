#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE_WRITE
#include <tiny_gltf.h>

// Provide TinyGLTF implementation in a single translation unit to avoid ODR.

// Stub image IO to satisfy symbols when NO_STB_IMAGE is used.
namespace tinygltf {
bool LoadImageData(Image*, int, std::string*, std::string*, int, int,
                   const unsigned char*, int, void*) {
  return true;  // we don't load embedded images via tinygltf
}
bool WriteImageData(const std::string*, const std::string*, const Image*, bool,
                    const FsCallbacks*, const URICallbacks*, std::string*,
                    void*) {
  return false;
}
}  // namespace tinygltf
