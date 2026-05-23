#include "foc2-gui/osm/osm_tile.hpp"

#include <stb_image.h>
#include <GL/gl.h>

namespace ImOsm {
    Tile::Tile(const int z, const int x, const int y, const std::vector<std::byte>& raw_blob, const bool preload)
        : TileDummy(z, x, y), raw_blob(raw_blob) {
        if (preload) {
            stbLoad();
        }
    }

    Tile::~Tile() {
        if (id != 0) {
            glDeleteTextures(1, &id);
        }
    }

    const char* Tile::rawBlob() const noexcept {
        return reinterpret_cast<const char*>(raw_blob.data());
    }

    std::size_t Tile::rawBlobSize() const noexcept {
        return raw_blob.size();
    }

    const char* Tile::rgbaBlob() const noexcept {
        return reinterpret_cast<const char*>(rgba_blob.data());
    }

    std::size_t Tile::rgbaBlobSize() const noexcept {
        return rgba_blob.size();
    }

    ImTextureID Tile::texture() const noexcept {
        return static_cast<size_t>(glID());
    }

    GLuint Tile::glID() const {
        if (id == 0) {
            if (rgba_blob.empty()) {
                stbLoad();
            }
            glLoad();
        }
        return id;
    }

    void Tile::stbLoad() const {
        stbi_set_flip_vertically_on_load(false);

        const auto image_pointer = stbi_load_from_memory(
            reinterpret_cast<stbi_uc const*>(raw_blob.data()),
            raw_blob.size(),
            &pixel_width, &pixel_height,
            &channels, STBI_rgb_alpha
        );

        if (image_pointer) {
            const auto size = static_cast<std::size_t>(pixel_width) * static_cast<std::size_t>(pixel_height) * STBI_rgb_alpha;
            rgba_blob.resize(size);
            rgba_blob.shrink_to_fit();

            const auto byte_pointer = reinterpret_cast<std::byte*>(image_pointer);
            rgba_blob.insert(rgba_blob.begin(), byte_pointer, byte_pointer + size);
            stbi_image_free(image_pointer);
        }
    }

    void Tile::glLoad() const {
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pixel_width, pixel_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbaBlob());
    }
};
