#include "foc2-gui/osm/osm_tile_texture.hpp"

#include <stb_image.h>

namespace ImOsm::Old {
    TileTexture::TileTexture(const int size, const TextureColor color) : width(size), height(size) {
        blob.resize(width * height * TextureColor::RGBA_SZ);
        blob.shrink_to_fit();
        for (size_t i = 0; i != blob.size(); i = i + TextureColor::RGBA_SZ) {
            blob[i] = static_cast<std::byte>(color.rgba[0]);
            blob[i + 1] = static_cast<std::byte>(color.rgba[1]);
            blob[i + 2] = static_cast<std::byte>(color.rgba[2]);
            blob[i + 3] = static_cast<std::byte>(color.rgba[3]);
        }
    }

    TileTexture::TileTexture(int /*size*/, const std::vector<std::byte>& blob) {
        stbi_set_flip_vertically_on_load(false);

        const auto ptr = stbi_load_from_memory(
            reinterpret_cast<stbi_uc const*>(blob.data()),
            static_cast<int>(blob.size()),
            &width, &height,
            &channels,
            STBI_rgb_alpha
        );

        if (ptr) {
            const auto byteptr = reinterpret_cast<std::byte*>(ptr);
            this->blob.insert(blob.begin(), byteptr, byteptr + width * height * STBI_rgb_alpha);
            stbi_image_free(ptr);
        }
    }

    TileTexture::~TileTexture() {
        glDeleteTextures(1, &id);
    }

    void TileTexture::loadTexture() const {
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, blob.data());
    }
}
