#pragma once

#include <vector>

#include "foc2-gui/osm/osm_tile_dummy.hpp"

namespace ImOsm {
    class Tile : public TileDummy {
    public:
        Tile(int z, int x, int y, const std::vector<std::byte>& raw_blob, bool preload = true);
        ~Tile() override;

        [[nodiscard]] const char* rawBlob() const noexcept override;
        [[nodiscard]] std::size_t rawBlobSize() const noexcept override;

        void rgbaLoad() const noexcept override {
            stbLoad();
        }

        [[nodiscard]] const char* rgbaBlob() const noexcept override;
        [[nodiscard]] std::size_t rgbaBlobSize() const noexcept override;
        [[nodiscard]] ImTextureID texture() const noexcept override;

    private:
        using GLuint = unsigned int;

        GLuint glID() const;
        void stbLoad() const;
        void glLoad() const;

        std::vector<std::byte> raw_blob;
        mutable int pixel_width;
        mutable int pixel_height;
        mutable int channels;
        mutable std::vector<std::byte> rgba_blob;
        mutable GLuint id = 0;
    };
}
