#pragma once

#include <array>
#include <imgui.h>
#include <vector>
#include <GL/gl.h>

namespace ImOsm::Old {
    struct TextureColor {
        static constexpr int RGBA_SZ = 4;
        static constexpr std::array<uint8_t, RGBA_SZ> Snow = {255, 250, 250, 255};
        static constexpr std::array<uint8_t, RGBA_SZ> Gold = {255, 215, 0, 255};
        static constexpr std::array<uint8_t, RGBA_SZ> Aqua = {0, 255, 255, 255};
        static constexpr std::array<uint8_t, RGBA_SZ> Lime = {0, 255, 0, 255};
        static constexpr std::array<uint8_t, RGBA_SZ> Slate = {112, 128, 144, 255};
        static constexpr std::array<uint8_t, RGBA_SZ> Coral = {255, 127, 80, 255};

        TextureColor() : rgba(Snow) {}
        TextureColor(const std::array<uint8_t, RGBA_SZ>& color) : rgba(color) {};

        const std::array<uint8_t, RGBA_SZ> rgba = Snow;
    };

    class TileTexture {
    public:
        TileTexture(int size = 256, TextureColor color = TextureColor::Snow);
        TileTexture(int size, const std::vector<std::byte>& blob);
        ~TileTexture();

        GLuint glID() const {
            if (id == 0) {
                loadTexture();
            }
            return id;
        }

        ImTextureID imID() const {
            return static_cast<size_t>(glID());
        }

    private:
        int width = 256;
        int height = 256;
        int channels = 0;
        std::vector<std::byte> blob;
        mutable GLuint id = 0;

        void loadTexture() const;
    };
}
