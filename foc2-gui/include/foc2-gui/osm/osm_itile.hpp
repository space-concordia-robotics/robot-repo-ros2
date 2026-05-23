#pragma once

#include <imgui.h>

namespace ImOsm {
    class ITile {
    public:
        virtual ~ITile() = default;

        [[nodiscard]] virtual int getZ() const noexcept = 0;
        [[nodiscard]] virtual int getX() const noexcept = 0;
        [[nodiscard]] virtual int getY() const noexcept = 0;

        [[nodiscard]] virtual bool isTileZXY(int z, int x, int y) const noexcept = 0;
        [[nodiscard]] virtual bool inBounds(int z, int xmin, int xmax, int ymin, int ymax) const noexcept = 0;

        [[nodiscard]] virtual bool isDummy() const noexcept = 0;

        [[nodiscard]] virtual const char* rawBlob() const noexcept = 0;
        [[nodiscard]] virtual size_t rawBlobSize() const noexcept = 0;

        virtual void rgbaLoad() const = 0;
        [[nodiscard]] virtual const char* rgbaBlob() const noexcept = 0;
        [[nodiscard]] virtual size_t rgbaBlobSize() const noexcept = 0;

        [[nodiscard]] virtual ImTextureID texture() const noexcept = 0;
    };
}
