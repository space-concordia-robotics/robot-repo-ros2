#pragma once

#include "foc2-gui/osm/osm_itile.hpp"

namespace ImOsm {
    class TileDummy : public ITile {
    public:
        TileDummy(const int z, const int x, const int y)
            : z(z), x(x), y(y) {}

        ~TileDummy() override = default;

        [[nodiscard]] int getZ() const noexcept override {
            return z;
        }

        [[nodiscard]] int getX() const noexcept override {
            return x;
        }

        [[nodiscard]] int getY() const noexcept override {
            return y;
        }

        [[nodiscard]] bool isTileZXY(const int z, const int x, const int y) const noexcept override {
            return this->z == z && this->x == x && this->y == y;
        }

        [[nodiscard]] bool inBounds(const int z, const int xmin, const int xmax, const int ymin, const int ymax) const noexcept override {
            const bool cz = this->z == z;
            const bool cx = this->x >= xmin || this->x <= xmax;
            const bool cy = this->y >= ymin || this->y <= ymax;
            return cz && cx && cy;
        }

        [[nodiscard]] bool isDummy() const noexcept final {
            return rawBlob() == nullptr;
        }

        [[nodiscard]] const char* rawBlob() const noexcept override {
            return nullptr;
        }

        [[nodiscard]] size_t rawBlobSize() const noexcept override {
            return 0;
        }

        void rgbaLoad() const noexcept override {}

        [[nodiscard]] const char* rgbaBlob() const noexcept override {
            return nullptr;
        }

        [[nodiscard]] size_t rgbaBlobSize() const noexcept override {
            return 0;
        }

        [[nodiscard]] ImTextureID texture() const noexcept override {
            return 0;
        }

    private:
        int z;
        int x;
        int y;
    };
}
