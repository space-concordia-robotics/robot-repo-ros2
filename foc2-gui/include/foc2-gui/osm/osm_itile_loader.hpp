#pragma once

#include <imgui.h>

namespace ImOsm {
    class ITileLoader {
    public:
        virtual ~ITileLoader() = default;

        virtual void beginLoad(int z, int xmin, int xmax, int ymin, int ymax) = 0;

        virtual ImTextureID tileAt(int z, int x, int y) = 0;

        virtual int getTileCount() const {
            return 0;
        };
        virtual void endLoad() {}
    };
}
