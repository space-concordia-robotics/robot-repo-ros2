#pragma once

#include <memory>
#include <vector>

namespace ImOsm {
    class ITile;

    class ITileSaver {
    public:
        virtual ~ITileSaver() = default;
        [[nodiscard]] virtual bool save(const std::shared_ptr<ITile>& tile) const = 0;
        [[nodiscard]] virtual bool saveMulti(const std::vector<std::shared_ptr<ITile>>& tiles) const = 0;
    };
}
