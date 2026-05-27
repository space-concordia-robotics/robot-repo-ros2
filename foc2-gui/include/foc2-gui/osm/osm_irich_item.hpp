#pragma once

#include <Eigen/Dense>

namespace ImOsm::Rich {
    class IRichItem {
    public:
        virtual ~IRichItem() = default;

        virtual bool inBounds(Eigen::AlignedBox2d geo_box) const = 0;
        virtual void setEnabled(bool enabled) = 0;
        virtual bool isEnabled() const = 0;
        virtual void paint() = 0;
    };
}
