#pragma once

#include <memory>

namespace ImOsm::Rich {
    class MarkStorage;

    class DistanceCalcWidget {
    public:
        DistanceCalcWidget(const std::shared_ptr<MarkStorage>& storage);
        ~DistanceCalcWidget();

        void paint() const;

    private:
        struct Ui;
        std::unique_ptr<Ui> ui;

        std::shared_ptr<MarkStorage> storage;
    };
}
