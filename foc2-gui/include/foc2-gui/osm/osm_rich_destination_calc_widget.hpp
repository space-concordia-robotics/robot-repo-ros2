#pragma once

#include <memory>

namespace ImOsm::Rich {
    class MarkStorage;

    class DestinationCalcWidget {
    public:
        DestinationCalcWidget(const std::shared_ptr<MarkStorage>& storage);
        ~DestinationCalcWidget();

        void paint() const;

    private:
        struct Ui;
        std::unique_ptr<Ui> ui;

        std::shared_ptr<MarkStorage> storage;
    };
}
