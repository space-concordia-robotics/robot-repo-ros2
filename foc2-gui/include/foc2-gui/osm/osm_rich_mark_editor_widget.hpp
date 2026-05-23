#pragma once

#include "foc2-gui/osm/osm_rich_mark_storage.hpp"

namespace ImOsm::Rich {
    class RichMapPlot;
    class MarkStorage;
    class MarkItemWidget;

    class MarkEditorWidget {
    public:
        MarkEditorWidget(const std::shared_ptr<RichMapPlot>& plot, const std::shared_ptr<MarkStorage>& storage);
        ~MarkEditorWidget();

        void paint();

    private:
        void paintLatLonInput() const;
        void paintMousePickBtn() const;
        void paintMarkNameInput() const;
        void paintAddMarkBtn() const;
        void paintMarkTable();
        void paintMarkTableRow(const MarkStorage::ItemNode& item);

        std::shared_ptr<RichMapPlot> plot;
        std::shared_ptr<MarkStorage> storage;
        std::unique_ptr<MarkItemWidget> item_widget;

        struct Ui;
        std::unique_ptr<Ui> ui;
    };
}
