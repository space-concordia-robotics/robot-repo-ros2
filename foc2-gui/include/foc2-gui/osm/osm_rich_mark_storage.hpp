#pragma once

#include <memory>
#include <vector>

#include "foc2-gui/osm/osm_coords.hpp"

namespace ImOsm::Rich {
    class IRichItem;
    class MarkItem;

    class MarkStorage {
        friend class MarkEditorWidget;

    public:
        MarkStorage();
        ~MarkStorage();

        std::shared_ptr<MarkItem> findMark(const std::string& name) const;
        GeographicLib::GeoCoords findMark(const std::string& name, bool& ok) const;

        void loadState(/*const mINI::INIStructure& ini*/);
        void saveState(/*mINI::INIStructure& ini*/) const;

        void setPickCoords(const GeographicLib::GeoCoords& coords) {
            pick_coords = coords;
            pick_state = true;
        }

    private:
        void addMark(const GeographicLib::GeoCoords& coords, const std::string& name);
        void rmMarks();
        bool handlePickState();
        bool handleLoadState();

        struct ItemNode;

        std::vector<ItemNode>& markItems() {
            return mark_items;
        }

        const std::vector<ItemNode>& markItems() const {
            return mark_items;
        }

        // Load State
        bool load_state = false;

        // Pick State
        GeographicLib::GeoCoords pick_coords;
        bool pick_state = false;

        // Mark Items
        struct ItemNode {
            std::shared_ptr<MarkItem> ptr;
            mutable bool rm_flag = false;
        };

        std::vector<ItemNode> mark_items;
    };
}
