#include "foc2-gui/osm/osm_tile_source_widget.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_map_plot.hpp"
#include "foc2-gui/osm/osm_tile_loader_impl.hpp"

namespace ImOsm {
    struct TileSourceWidget::Ui {
        std::string source = TileSourceUrlOsm::URL_TEMPLATE;
        int request_limit = 10;
    };

    TileSourceWidget::TileSourceWidget(const std::shared_ptr<MapPlot>& map_plot)
        : map_plot(map_plot), ui(std::make_unique<Ui>()) {
        updateTileLoader();
    }

    void TileSourceWidget::loadState(/*const mINI::INIStructure& ini*/) {
        // if (ini.has("tile_source")) {
        //     const auto opts{ini.get("tile_source")};
        //     if (opts.has("path_url")) {
        //         _ui->source = opts.get("path_url");
        //     }
        //     if (opts.has("request_limit")) {
        //         _ui->requestLimit = std::stoi(opts.get("request_limit"));
        //         _ui->requestLimit = std::clamp(_ui->requestLimit, 1, 99);
        //     }
        // }
        updateTileLoader();
    }

    void TileSourceWidget::saveState(/*mINI::INIStructure& ini*/) const {
        // ini["tile_source"].set("path_url", _ui->source);
        // ini["tile_source"].set("request_limit", std::to_string(_ui->requestLimit));
    }

    TileSourceWidget::~TileSourceWidget() = default;

    void TileSourceWidget::paint() {
        ImGui::PushID(this);

        ImGui::TextUnformatted("Tile Source");
        if (ImGui::Button("Apply")) {
            updateTileLoader();
        }
        ImGui::SameLine();
        ImGui::InputText("Path / URL", &ui->source);

        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Max. Requests", &ui->request_limit);
        ui->request_limit = std::clamp(ui->request_limit, 1, 99);
        ImGui::SameLine();
        if (ImGui::Button("OSM")) {
            ui->source = TileSourceUrlOsm::URL_TEMPLATE;
            map_plot->setTileLoader(std::make_shared<TileLoaderOsmMap>(ui->request_limit));
        }
        ImGui::SameLine();
        if (ImGui::Button("ARC")) {
            ui->source = TileSourceUrlArc::URL_TEMPLATE;
            map_plot->setTileLoader(std::make_shared<TileLoaderArcMap>(ui->request_limit));
        }

        ImGui::PopID();
    }

    void TileSourceWidget::updateTileLoader() {
        if (ui->source.starts_with("http")) {
            tile_loader = std::make_shared<TileLoaderUrlMap>(ui->source, ui->request_limit);
        } else if (!ui->source.empty()) {
            tile_loader = std::make_shared<TileLoaderFsMap>(ui->source, std::nullopt, ui->request_limit);
        }
        map_plot->setTileLoader(tile_loader);
    }
}
