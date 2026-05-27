#include "foc2-gui/osm/osm_tile_grabber_widget.hpp"

#include <cmath>
#include <imgui.h>
#include <string>
#include <misc/cpp/imgui_stdlib.h>

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/osm/osm_map_plot.hpp"
#include "foc2-gui/osm/osm_tile_grabber.hpp"
#include "foc2-gui/osm/osm_tile_saver.hpp"
#include "foc2-gui/osm/osm_tile_source_url_impl.hpp"
#include "foc2-gui/util/imgui_util.hpp"

namespace ImOsm {
    struct TileGrabberWidget::Ui {
        std::string url;
        std::string dirname = "tiles";
        int requestLimit = 10;
        const bool preload = false;
        int minZ = 0;
        int maxZ = 18;
        int tileCount = 0;
        int tileTotal;
        double progress;
    };

    TileGrabberWidget::TileGrabberWidget(const std::shared_ptr<MapPlot>& mapPlot)
        : map_plot{mapPlot}, ui{std::make_unique<Ui>()} {
        ui->url = TileSourceUrlOsm::URL_TEMPLATE;
    }

    void TileGrabberWidget::loadState(/*const mINI::INIStructure& ini*/) {
        // if (ini.has("tile_grabber")) {
        //     const auto opts{ini.get("tile_grabber")};
        //     if (opts.has("source_url")) {
        //         _ui->url = opts.get("source_url");
        //     }
        //     if (opts.has("target_dir")) {
        //         _ui->dirname = opts.get("target_dir");
        //     }
        //     if (opts.has("request_limit")) {
        //         _ui->requestLimit = std::stoi(opts.get("request_limit"));
        //         _ui->requestLimit = std::clamp(_ui->requestLimit, 1, 99);
        //     }
        //     if (opts.has("min_zoom")) {
        //         _ui->minZ = std::stoi(opts.get("min_zoom"));
        //         _ui->maxZ = std::clamp(_ui->maxZ, MIN_ZOOM, MAX_ZOOM);
        //     }
        //     if (opts.has("max_zoom")) {
        //         _ui->maxZ = std::stoi(opts.get("max_zoom"));
        //         _ui->maxZ = std::clamp(_ui->maxZ, MIN_ZOOM, MAX_ZOOM);
        //     }
        // }
    }

    void TileGrabberWidget::saveState(/*mINI::INIStructure& ini*/) const {
        // ini["tile_grabber"].set("source_url", _ui->url);
        // ini["tile_grabber"].set("target_dir", _ui->dirname);
        // ini["tile_grabber"].set("request_limit", std::to_string(_ui->requestLimit));
        // ini["tile_grabber"].set("min_zoom", std::to_string(_ui->minZ));
        // ini["tile_grabber"].set("max_zoom", std::to_string(_ui->maxZ));
    }

    TileGrabberWidget::~TileGrabberWidget() = default;

    void TileGrabberWidget::paint() {
        ImGui::PushID(this);
        ImGui::TextUnformatted("Tile Grabber");
        const auto geo_box = map_plot->geoBox();
        ImGui::TextFmt("Min Lat {:.6f}, Min Lon {:.6f}", geo_box.min().x(), geo_box.min().y());
        ImGui::TextFmt("Max Lat {:.6f}, Max Lon {:.6f}", geo_box.max().x(), geo_box.max().y());
        ImGui::AlignTextToFramePadding();

        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Min Z", &ui->minZ);
        ui->minZ = std::clamp(ui->minZ, 0, MAX_ZOOM);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Max Z", &ui->maxZ);
        ui->maxZ = std::clamp(ui->maxZ, 0, MAX_ZOOM);
        ImGui::SameLine();
        ImGui::TextFmt(
            "Tiles Count: {}",
            countTiles(geo_box.min().x(), geo_box.max().x(), geo_box.min().y(), geo_box.max().y(), ui->minZ, ui->maxZ)
        );

        ImGui::InputText("Tile Server URL##", &ui->url);

        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Max. Requests", &ui->requestLimit);
        ui->requestLimit = std::clamp(ui->requestLimit, 1, 99);
        ImGui::SameLine();
        if (ImGui::Button("OSM")) {
            ui->url = TileSourceUrlOsm::URL_TEMPLATE;
        }
        ImGui::SameLine();
        if (ImGui::Button("ARC")) {
            ui->url = TileSourceUrlArcImagery::URL_TEMPLATE;
        }

        ImGui::InputText("Destination Dir##", &ui->dirname);
        if (ImGui::Button("Grab")) {
            ui->tileTotal = countTiles(geo_box.min().x(), geo_box.max().x(), geo_box.min().y(), geo_box.max().y(), ui->minZ, ui->maxZ);
            tile_grabber = std::make_unique<TileGrabber>(
                std::make_shared<TileSourceUrlCustom>(ui->requestLimit, ui->preload, ui->url),
                std::make_shared<TileSaverSubDir>(ui->dirname)
            );
            tile_grabber->grab(geo_box.min().x(), geo_box.max().x(), geo_box.min().y(), geo_box.max().y(), ui->minZ, ui->maxZ);
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop") && tile_grabber) {
            tile_grabber->stop();
        }

        if (tile_grabber) {
            ui->tileCount = tile_grabber->tileCounter();
        }
        ui->progress = static_cast<float>(ui->tileCount) / static_cast<float>(ui->tileTotal);
        ui->progress = std::isnan(ui->progress) ? 0.f : ui->progress;
        ImGui::SameLine();
        ImGui::TextFmt("{}/{}", ui->tileCount, ui->tileTotal);
        ImGui::SameLine();
        ImGui::ProgressBar(ui->progress);

        ImGui::PopID();
    }
}
