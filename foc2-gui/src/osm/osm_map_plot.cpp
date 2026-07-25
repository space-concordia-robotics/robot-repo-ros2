#include "foc2-gui/osm/osm_map_plot.hpp"

#include <algorithm>
#include <implot.h>

#include "foc2-gui/osm/osm_tile_loader_impl.hpp"

namespace ImOsm {
    struct MapPlot::Impl {
        constexpr static ImPlotFlags PLOT_FLAGS = ImPlotFlags_Equal | ImPlotFlags_NoLegend;

        constexpr static ImPlotAxisFlags X_FLAGS =
            ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoGridLines |
            ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels |
            ImPlotAxisFlags_NoInitialFit | ImPlotAxisFlags_NoMenus |
            ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight;

        constexpr static ImPlotAxisFlags Y_FLAGS = X_FLAGS | ImPlotAxisFlags_Invert;

        constexpr static auto UV0 = ImVec2(0, 1);
        constexpr static auto UV1 = ImVec2(1, 0);
        constexpr static auto TINT = ImVec4(1, 1, 1, 1);

        ImPlotPoint mouse_pos;
        ImPlotRect plot_lims;
        ImVec2 plot_size;
    };

    MapPlot::MapPlot()
        : impl(std::make_unique<Impl>()),
          loader(std::make_shared<TileLoaderOsmMap>()) {}

    MapPlot::MapPlot(const std::shared_ptr<ITileLoader>& loader)
        : impl(std::make_unique<Impl>()),
          loader(loader),
          local_box(Eigen::Vector2d(MIN_LAT, MIN_LON), Eigen::Vector2d(MAX_LAT, MAX_LON)) {
        resetBounds();
    }

    MapPlot::~MapPlot() = default;

    void MapPlot::paint() {
        ImPlot::PushStyleVar(ImPlotStyleVar_PlotBorderSize, 0.0f);
        ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0, 0));

        if (ImPlot::BeginPlot("##ImOsmMapPlot", ImVec2(-1, -1), Impl::PLOT_FLAGS)) {
            ImPlot::SetupAxis(ImAxis_X1, nullptr, Impl::X_FLAGS);
            ImPlot::SetupAxis(ImAxis_Y1, nullptr, Impl::Y_FLAGS);
            ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0.0, 1.0);

            if (set_bounds != SetBounds::None) {
                if (set_bounds == SetBounds::Geo) {
                    local_box = geoToLocal(geo_box, 0);
                } else if (set_bounds == SetBounds::Local) {
                    // do nothing
                }
                ImPlot::SetupAxisLimits(ImAxis_X1, local_box.min().x(), local_box.max().x(), ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, local_box.min().y(), local_box.max().y(), ImPlotCond_Always);
                set_bounds = SetBounds::None;
            }

            ImPlot::SetupFinish();

            impl->mouse_pos = ImPlot::GetPlotMousePos(ImAxis_X1, ImAxis_Y1);
            impl->plot_lims = ImPlot::GetPlotLimits(ImAxis_X1, ImAxis_Y1);
            impl->plot_size = ImPlot::GetPlotSize();

            mouse_geo.Reset(y2lat(impl->mouse_pos.y, 0), x2lon(impl->mouse_pos.x, 0));

            pixels.x() = impl->plot_size.x;
            pixels.y() = impl->plot_size.y;

            local_box = Eigen::AlignedBox2d(
                Eigen::Vector2d(impl->plot_lims.X.Min, impl->plot_lims.Y.Min),
                Eigen::Vector2d(impl->plot_lims.X.Max, impl->plot_lims.Y.Max)
            );

            range = local_box.max() - local_box.min();

            res.x() = pixels.x() / range.x();
            res.y() = pixels.y() / range.y();

            // TODO 2026-05-15 (Will Free): should this be std::min or std::max?
            zoom_level = std::clamp(static_cast<int>(std::lround(std::log2(std::max(res.x(), res.y()) / tile_pixels) + 0.5)), MIN_ZOOM, MAX_ZOOM);
            tiles_num = 1 << zoom_level;
            tile_size = 1.0 / tiles_num;

            geo_box = localToGeo(local_box, zoom_level);
            tile_index_box = localToTiles(local_box, zoom_level);

            loader->beginLoad(zoom_level, tile_index_box.min().x(), tile_index_box.max().x(), tile_index_box.min().y(), tile_index_box.max().y());

            auto bmin = ImVec2(tile_index_box.min().x(), tile_index_box.min().y());
            auto bmax = ImVec2(tile_index_box.max().x(), tile_index_box.max().y());

            for (auto x = tile_index_box.min().x(); x <= tile_index_box.max().x() + 1; ++x) {
                bmin.x = x * tile_size;
                bmax.x = (x + 1) * tile_size;
                for (auto y = tile_index_box.min().y(); y <= tile_index_box.max().y() + 1; ++y) {
                    bmin.y = y * tile_size;
                    bmax.y = (y + 1) * tile_size;
                    ImPlot::PlotImage("##", loader->tileAt(zoom_level, x, y), bmin, bmax, Impl::UV0, Impl::UV1, Impl::TINT);
                }
            }

            loader->endLoad();

            paintOverMap();

            ImPlot::EndPlot();
        }

        ImPlot::PopStyleVar();
        ImPlot::PopStyleVar();
    }

    void MapPlot::resetBounds() {
        geo_box = Eigen::AlignedBox2d(
            Eigen::Vector2d(MIN_LAT, MIN_LON),
            Eigen::Vector2d(MAX_LAT, MAX_LON)
        );

        set_bounds = SetBounds::Geo;
    }

    void MapPlot::setBoundsGeo(const Eigen::AlignedBox2d& bounds) {
        geo_box = bounds;
        set_bounds = SetBounds::Geo;
    }

    void MapPlot::setBoundsLocal(const float minX, const float maxX, const float minY, const float maxY) {
        local_box = Eigen::AlignedBox2d(
            Eigen::Vector2d(minX, minY),
            Eigen::Vector2d(maxX, maxY)
        );
        set_bounds = SetBounds::Local;
    }

    bool MapPlot::mouseOnPlot() const {
        return impl->mouse_pos.x > impl->plot_lims.X.Min &&
            impl->mouse_pos.x < impl->plot_lims.X.Max &&
            impl->mouse_pos.y > impl->plot_lims.Y.Min &&
            impl->mouse_pos.y < impl->plot_lims.Y.Max;
    }
}
