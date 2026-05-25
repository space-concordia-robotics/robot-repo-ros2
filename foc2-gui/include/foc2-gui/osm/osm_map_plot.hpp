#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <GeographicLib/GeoCoords.hpp>

#include "foc2-gui/osm/osm_coords.hpp"

namespace ImOsm {
    class ITileLoader;

    static Eigen::AlignedBox2d geoToLocal(const Eigen::AlignedBox2d& geo_box, const int zoom) {
        return Eigen::AlignedBox2d(
            Eigen::Vector2d(lon2x(geo_box.min().y(), zoom), lat2y(geo_box.min().x(), zoom)),
            Eigen::Vector2d(lon2x(geo_box.max().y(), zoom), lat2y(geo_box.max().x(), zoom))
        );
    }

    static Eigen::AlignedBox2d localToGeo(const Eigen::AlignedBox2d& local_box, const int zoom) {
        const auto tile_scale = 1 << zoom;

        return Eigen::AlignedBox2d(
            Eigen::Vector2d(y2lat(local_box.min().y() * tile_scale, zoom), x2lon(local_box.min().x() * tile_scale, zoom)),
            Eigen::Vector2d(y2lat(local_box.max().y() * tile_scale, zoom), x2lon(local_box.max().x() * tile_scale, zoom))
        );
    }

    static Eigen::Vector2d localToGeo(const Eigen::Vector2d& local_pt, const int zoom) {
        const auto tile_scale = zoom << 1;

        return Eigen::Vector2d(y2lat(local_pt.y() * tile_scale, zoom), x2lon(local_pt.x() * tile_scale, zoom));
    }

    static Eigen::Vector2d geoToLocal(const Eigen::Vector2d& geo_pt, const int zoom) {
        return Eigen::Vector2d(lon2x(geo_pt.y(), zoom), lat2y(geo_pt.x(), zoom));
    }

    static Eigen::AlignedBox2i localToTiles(const Eigen::AlignedBox2d& local_box, const int zoom) {
        const auto tile_scale = 1 << zoom;

        const auto min = Eigen::Vector2i(
            std::clamp(static_cast<int>(local_box.min().x() * tile_scale), 0, tile_scale - 1),
            std::clamp(static_cast<int>(local_box.min().y() * tile_scale), 0, tile_scale - 1)
        );

        const auto max = Eigen::Vector2i(
            std::clamp(static_cast<int>(local_box.max().x() * tile_scale), 0, tile_scale - 1),
            std::clamp(static_cast<int>(local_box.max().y() * tile_scale), 0, tile_scale - 1)
        );

        return Eigen::AlignedBox2i(min, max);
    }

    static Eigen::AlignedBox2d tilesToLocal(const Eigen::AlignedBox2i& tile_box, const int zoom) {
        const int tile_scale = 1 << zoom;
        const double tile_size = 1.0 / static_cast<double>(tile_scale);

        return Eigen::AlignedBox2d(
            Eigen::Vector2d(tile_box.min().x() * tile_size, tile_box.min().y() * tile_size),
            Eigen::Vector2d((tile_box.max().x() + 1.0) * tile_size, (tile_box.max().y() + 1.0) * tile_size)
        );
    }


    class MapPlot {
    public:
        MapPlot();
        MapPlot(const std::shared_ptr<ITileLoader>& loader);
        virtual ~MapPlot();

        void setTileLoader(const std::shared_ptr<ITileLoader>& loader) {
            this->loader = loader;
        }

        virtual void paint();
        virtual void paintOverMap() {}

        inline void resetBounds();

        void setBoundsGeo(const Eigen::AlignedBox2d& bounds);

        GeographicLib::GeoCoords mouse() const {
            return mouse_geo;
        }

        Eigen::AlignedBox2d geoBox() const {
            return geo_box;
        }

        bool inBoundsGeo(const GeographicLib::GeoCoords& coords) const {
            return geo_box.contains(Eigen::Vector2d(coords.Latitude(), coords.Longitude()));
        }

        Eigen::AlignedBox2i tileBox() const {
            return tile_index_box;
        }

        int zoom() const {
            return zoom_level;
        }

        void setBoundsLocal(float minX, float maxX, float minY, float maxY);

        Eigen::AlignedBox2d localBox() const {
            return local_box;
        }

        bool inBoundsLocal(const Eigen::Vector2d& point) const {
            return local_box.contains(point);
        }

        bool mouseOnPlot() const;

    private:
        struct Impl;

        std::unique_ptr<Impl> impl;
        std::shared_ptr<ITileLoader> loader;
        std::shared_ptr<ITileLoader> prev_loader;

        double tile_pixels = 256.0;
        double tile_size = 0;

        Eigen::AlignedBox2d geo_box = Eigen::AlignedBox2d();

        Eigen::AlignedBox2d local_box = Eigen::AlignedBox2d();
        Eigen::AlignedBox2i tile_index_box = Eigen::AlignedBox2i();

        Eigen::Vector2d pixels = Eigen::Vector2d::Zero();
        Eigen::Vector2d range = Eigen::Vector2d::Zero();
        Eigen::Vector2d res = Eigen::Vector2d::Zero();

        int zoom_level = 0;
        int tiles_num = 0;
        GeographicLib::GeoCoords mouse_geo;
        GeographicLib::GeoCoords mouse_clicked_geo;

        enum class SetBounds {
            None,
            Geo,
            Local
        };

        SetBounds set_bounds = SetBounds::None;
    };
}
