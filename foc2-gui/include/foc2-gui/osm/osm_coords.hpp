#pragma once

#include <algorithm>
#include <implot.h>
#include <math.h>
#include <numbers>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>

namespace ImOsm {
    inline constexpr auto MIN_LAT = -85.0;
    inline constexpr auto MAX_LAT = +85.0;
    inline constexpr auto MIN_LON = -179.9;
    inline constexpr auto MAX_LON = +179.9;
    inline constexpr auto MIN_ZOOM = 0;
    inline constexpr auto MAX_ZOOM = 18;

    inline double lon2x(const double lon, const int z = 0) {
        return (lon + 180.0) / 360.0 * (1 << z);
    }

    inline double lat2y(const double lat, const int z = 0) {
        return (1.0 - asinh(tan(lat * std::numbers::pi / 180.0)) / std::numbers::pi) / 2.0 * (1 << z);
    }

    inline double x2lon(const double x, const int z = 0) {
        return x / (1 << z) * 360.0 - 180.0;
    }

    inline double y2lat(const double y, const int z = 0) {
        const auto n = std::numbers::pi - std::numbers::pi * 2 * y / (1 << z);
        return 180.0 / std::numbers::pi * atan(0.5 * (exp(n) - exp(-n)));
    }

    inline int lon2tx(const double lon, const int z) {
        return static_cast<int>(floor(lon2x(lon, z)));
    }

    inline int lat2ty(const double lat, const int z) {
        return static_cast<int>(floor(lat2y(lat, z)));
    }

    inline std::pair<int, int> minmax_tx(double minLon, double maxLon, int z) {
        minLon = std::clamp(minLon, MIN_LON, MAX_LON);
        maxLon = std::clamp(maxLon, MIN_LON, MAX_LON);
        z = std::clamp(z, MIN_ZOOM, MAX_ZOOM);
        std::pair<int, int> mm = std::minmax(lon2tx(minLon, z), lon2tx(maxLon, z));
        mm.second = std::clamp(mm.second, 0, (1 << z) - 1);
        return mm;
    }

    inline std::pair<int, int> minmax_ty(double minLat, double maxLat, int z) {
        minLat = std::clamp(minLat, MIN_LAT, MAX_LAT);
        maxLat = std::clamp(maxLat, MIN_LAT, MAX_LAT);
        z = std::clamp(z, MIN_ZOOM, MAX_ZOOM);
        std::pair<int, int> mm = std::minmax(lat2ty(minLat, z), lat2ty(maxLat, z));
        mm.second = std::clamp(mm.second, 0, (1 << z) - 1);
        return mm;
    }

    inline size_t countTiles(const double minLat, const double maxLat, const double minLon, const double maxLon, const int minZ, const int maxZ) {
        if (minZ > maxZ || minLat > maxLat || minLon > maxLon) {
            return 0;
        }

        size_t counter = 0;
        for (auto z = minZ; z != maxZ + 1; ++z) {
            const auto [tx_min, tx_max]{minmax_tx(minLon, maxLon, z)};
            const auto [ty_min, ty_max]{minmax_ty(minLat, maxLat, z)};
            const auto total = static_cast<size_t>(tx_max - tx_min + 1) * static_cast<size_t>(ty_max - ty_min + 1);
            counter += total;
        }
        return counter;
    }

    struct OsmCoords {
        double x{};
        double y{};

        OsmCoords(const double x, const double y)
            : x(x), y(y) {}

        explicit OsmCoords(const GeographicLib::GeoCoords& coords)
            : x(lon2x(coords.Latitude())), y(lat2y(coords.Longitude())) {}

        ImPlotPoint toPlotPoint() const {
            return {x, y};
        }

        inline GeographicLib::GeoCoords toGeoCoords() const;
    };

    inline GeographicLib::GeoCoords OsmCoords::toGeoCoords() const {
        return {y2lat(y), x2lon(x)};
    }

    /**
     * Moves forward by distance in direction
     * @param src starting point
     * @param d distance
     * @param b direction
     * @return final point
     */
    inline GeographicLib::GeoCoords destination(const GeographicLib::GeoCoords& src, const double d, const double b = 0.0) {
        double lat;
        double lon;
        GeographicLib::Geodesic::WGS84().Direct(src.Latitude(), src.Longitude(), b, d, lat, lon);

        return GeographicLib::GeoCoords(lat, lon);
    }

    inline double distance(const GeographicLib::GeoCoords& first, const GeographicLib::GeoCoords& second) {
        GeographicLib::Math::real distance;

        GeographicLib::Geodesic::WGS84().Inverse(first.Latitude(), first.Longitude(), second.Latitude(), second.Longitude(), distance);

        return distance;
    }

    inline double bearing(const GeographicLib::GeoCoords& first, const GeographicLib::GeoCoords& second) {
        double distance = 0.0;
        double azimuth = 0.0;
        double final_azimuth = 0.0;

        GeographicLib::Geodesic::WGS84().Inverse(
            first.Latitude(), first.Longitude(),
            second.Latitude(), second.Longitude(),
            distance,
            azimuth,
            final_azimuth
        );

        return azimuth;
    }

    constexpr GeographicLib::GeoCoords midpoint(const GeographicLib::GeoCoords& a, const GeographicLib::GeoCoords& b) {
        double distance;
        double azimuth;
        double final_azimuth;

        GeographicLib::Geodesic::WGS84().Inverse(
            a.Latitude(), a.Longitude(),
            b.Latitude(), b.Longitude(),
            distance, azimuth, final_azimuth
        );

        double lat;
        double lon;

        GeographicLib::Geodesic::WGS84().Direct(
            a.Latitude(), a.Longitude(),
            azimuth,
            distance * 0.5,
            lat, lon
        );

        return GeographicLib::GeoCoords(lat, lon);
    }
}
