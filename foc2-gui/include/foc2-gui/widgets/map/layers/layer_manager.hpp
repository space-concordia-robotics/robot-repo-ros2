#pragma once

#include <ros2_fmt_logger/logger.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/widgets/map_widget.hpp"

class MaplibreLayerManager {
public:
    // TODO 2026-10-29 (Will Free): consider passing the style to the constructor and storing it?

    explicit MaplibreLayerManager(ImApplication& application, MapWidget& widget)
        : widget(widget), application(application), logger(application.get_logger()) {}

    virtual ~MaplibreLayerManager() = default;

    MaplibreLayerManager(const MaplibreLayerManager& other) = delete;

    MaplibreLayerManager(MaplibreLayerManager&& other) noexcept
        : widget(other.widget),
          application(other.application),
          logger(std::move(other.logger)) {}

    MaplibreLayerManager& operator=(const MaplibreLayerManager& other) = delete;

    // TODO 2026-10-30 (Will Free): don't use references for fields & implement move operator
    MaplibreLayerManager& operator=(MaplibreLayerManager&& other) = delete;

    virtual void install(mln::style::Style& style) = 0;
    virtual void uninstall(mln::style::Style& style) = 0;

    virtual void update() = 0;

    virtual void enable() = 0;
    virtual void disable() = 0;

protected:
    MapWidget& widget;
    ImApplication& application;
    ros2_fmt_logger::Logger logger;
};
