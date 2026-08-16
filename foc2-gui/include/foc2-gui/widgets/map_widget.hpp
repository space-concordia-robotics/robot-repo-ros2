#pragma once

#include <mbgl/map/map_observer.hpp>
#include <mbgl/util/run_loop.hpp>
#include <mbgl/util/size.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/widget.hpp"

class SDL3OpenGLRendererFrontend;
class SDL3OpenGLRendererBackend;

namespace mbgl {
    class Map;
}

class MapWidget : public UiWidget, public mbgl::MapObserver {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(MapWidget)

    explicit MapWidget(ImApplication& application);
    ~MapWidget() override;

    MapWidget(const MapWidget& other) = delete;
    MapWidget& operator=(const MapWidget& other) = delete;
    MapWidget(MapWidget&& other) noexcept = delete;
    MapWidget& operator=(MapWidget&& other) noexcept = delete;

    void onInit() override;
    void onShutdown() override;

    void invalidate();

    void onDidFailLoadingMap(mbgl::MapLoadError error, const std::string& reason) override;
    void onRenderError(std::exception_ptr exception_ptr) override;

protected:
    void draw() override;

private:
    void handleScroll() const;
    void handleResize();
    void handleMouse();
    void handleMouseMove() const;
    void handleMouseClick();

    std::unique_ptr<mbgl::Map> map;
    std::unique_ptr<SDL3OpenGLRendererFrontend> rendererFrontend;
    std::unique_ptr<SDL3OpenGLRendererBackend> backend;

    bool tracking = false;
    bool rotating = false;
    bool pitching = false;

    // the initial size should not be zero
    mbgl::Size size = mbgl::Size(1, 1);

    mbgl::util::RunLoop runLoop;

    bool dirty = false;

    bool focused = false;
};
