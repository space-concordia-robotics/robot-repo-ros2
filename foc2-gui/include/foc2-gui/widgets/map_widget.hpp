#pragma once

#include <Eigen/Core>
#include <mln/map/map_observer.hpp>
#include <mln/util/run_loop.hpp>
#include <mln/util/size.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/widget.hpp"

class MaplibreLayerManager;
class SDL3OpenGLRendererFrontend;
class SDL3OpenGLRendererBackend;

namespace mln {
    class Map;
}

class MapWidget : public UiWidget, public mln::MapObserver {
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

    void onDidFailLoadingMap(mln::MapLoadError error, const std::string& reason) override;
    void onRenderError(std::exception_ptr exception_ptr) override;

    void toLatLong(const geometry_msgs::msg::PoseStamped& pose, std::function<void(const geographic_msgs::msg::GeoPoint&)> callback) const;

protected:
    void draw() override;

private:
    void configureStyle();
    void handleResize();
    void handleMouse(const Eigen::Vector2d& widget_origin, const Eigen::Vector2d& widget_size);
    void handleScroll(const Eigen::Vector2d& mouse_pos) const;
    void handleMouseMove() const;
    void handleMouseClick(bool in_bounds);

    rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_client;

    std::unique_ptr<mln::Map> map;
    std::unique_ptr<SDL3OpenGLRendererFrontend> renderer_frontend;
    std::unique_ptr<SDL3OpenGLRendererBackend> backend;

    std::vector<std::unique_ptr<MaplibreLayerManager>> layer_managers;

    bool tracking = false;
    bool rotating = false;
    bool pitching = false;

    // the initial size should not be zero
    mln::Size size = mln::Size(1, 1);

    mln::util::RunLoop run_loop;

    bool dirty = false;

    bool focused = false;
};
