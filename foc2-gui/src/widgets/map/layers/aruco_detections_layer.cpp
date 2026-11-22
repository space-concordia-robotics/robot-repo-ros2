#include "foc2-gui/widgets/map/layers/aruco_detections_layer.hpp"

#include <fmt/format.h>
#include <mln/style/expression/coalesce.hpp>
#include <mln/style/expression/dsl.hpp>
#include <mln/style/layers/symbol_layer.hpp>
#include <mln/style/sources/geojson_source.hpp>
#include <mln/util/geojson.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using geometry_msgs::msg::PoseStamped;
using geographic_msgs::msg::GeoPoint;

static constexpr auto ARUCO_LAYER_ID = "aruco-detections";
static constexpr auto ARUCO_SOURCE_ID = "aruco-detections-source";

ArucoMarkerLayerManager::ArucoMarkerLayerManager(ImApplication& application, MapWidget& widget) : MaplibreLayerManager(application, widget) {
    subscription = application.create_subscription<ArucoDetections>(
        "/aruco_detections",
        rclcpp::SensorDataQoS(),
        [this](const ArucoDetections::ConstSharedPtr& msg) {
            onArucoDetections(msg);
        }
    );
}

ArucoMarkerLayerManager::~ArucoMarkerLayerManager() {
    // TODO 2026-10-29 (Will Free): make sure that it has been uninstalled?

    assert(marker_layer == nullptr);
    assert(marker_source == nullptr);
}

ArucoMarkerLayerManager::ArucoMarkerLayerManager(ArucoMarkerLayerManager&& other) noexcept
    : MaplibreLayerManager(std::move(other)),
      subscription(std::move(other.subscription)) {
    std::scoped_lock lock(other.mutex, mutex);

    detected_markers = std::move(other.detected_markers); // NOLINT(*-prefer-member-initializer)
    has_update = std::exchange(other.has_update, false);

    marker_source = std::exchange(other.marker_source, nullptr);
    marker_layer = std::exchange(other.marker_layer, nullptr);
}

void ArucoMarkerLayerManager::install(mln::style::Style& style) {
    std::scoped_lock lock(mutex);

    // TODO 2026-10-29 (Will Free): create aruco fallback image

    /*
     * Generates a fallback marker for a given size
     */
    auto generate_fallback = [](const int border_bits, const int aruco_size) {
        // TODO 2026-10-29 (Will Free): consider other options for the fallback marker?

        const auto is_border_index = [aruco_size, border_bits](const int i) {
            return i < border_bits || i >= aruco_size - border_bits;
        };

        auto image = mln::PremultipliedImage(mln::Size(aruco_size, aruco_size));

        using RGBA = std::array<uint8_t, 4>;
        auto pixels = std::span<RGBA>(reinterpret_cast<RGBA*>(image.data.get()), static_cast<std::size_t>(aruco_size) * aruco_size);

        // TODO 2026-10-30 (Will Free): with higher scales, this generates a checkerboard for every pixel not for every cell
        for (auto row = 0; row < aruco_size; ++row) {
            for (auto col = 0; col < aruco_size; ++col) {
                const bool is_border = is_border_index(row) || is_border_index(col);

                const bool is_black = is_border || (row + col) % 2 == 0;

                const uint8_t value = is_black ? 0 : 0xFF;

                const auto index = row * aruco_size + col;

                // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access): this is safe and .at() was only added to span in C++26
                pixels[index] = {value, value, value, 0xFF};
            }
        }

        return image;
    };

    /*
     * Generates an ArUco marker for a given size & id from a dictionary
     */
    auto generate_marker = [](const int border_bits, const int aruco_size, const cv::aruco::Dictionary& dictionary, const int id) {
        // this is inefficient and allocates a bunch of cv::Mats,
        // but it's fine because this is only run once.
        auto marker_image = cv::Mat(aruco_size, aruco_size, CV_8UC1, cv::Scalar::all(0));

        // generate marker using OpenCV
        dictionary.generateImageMarker(id, aruco_size, marker_image, border_bits);

        // from some testing, it seems that generateImageMarker() always emits a CV_8UC1 image.
        // if this ever changes, then this code needs updating.
        assert(marker_image.type() == CV_8UC1);

        auto image = mln::PremultipliedImage(mln::Size(aruco_size, aruco_size));

        using RGBA = std::array<uint8_t, 4>;
        auto pixels = std::span<RGBA>(reinterpret_cast<RGBA*>(image.data.get()), static_cast<std::size_t>(aruco_size) * aruco_size);

        // copy image from cv::Mat into mln::PremultipliedImage
        for (auto row = 0; row < aruco_size; ++row) {
            for (auto col = 0; col < aruco_size; ++col) {
                const auto index = row * aruco_size + col;

                const auto value = marker_image.at<uint8_t>(row, col);

                // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access): this is safe and .at() was only added to span in C++26
                pixels[index] = {value, value, value, 0xFF};
            }
        }

        return image;
    };

    // TODO 2026-10-29 (Will Free):
    //  consider eventually doing the following:
    //  - putting the dictionary in the ArucoMarker message
    //  - pre-generating images for ALL the ArUco tag dictionaries
    //  from some rough calculations, pre-generating images for all the dictionaries
    //  at the following scales will consume this much memory:
    //  - scale=1..1: ~1.23 MiB
    //  - scale=1..4: ~36.8 MiB
    //  - scale=1..6: ~112 MiB
    //  - scale=1..8: ~250 MiB

    static constexpr auto ARUCO_DICTIONARY = cv::aruco::DICT_4X4_50;
    const auto dictionary = cv::aruco::getPredefinedDictionary(ARUCO_DICTIONARY);

    static constexpr auto BORDER_BITS = 1;
    // TODO 2026-10-29 (Will Free): maybe consider larger sizes rather than software scaling?
    static constexpr auto MARKER_SCALE = 6;

    const auto aruco_size = (dictionary.markerSize + BORDER_BITS * 2) * MARKER_SCALE; // for square pixels, it should be size + border * 2

    auto aruco_fallback = std::make_unique<mln::style::Image>("aruco_fallback", generate_fallback(BORDER_BITS, aruco_size), 1.0);
    style.addImage(std::move(aruco_fallback));

    // generate aruco_{id} marker images
    for (auto i = 0; i < dictionary.bytesList.rows; ++i) {
        // TODO 2026-10-29 (Will Free): consider generating markers at different resolutions,
        //  e.g. aruco_{id}_{scale}? (maybe instead of scale, it should be like floor((size*scale)/8),
        //  then only include the nearest values?
        //  that way if/when multiple dictionaries are added,
        //  aruco_4x4_{id}_2 and aruco_6x6_{id}_2 are about the same size

        auto marker = generate_marker(BORDER_BITS, aruco_size, dictionary, i);
        auto image = std::make_unique<mln::style::Image>(fmt::format("aruco_{}", i), std::move(marker), 1.0);
        style.addImage(std::move(image));
    }

    // TODO 2026-10-29 (Will Free): set clustering?
    auto source = std::make_unique<mln::style::GeoJSONSource>(ARUCO_SOURCE_ID);
    this->marker_source = source.get();

    // initialize GeoJSON source with no markers
    source->setGeoJSON(mln::GeoJSON(mln::FeatureCollection()));

    style.addSource(std::move(source));

    {
        auto layer = std::make_unique<mln::style::SymbolLayer>(ARUCO_LAYER_ID, ARUCO_SOURCE_ID);

        using namespace mln::style::expression;

        layer->setIconImage(
            /*
            [
                "coalesce",
                ["image", ["concat", "aruco_", ["get", "aruco_id"]]],
                ["image", "aruco_fallback"]
            ]
            */
            mln::style::PropertyExpression<Image>(
                std::make_unique<Coalesce>(
                    type::Image,
                    dsl::vec(
                        dsl::image(dsl::concat(dsl::vec(dsl::literal("aruco_"), dsl::toString(dsl::get("aruco_id"))))),
                        dsl::image("aruco_fallback")
                    )
                )
            )
        );

        layer->setTextField(mln::style::PropertyValue<Formatted>(dsl::toFormatted(dsl::get("aruco_id"))));
        layer->setTextColor(mln::Color(1.0, 1.0, 1.0, 1.0));
        layer->setTextHaloColor(mln::Color(0.2, 0.2, 0.2, 1.0));
        layer->setTextHaloWidth(1.1);
        layer->setTextFont(std::vector<std::string>{"Roboto Regular"});
        layer->setTextSize(24);

        // TODO 2026-10-29 (Will Free): consider layer->setSymbolSortKey()?
        // TODO 2026-10-29 (Will Free): consider layer->setIconAllowOverlap(true) & layer->setTextAllowOverlap(true)?
        // TODO 2026-10-29 (Will Free): consider layer->setIconRotationAlignment(AlignmentType::Map) & layer->setIconPitchAlignment(AlignmentType::Map)?

        layer->setIconSize(1.0);

        /* bottom makes it render above (?) */
        layer->setTextAnchor(mln::style::SymbolAnchorType::Bottom);
        layer->setTextRadialOffset(1.2);
        layer->setTextJustify(mln::style::TextJustifyType::Center);

        this->marker_layer = layer.get();

        style.addLayer(std::move(layer));
    }

    has_update = true;
}

void ArucoMarkerLayerManager::uninstall(mln::style::Style& style) {
    std::scoped_lock lock(mutex);

    const auto removed_layer = style.removeLayer(ARUCO_LAYER_ID);
    const auto removed_source = style.removeSource(ARUCO_SOURCE_ID);

    assert(removed_layer.get() == marker_layer);
    assert(removed_source.get() == marker_source);

    marker_layer = nullptr;
    marker_source = nullptr;
}

void ArucoMarkerLayerManager::update() {
    std::scoped_lock lock(mutex);

    assert(marker_layer != nullptr);
    assert(marker_source != nullptr);

    if (!has_update)
        return;

    has_update = false;

    // rebuild feature collection on update
    // there isn't really a better way to do this,
    // and updating the GeoJSON causes maplibre to rebuild its internal data structure anyways.

    auto collection = mln::FeatureCollection();

    namespace geojson = mapbox::geojson;

    for (auto& [id, position] : detected_markers) {
        auto properties = std::unordered_map<std::string, mln::Value>();
        // TODO 2026-10-29 (Will Free): I think this breaks if it's not a string
        properties["aruco_id"] = std::to_string(id);

        auto geometry = geojson::point(position.longitude, position.latitude);

        auto feature = geojson::feature(geometry, properties);

        collection.emplace_back(geometry, properties);
    }

    marker_source->setGeoJSON(mln::GeoJSON{collection});
}

void ArucoMarkerLayerManager::enable() {
    std::scoped_lock lock(mutex);

    assert(marker_layer != nullptr);
    assert(marker_source != nullptr);

    marker_layer->setVisibility(mln::style::VisibilityType::Visible);
}

void ArucoMarkerLayerManager::disable() {
    std::scoped_lock lock(mutex);

    assert(marker_layer != nullptr);
    assert(marker_source != nullptr);

    marker_layer->setVisibility(mln::style::VisibilityType::None);
}

void ArucoMarkerLayerManager::onArucoDetections(const ArucoDetections::ConstSharedPtr& msg) {
    for (auto&& aruco_marker : msg->markers) {
        const auto pose = geometry_msgs::build<PoseStamped>()
                          .header(aruco_marker.header)
                          .pose(aruco_marker.pose);

        widget.toLatLong(pose, [this, aruco_marker](const GeoPoint& point) {
            // we're just hoping that the manager doesn't get free'd before this callback is invoked

            std::scoped_lock lock(mutex);

            // TODO 2026-10-29 (Will Free): perform clustering on close markers of the same id

            const auto marker = DetectedArucoMarker{
                .id       = aruco_marker.id,
                .position = point,
            };
            detected_markers.push_back(marker);

            has_update = true;
        });
    }
}
