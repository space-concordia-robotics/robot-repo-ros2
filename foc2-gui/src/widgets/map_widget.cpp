#include "foc2-gui/widgets/map_widget.hpp"

/// hack to allow database_file_source.hpp to be included, until https://github.com/maplibre/maplibre-native/issues/4472 is fixed
#define nsel_CONFIG_SELECT_EXPECTED nsel_EXPECTED_NONSTD

#include <fstream>
#include <imgui.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_video.h>
#include <magic_enum/magic_enum.hpp>
#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/gl/renderable_resource.hpp>
#include <mbgl/map/map.hpp>
#include <mbgl/map/map_options.hpp>
#include <mbgl/renderer/renderer.hpp>
#include <mbgl/storage/database_file_source.hpp>
#include <mbgl/storage/file_source.hpp>
#include <mbgl/storage/file_source_manager.hpp>
#include <mbgl/style/style.hpp>
#include <mbgl/util/action_journal_options.hpp>

#include "foc2-gui/util/resources.hpp"
#include "foc2-gui/widgets/map/renderer_backend.hpp"
#include "foc2-gui/widgets/map/renderer_frontend.hpp"

static std::optional<std::filesystem::path> cacheDirectory(const std::string& name) {
    // TODO 2026-05-28 (Will Free): support like osx/windows at some point.

    // NOLINTNEXTLINE(*-mt-unsafe): this is fine, because we're not calling cacheDirectory in multithreaded code, and we immediately copy the value
    if (const char* xdg = std::getenv("XDG_CACHE_HOME"))
        return std::filesystem::path(xdg) / name;
    // NOLINTNEXTLINE(*-mt-unsafe): this is fine, because we're not calling cacheDirectory in multithreaded code, and we immediately copy the value
    if (const char* home = std::getenv("HOME"))
        return std::filesystem::path(home) / ".cache" / name;

    return std::nullopt;
}

static std::optional<std::string> getApiKey() {
    // TODO 2026-08-10 (Will Free): add config
    // TODO 2026-08-10 (Will Free): add other fallbacks

    // NOLINTNEXTLINE(*-mt-unsafe): this is fine, because we're not calling getApiKey in multithreaded code, and we immediately copy the value
    if (const char* apiKey = std::getenv("MAPTILER_API_KEY"))
        return std::string(apiKey);
    else
        return std::nullopt;
}

static std::string readFile(const std::string& filename) {
    if (const auto file = std::ifstream(filename, std::ios::binary); file.good()) {
        std::stringstream data;
        data << file.rdbuf();
        return data.str();
    } else {
        throw std::runtime_error(std::string("Cannot read file ") + filename);
    }
}

static std::string readStyleJson() {
    const auto styleJsonPath = RESOURCES_DIR / "style.json";

    return readFile(styleJsonPath);
}

MapWidget::MapWidget(ImApplication& application)
    : UiWidget(application) {}

MapWidget::~MapWidget() {
    // delete the map, then the frontend, then the backend.
    // they must be deleted in this order to properly clean up their resources.
    map = nullptr;
    rendererFrontend = nullptr;
    backend = nullptr;
}

static constexpr auto MAXIMUM_CACHE_SIZE = 2uL * 1024 * 1024 * 1024;

void MapWidget::onInit() {
    const auto apiKey = getApiKey();
    if (!apiKey.has_value())
        throw std::runtime_error("Could not find api key for maptiler");

    const auto mapTilerConfiguration = mbgl::TileServerOptions::MapTilerConfiguration();
    auto resourceOptions = mbgl::ResourceOptions::Default();
    resourceOptions
        .withApiKey(apiKey.value())
        .withMaximumCacheSize(MAXIMUM_CACHE_SIZE)
        .withTileServerOptions(mapTilerConfiguration);

    if (const auto cache_directory = cacheDirectory(FOC2_PACKAGE_NAME); !cache_directory.has_value()) {
        logger.error("Cannot use offline map cache: Could not find $XDG_CACHE_HOME or $HOME. you may be using an unsupported operating system.");
    } else {
        auto ec = std::error_code();
        std::filesystem::create_directories(cache_directory.value(), ec);
        if (ec) {
            logger.error("Cannot use offline map cache: Could not create directory {} due to OS error: {} ({})", ec.message(), ec.value());
        } else {
            resourceOptions.withCachePath(cache_directory.value() / "cache.sqlite");
        }
    }

    const auto clientOptions = mbgl::ClientOptions();
    auto orderedStyles = mapTilerConfiguration.defaultStyles();

    backend = std::make_unique<SDL3OpenGLRendererBackend>(SDL_GL_GetCurrentWindow(), SDL_GL_GetCurrentContext());

    backend->resize(size);

    const auto databaseFileSource = std::static_pointer_cast<mbgl::DatabaseFileSource>(
        mbgl::FileSourceManager::get()->getFileSource(mbgl::FileSourceType::Database, resourceOptions, clientOptions)
    );
    databaseFileSource->setOfflineMapboxTileCountLimit(100'000);
    databaseFileSource->setMaximumAmbientCacheSize(MAXIMUM_CACHE_SIZE, [](const std::exception_ptr&) {});

    // TODO 2026-08-02 (Will Free): offline mode
    // const auto onlineFileSource = mbgl::FileSourceManager::get()->getFileSource(mbgl::FileSourceType::Network, resourceOptions, clientOptions);
    //
    // if constexpr (false) {
    //     if (onlineFileSource) {
    //         onlineFileSource->setProperty("online-status", false);
    //         logger.info("Application launched in offline mode");
    //     } else {
    //         logger.warn("Network FileSource is not available, only local requests are supported");
    //     }
    // }

    rendererFrontend = std::make_unique<SDL3OpenGLRendererFrontend>(std::make_unique<mbgl::Renderer>(*backend, 1.0), *this, *backend);

    auto actionJournalOptions = mbgl::util::ActionJournalOptions();
    // TODO 2026-08-10 (Will Free): support for action journal
    // if (false) {
    //     const std::string actionJournalDir = args::get(actionJournalDirValue);
    //     actionJournalOptions
    //         .enable(true)
    //         .withPath(actionJournalDir);
    //     mbgl::Log::Info(mbgl::Event::General, "Action journal enabled. Logs will be written to: " + actionJournalDir);
    // }

    auto mapOptions = mbgl::MapOptions();
    mapOptions
        .withMapMode(mbgl::MapMode::Continuous)
        .withConstrainMode(mbgl::ConstrainMode::WidthAndHeight)
        .withFastPFOREnabled(true)
        .withSize(size)
        .withPixelRatio(1.0);


    map = std::make_unique<mbgl::Map>(
        *rendererFrontend,
        *this,
        mapOptions,
        resourceOptions,
        clientOptions,
        actionJournalOptions
    );

    // TODO 2026-08-10 (Will Free): settings for toggling some of the different map layers
    // TODO 2026-08-11 (Will Free): settings to swap between using maptiler & ArcGIS
    map->getStyle().loadJSON(readStyleJson());
    // map->getStyle().loadURL("https://tiles.openfreemap.org/styles/liberty");
}

void MapWidget::onShutdown() {
    // delete the map, then the frontend, then the backend.
    // they must be deleted in this order to properly clean up their resources.
    map = nullptr;
    rendererFrontend = nullptr;
    backend = nullptr;
}

void MapWidget::draw() {
    handleResize();

    // TODO 2026-08-10 (Will Free): check if entire application is focused, not just the window.
    const auto focused = ImGui::IsWindowFocused();

    if (!focused && focused != this->focused) {
        rendererFrontend->getRenderer().reduceMemoryUse();
    }
    this->focused = focused;

    runLoop.runOnce();

    if (dirty && rendererFrontend) {
        dirty = false;

        auto scope = mbgl::gfx::BackendScope(*backend);

        rendererFrontend->render();

        // TODO 2026-08-10 (Will Free): don't invalidate every frame, only invalidate when we actually need to re-render the map
        invalidate();
    }

    // update time after render
    runLoop.updateTime();

    ImGui::Image(
        backend->texture(),
        ImGui::GetContentRegionAvail(),
        // flip the texture because imgui wants (0,0) as the top left
        ImVec2(0, 1),
        ImVec2(1, 0)
    );

    handleScroll();
    handleMouse();
}

void MapWidget::handleScroll() const {
    const ImGuiIO& io = ImGui::GetIO();

    // if not hovered, don't handle mouse events
    if (!ImGui::IsItemHovered())
        return;

    const auto wheel = io.MouseWheel;
    if (std::abs(wheel) < 1e-2)
        return;

    const auto delta = wheel * 40.0;

    const auto absDelta = std::abs(delta);
    auto scale = 2.0 / (1.0 + std::exp(-absDelta / 100.0));

    // Zooming out.
    if (delta < 0 && scale != 0)
        scale = 1.0 / scale;

    map->scaleBy(scale, mbgl::ScreenCoordinate(io.MousePos.x, io.MousePos.y));
}

void MapWidget::handleResize() {
    const auto available = ImGui::GetContentRegionAvail();

    // auto context = ImGui::GetCurrentContext();

    const auto availableSize = mbgl::Size(static_cast<uint32_t>(available.x), static_cast<uint32_t>(available.y));

    if (availableSize == size)
        return;

    size = availableSize;

    map->setSize(size);

    backend->resize(size);

    // This is only triggered when the framebuffer is resized, but not the
    // window. It can happen when you move the window between screens with a
    // different pixel ratio. We are forcing a repaint my invalidating the view,
    // which triggers a rerender with the new framebuffer dimensions.
    invalidate();
}

void MapWidget::handleMouse() {
    handleMouseClick();
    handleMouseMove();
}

void MapWidget::handleMouseMove() const {
    ImGuiIO& io = ImGui::GetIO();

    const auto delta = io.MouseDelta;

    // did not move, or current/previous position was invalid
    if (delta.x == 0.0 && delta.y == 0.0)
        return;

    if (tracking) {
        map->moveBy({delta.x, delta.y});
    } else if (rotating) {
        map->rotateBy({io.MousePosPrev.x, io.MousePosPrev.y}, {io.MousePos.x, io.MousePos.y});
    } else if (pitching) {
        map->pitchBy(delta.y / 2);
    }
}

void MapWidget::handleMouseClick() {
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right) || ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        rotating = false;
        pitching = false;
        tracking = false;
        map->setGestureInProgress(false);
        return;
    }

    // if not hovered, don't handle mouse events
    if (!ImGui::IsItemHovered())
        return;

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) || (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::IsKeyDown(ImGuiMod_Ctrl))) {
        rotating = true;
        map->setGestureInProgress(true);
        return;
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::IsKeyDown(ImGuiMod_Shift)) {
        pitching = true;
        map->setGestureInProgress(true);
        return;
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        tracking = true;
        map->setGestureInProgress(true);
        return;
    }

    if (ImGui::IsAnyMouseDown())
        return;

    rotating = false;
    pitching = false;
    tracking = false;
    map->setGestureInProgress(false);
}

void MapWidget::invalidate() {
    dirty = true;
}

void MapWidget::onDidFailLoadingMap(const mbgl::MapLoadError error, const std::string& reason) {
    logger.info("failed loading map: {} ({})", reason, magic_enum::enum_name(error));
}

void MapWidget::onRenderError(const std::exception_ptr exception_ptr) {
    try {
        if (exception_ptr) {
            std::rethrow_exception(exception_ptr);
        }
    } catch (const std::exception& e) {
        logger.error("MapLibre render exception: {}", e.what());
    } catch (...) {
        logger.error("MapLibre render exception: unknown");
    }
}
