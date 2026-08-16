#pragma once

#include <memory>
#include <mbgl/renderer/renderer_frontend.hpp>

class MapWidget;

namespace mbgl {
    class Renderer;
    class UpdateParameters;

    namespace gfx {
        class RendererBackend;
    }
}

class SDL3OpenGLRendererFrontend : public mbgl::RendererFrontend {
public:
    SDL3OpenGLRendererFrontend(std::unique_ptr<mbgl::Renderer> renderer, MapWidget& widget, mbgl::gfx::RendererBackend& backend);
    ~SDL3OpenGLRendererFrontend() override;

    SDL3OpenGLRendererFrontend(const SDL3OpenGLRendererFrontend&) = delete;
    SDL3OpenGLRendererFrontend& operator=(const SDL3OpenGLRendererFrontend&) = delete;

    SDL3OpenGLRendererFrontend(SDL3OpenGLRendererFrontend&&) noexcept = delete;
    SDL3OpenGLRendererFrontend& operator=(SDL3OpenGLRendererFrontend&&) noexcept = delete;

    void reset() override;

    void setObserver(mbgl::RendererObserver& observer) override;

    void update(std::shared_ptr<mbgl::UpdateParameters> parameters) override;

    [[nodiscard]] const mbgl::TaggedScheduler& getThreadPool() const override;

    void render() const;

    [[nodiscard]] mbgl::Renderer& getRenderer() const;

private:
    std::unique_ptr<mbgl::Renderer> renderer;
    MapWidget& widget;
    mbgl::gfx::RendererBackend& backend;
    std::shared_ptr<mbgl::UpdateParameters> update_parameters = nullptr;
};
