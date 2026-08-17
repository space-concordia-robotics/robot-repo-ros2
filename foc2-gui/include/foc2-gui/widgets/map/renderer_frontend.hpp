#pragma once

#include <memory>
#include <mbgl/renderer/renderer_frontend.hpp>

class MapWidget;

namespace mln {
    class Renderer;
    class UpdateParameters;

    namespace gfx {
        class RendererBackend;
    }
}

class SDL3OpenGLRendererFrontend : public mln::RendererFrontend {
public:
    SDL3OpenGLRendererFrontend(std::unique_ptr<mln::Renderer> renderer, MapWidget& widget, mln::gfx::RendererBackend& backend);
    ~SDL3OpenGLRendererFrontend() override;

    SDL3OpenGLRendererFrontend(const SDL3OpenGLRendererFrontend&) = delete;
    SDL3OpenGLRendererFrontend& operator=(const SDL3OpenGLRendererFrontend&) = delete;

    SDL3OpenGLRendererFrontend(SDL3OpenGLRendererFrontend&&) noexcept = delete;
    SDL3OpenGLRendererFrontend& operator=(SDL3OpenGLRendererFrontend&&) noexcept = delete;

    void reset() override;

    void setObserver(mln::RendererObserver& observer) override;

    void update(std::shared_ptr<mln::UpdateParameters> parameters) override;

    [[nodiscard]] const mln::TaggedScheduler& getThreadPool() const override;

    void render() const;

    [[nodiscard]] mln::Renderer& getRenderer() const;

private:
    std::unique_ptr<mln::Renderer> renderer;
    MapWidget& widget;
    mln::gfx::RendererBackend& backend;
    std::shared_ptr<mln::UpdateParameters> update_parameters = nullptr;
};
