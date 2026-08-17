#include "foc2-gui/widgets/map/renderer_frontend.hpp"

#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/gfx/renderer_backend.hpp>
#include <mbgl/renderer/renderer.hpp>
#include <mbgl/util/instrumentation.hpp>

#include "foc2-gui/widgets/map_widget.hpp"

SDL3OpenGLRendererFrontend::SDL3OpenGLRendererFrontend(std::unique_ptr<mln::Renderer> renderer, MapWidget& widget, mln::gfx::RendererBackend& backend)
    : renderer(std::move(renderer)),
      widget(widget),
      backend(backend) {}

SDL3OpenGLRendererFrontend::~SDL3OpenGLRendererFrontend() = default;

void SDL3OpenGLRendererFrontend::reset() {
    assert(renderer);
    renderer.reset();
}

void SDL3OpenGLRendererFrontend::setObserver(mln::RendererObserver& observer) {
    assert(renderer);
    renderer->setObserver(&observer);
}

void SDL3OpenGLRendererFrontend::update(std::shared_ptr<mln::UpdateParameters> parameters) {
    update_parameters = std::move(parameters);
    widget.invalidate();
}

const mln::TaggedScheduler& SDL3OpenGLRendererFrontend::getThreadPool() const {
    return backend.getThreadPool();
}

void SDL3OpenGLRendererFrontend::render() const {
    MLN_TRACE_FUNC();

    assert(renderer);

    if (!update_parameters)
        return;

    // technically this is not necessary,
    // however if we ever move to vulkan instead of open gl, then it will be.
    auto guard = mln::gfx::BackendScope(backend, mln::gfx::BackendScope::ScopeType::Implicit);

    // onStyleImageMissing might be called during a render. The user implemented
    // method could trigger a call to MLNRenderFrontend#update which overwrites
    // `updateParameters`. Copy the shared pointer here so that the parameters
    // aren't destroyed while `render(...)` is still using them.
    const auto update_parameters = this->update_parameters;
    renderer->render(update_parameters);
}

mln::Renderer& SDL3OpenGLRendererFrontend::getRenderer() const {
    assert(renderer);
    return *renderer;
}
