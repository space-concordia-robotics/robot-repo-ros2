#include "foc2-gui/widgets/map/renderer_frontend.hpp"

#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/gfx/renderer_backend.hpp>
#include <mbgl/renderer/renderer.hpp>
#include <mbgl/util/instrumentation.hpp>

#include "foc2-gui/widgets/map_widget.hpp"

SDL3OpenGLRendererFrontend::SDL3OpenGLRendererFrontend(std::unique_ptr<mbgl::Renderer> renderer, MapWidget& widget, mbgl::gfx::RendererBackend& backend)
    : renderer(std::move(renderer)),
      widget(widget),
      backend(backend) {}

SDL3OpenGLRendererFrontend::~SDL3OpenGLRendererFrontend() = default;

void SDL3OpenGLRendererFrontend::reset() {
    assert(renderer);
    renderer.reset();
}

void SDL3OpenGLRendererFrontend::setObserver(mbgl::RendererObserver& observer) {
    assert(renderer);
    renderer->setObserver(&observer);
}

void SDL3OpenGLRendererFrontend::update(std::shared_ptr<mbgl::UpdateParameters> parameters) {
    update_parameters = std::move(parameters);
    widget.invalidate();
}

const mbgl::TaggedScheduler& SDL3OpenGLRendererFrontend::getThreadPool() const {
    return backend.getThreadPool();
}

void SDL3OpenGLRendererFrontend::render() const {
    MLN_TRACE_FUNC();

    assert(renderer);

    if (!update_parameters)
        return;

    // technically this is not necessary,
    // however if we ever move to vulkan instead of open gl, then it will be.
    auto guard = mbgl::gfx::BackendScope(backend, mbgl::gfx::BackendScope::ScopeType::Implicit);

    // onStyleImageMissing might be called during a render. The user implemented
    // method could trigger a call to MLNRenderFrontend#update which overwrites
    // `updateParameters`. Copy the shared pointer here so that the parameters
    // aren't destroyed while `render(...)` is still using them.
    const auto update_parameters = this->update_parameters;
    renderer->render(update_parameters);
}

mbgl::Renderer& SDL3OpenGLRendererFrontend::getRenderer() const {
    assert(renderer);
    return *renderer;
}
