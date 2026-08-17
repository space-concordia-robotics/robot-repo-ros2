#include "foc2-gui/widgets/map/renderer_backend.hpp"

#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_opengles2.h>
#include <mbgl/gfx/context.hpp>
#include <mbgl/gl/renderable_resource.hpp>

class SDL3OpenGLRenderableResource final : public mbgl::gl::RenderableResource {
public:
    explicit SDL3OpenGLRenderableResource(SDL3OpenGLRendererBackend& backend)
        : backend(backend) {}

    void bind() override {
        backend.setFramebufferBinding(backend.framebuffer);
        backend.setViewport(0, 0, backend.size);

        glDisable(GL_SCISSOR_TEST);

        // Clear the framebuffer
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    }

private:
    SDL3OpenGLRendererBackend& backend;
};

// ReSharper disable once CppParameterMayBeConst
SDL3OpenGLRendererBackend::SDL3OpenGLRendererBackend(SDL_Window* window, SDL_GLContext opengl_context)
    : RendererBackend(mbgl::gfx::ContextMode::Unique),
      Renderable({0, 0}, std::make_unique<SDL3OpenGLRenderableResource>(*this)),
      window(window),
      previous_window(window),
      previous_context(opengl_context) {

    assert(previous_context != nullptr);
    assert(previous_window != nullptr);

    if (!SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1))
        throw std::runtime_error(SDL_GetError());

    this->opengl_context = SDL_GL_CreateContext(window);

    if (this->opengl_context == nullptr) {
        throw std::runtime_error(SDL_GetError());
    }

    if (!SDL_GL_MakeCurrent(previous_window, previous_context)) {
        SDL_GL_DestroyContext(this->opengl_context);
        this->opengl_context = nullptr;

        throw std::runtime_error(SDL_GetError());
    }
}

SDL3OpenGLRendererBackend::~SDL3OpenGLRendererBackend() {
    previous_window = SDL_GL_GetCurrentWindow();
    previous_context = SDL_GL_GetCurrentContext();

    SDL_GL_MakeCurrent(window, opengl_context);

    context->performCleanup();
    context->reduceMemoryUsage();

    if (framebuffer != 0)
        glDeleteFramebuffers(1, &framebuffer);

    if (color_texture != 0)
        glDeleteTextures(1, &color_texture);

    if (depth_stencil != 0)
        glDeleteRenderbuffers(1, &depth_stencil);

    SDL_GL_MakeCurrent(previous_window, previous_context);

    if (opengl_context != nullptr)
        SDL_GL_DestroyContext(opengl_context);
}

void SDL3OpenGLRendererBackend::resize(const mbgl::Size size) {
    swapContext();

    this->size = size;

    // magic opengl shit, do not touch unless you understand opengl - Will Free

    if (color_texture != 0)
        glDeleteTextures(1, &color_texture);

    if (depth_stencil != 0)
        glDeleteRenderbuffers(1, &depth_stencil);

    if (framebuffer != 0)
        glDeleteFramebuffers(1, &framebuffer);

    glGenFramebuffers(1, &framebuffer);

    glGenTextures(1, &color_texture);
    glBindTexture(GL_TEXTURE_2D, color_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size.width, size.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

    glGenRenderbuffers(1, &depth_stencil);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil);
    glRenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH24_STENCIL8, size.width, size.height);

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, color_texture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_stencil);

    const auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    assert(status == GL_FRAMEBUFFER_COMPLETE);

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    restoreContext();
}

void SDL3OpenGLRendererBackend::swapContext() {
    previous_window = SDL_GL_GetCurrentWindow();
    previous_context = SDL_GL_GetCurrentContext();

    if (!SDL_GL_MakeCurrent(window, opengl_context))
        throw std::runtime_error(SDL_GetError());
}

void SDL3OpenGLRendererBackend::restoreContext() const {
    if (!SDL_GL_MakeCurrent(previous_window, previous_context))
        throw std::runtime_error(SDL_GetError());
}

void SDL3OpenGLRendererBackend::activate() {
    swapContext();

    glViewport(0, 0, size.width, size.height);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
}

void SDL3OpenGLRendererBackend::deactivate() {
    restoreContext();
}

mbgl::gl::ProcAddress SDL3OpenGLRendererBackend::getExtensionFunctionPointer(const char* name) {
    return SDL_GL_GetProcAddress(name);
}

void SDL3OpenGLRendererBackend::updateAssumedState() {
    assumeFramebufferBinding(framebuffer);
    assumeViewport(0, 0, size);
}
