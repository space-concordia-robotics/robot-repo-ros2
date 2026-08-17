#pragma once

#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_video.h>
#include <mbgl/gfx/renderable.hpp>
#include <mbgl/gl/renderer_backend.hpp>

class SDL3OpenGLRenderableResource;

class SDL3OpenGLRendererBackend : public mbgl::gl::RendererBackend, public mbgl::gfx::Renderable {
public:
    explicit SDL3OpenGLRendererBackend(SDL_Window* window, SDL_GLContext opengl_context);
    ~SDL3OpenGLRendererBackend() override;

    SDL3OpenGLRendererBackend(const SDL3OpenGLRendererBackend&) = delete;
    SDL3OpenGLRendererBackend& operator=(const SDL3OpenGLRendererBackend&) = delete;

    SDL3OpenGLRendererBackend(SDL3OpenGLRendererBackend&&) = delete;
    SDL3OpenGLRendererBackend& operator=(SDL3OpenGLRendererBackend&&) = delete;

    Renderable& getDefaultRenderable() override {
        return *this;
    }

    void updateAssumedState() override;

    void resize(mbgl::Size size);

    [[nodiscard]] GLuint texture() const {
        return color_texture;
    }

protected:
    void activate() override;
    void deactivate() override;

    mbgl::gl::ProcAddress getExtensionFunctionPointer(const char* name) override;

private:
    void swapContext();
    void restoreContext() const;

    SDL_Window* window = nullptr;
    SDL_GLContext opengl_context = nullptr;

    SDL_Window* previous_window = nullptr;
    SDL_GLContext previous_context = nullptr;

    GLuint framebuffer = 0;
    GLuint color_texture = 0;
    GLuint depth_stencil = 0;

    friend class SDL3OpenGLRenderableResource;
};
