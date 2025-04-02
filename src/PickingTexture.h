#pragma once

// This class is largely based on this video: https://www.youtube.com/watch?v=71G-PVpaVk8

#include <GL/glew.h>
#include <iostream>

class PickingTexture
{
    public:
        struct PixelInfo
        {
            uint objectID;
            uint drawID;
            uint primitiveID;

            void print()
            {
                std::cout << "objectID: " << objectID << "\nDrawID: " << drawID << "\nPrimitiveID: " << primitiveID << "\n\n";
            }
        };

        PickingTexture(GLsizei width, GLsizei height)
        {
            m_width = width;
            m_height = height;

            glGenFramebuffers(1, &m_frameBuffer);
            glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);

            // Make texture
            glGenTextures(1, &m_texture);
            glBindTexture(GL_TEXTURE_2D, m_texture);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, m_width, m_height, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, 0);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texture, 0);
            GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
            glDrawBuffers(1, drawBuffers);

            // Make depth buffer
            glGenRenderbuffers(1, &m_depthBuffer);
            glBindRenderbuffer(GL_RENDERBUFFER, m_depthBuffer);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_width, m_height);

            // Bind the framebuffer together
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthBuffer);
            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_texture, 0);

            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                throw std::runtime_error("Something went wrong making the framebuffer\n");

            glBindTexture(GL_TEXTURE_2D, 0);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);
        }

        PixelInfo readPixel(unsigned int x, unsigned int y)
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, m_frameBuffer);
            glReadBuffer(GL_COLOR_ATTACHMENT0);

            PixelInfo pixel;
            glReadPixels(x, y, 1, 1, GL_RGB_INTEGER, GL_UNSIGNED_INT, &pixel);

            glReadBuffer(GL_NONE);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            return pixel;
        };

        void bind() { glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frameBuffer); }
        void unbind() { glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); }

        ~PickingTexture(){}

    private:
        GLuint m_frameBuffer, m_texture, m_depthBuffer;
        GLsizei m_width, m_height;
};
