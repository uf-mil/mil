Shaders
=======

These are GLSL (GL Shader Langauge) programs, written for OpenGL. They are compiled, and then run on the GPU. Vertex shaders are executed once for every vertex, and fragment shaders are executed many times, but you can imagine them as being executed once per pixel.

# Shader standard

Shaders must implement

    uniform mat4 u_model
    uniform mat4 u_view
    uniform mat4 u_projection
    attribute vec3 a_position
    attribute vec3 a_normal

All object models MUST set (for each vertex)..

    a_position
    a_normal

# TODO
    * Force all shaders to support alpha