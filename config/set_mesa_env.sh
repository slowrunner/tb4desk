# set_mesa_env

# Configure the Mesa Environment
# https://docs.mesa3d.org/envvars.html

# Core Mesa environment variables
export MESA_DEBUG=1
export MESA_GL_VERSION_OVERRIDE=4.1
export MESA_GLSL_VERSION_OVERRIDE=410
export MESA_EXTENSION_OVERRIDE="\
  -GL_ARB_buffer_storage \
  -GL_ARB_multi_draw_indirect \
  -GL_ARB_texture_buffer_range \
  -GL_ARB_compute_shader \
  -GL_ARB_ES3_compatibility \
  "
