# LibGL environment variables 
export LIBGL_DEBUG=verbose
export LIBGL_ALWAYS_INDIRECT=false
# Set following to true for soft openGL, false for hardware OpenGL
export LIBGL_ALWAYS_SOFTWARE=false
export LIBGL_NO_DRAWARRAYS=false
export LIBGL_SHOW_FPS=true
export LIBGL_DRI2_DISABLE=false
export LIBGL_DRI3_DISABLE=false

# Gallium environment variables
export GALLIUM_HUD="simple,\
  fps\
 +frametime\
 +cpu\
 +cpu0\
 +cpu1\
 +cpu2\
 +cpu3\
 +samples-passed\
 +primitives-generated\
 +num-draw-calls\
 +nic-rx-ens33\
 +nic-tx-ens33\
 +num-fallbacks\
 +num-flushes\
 +num-validations\
 +map-buffer-time\
 +num-buffers-mapped\
 +num-textures-mapped\
 +num-bytes-uploaded\
 +num-command-buffers\
 +command-buffer-size\
 +flush-time\
 +surface-write-flushes\
 +num-readbacks\
 +num-resource-updates\
 +num-buffer-uploads\
 +num-const-buf-updates\
 +num-const-updates\
 +num-shader-relocations\
 +num-surface-relocations\
 +memory-used\
+num-shaders\
+num-resources\
+num-state-objects\
+num-surface-views\
+num-generate-mipmap\
+num-failed-allocations\
+num-commands-per-draw\
+shader-mem-used\
"
