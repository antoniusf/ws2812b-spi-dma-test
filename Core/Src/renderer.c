#include "main.h"
#include "math.h"

void *init_renderer(void) {
  return NULL;
}

void render_frame(void *data, FrameBuffer *fb, uint32_t time) {
  clear_framebuffer(fb);
  for (int i=0; i<fb->num_leds; i++) {
    framebuffer_append_hsv(fb, fmod((double) i/(double) fb->num_leds + (double) time / 10000, 1.), 1., 0.1);
  }
}
