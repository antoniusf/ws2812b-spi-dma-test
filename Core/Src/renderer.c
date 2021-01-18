#include "main.h"
#include "math.h"

void *init_renderer(void) {
  return NULL;
}

void render_frame(void *data, FrameBuffer *fb, uint32_t time) {
  clear_framebuffer(fb);
  for (int i=0; i<fb->num_leds; i++) {
    if (i % 2 == 0) {
      framebuffer_push_hsv(fb, fmod((double) i/(double) fb->num_leds + (double) time / 10000, 1.), 1., 0.05);
    }
    else {
      framebuffer_push_rgb(fb, 0, 0, 0);
    }
  }
}
