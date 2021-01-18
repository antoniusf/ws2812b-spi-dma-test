#include "main.h"
#include "math.h"

void *init_renderer(void) {
  return NULL;
}

void render_frame(void *data, FrameBuffer *fb, uint32_t time) {
  clear_framebuffer(fb);

  uint32_t ftime = time * 256 / 10000;
  uint32_t hue_increment = 256 / fb->num_leds;
  uint32_t hue = 0;

  for (int i=0; i<fb->num_leds; i++) {
    framebuffer_push_hsv(fb, hue + ftime, 255, 12);

    hue += hue_increment;
  }
}
