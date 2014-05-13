#ifndef RGBDSLAM_RENDERABLE_H
#define RGBDSLAM_RENDERABLE_H

#include <GL/gl.h>
class Renderable {
  public:
  virtual void render() = 0;
};

#endif
