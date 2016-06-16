#pragma once

namespace PerseusLib
{
namespace Objects
{
class View3DParams
{
public:
  float zNear, zFar;
  float zBufferOffset;

  View3DParams(void) {
    zBufferOffset = 0.0001f;
	zFar = 5000000000.f;
    zNear = 1.f;
  }


  View3DParams(float f_zFar, float f_zNear) {
    zBufferOffset = 0.0001f;
    zFar = f_zFar;
    zNear = f_zNear;
  }

  ~View3DParams(void) {}
};
}
}
