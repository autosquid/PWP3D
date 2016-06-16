#pragma once

#include <PerseusLib/Others/PerseusLibDefines.h>
#include <PerseusLib/Renderer/Model/ModelFace.h>
#include <PerseusLib/Renderer/Model/ModelVertex.h>

//#include <vector>
#include <string>

namespace Renderer
{
namespace Model3D
{
class ModelGroup
{
public:

  std::vector<ModelFace*> faces;
  std::string groupName;

  ModelGroup(std::string groupName);
  ModelGroup(void);
  ~ModelGroup(void);
};
}
}
