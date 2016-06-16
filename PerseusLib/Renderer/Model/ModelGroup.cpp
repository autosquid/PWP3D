#include "ModelGroup.h"
#include <string.h>

using namespace Renderer::Model3D;

ModelGroup::ModelGroup(void)
{
}

ModelGroup::~ModelGroup(void)
{
  size_t i;
  for (i=0;i<faces.size();i++)
    delete faces[i];
}

ModelGroup::ModelGroup(const std::string groupName)
{
  this->groupName = groupName;
}
