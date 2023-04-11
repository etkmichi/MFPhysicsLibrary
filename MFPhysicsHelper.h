/*
 * MFPhysicsHelper.h
 *
 *  Created on: 17.02.2021
 *      Author: michl
 */

#ifndef MFPHYSICSHELPER_H_
#define MFPHYSICSHELPER_H_
#include <glm/glm.hpp>
#include <btBulletDynamicsCommon.h>
#include <MFBasicDefines.h>
#define GLMV3_TO_BTVEC3(vec3) btVector3(vec3.x,vec3.y,vec3.z)
#define GV3_T_B3(vec3) btVector3(vec3.x,vec3.y,vec3.z)
#define B3(vec3) btVector3(vec3.x,vec3.y,vec3.z)
#define B3XYZ(x,y,z) btVector3(x,y,z)

#define B3TV3(bV3) glm::vec3(bV3.x(),bV3.y(),bV3.z())

class MFPhysicsHelper {
public:
  MFPhysicsHelper();
  virtual ~MFPhysicsHelper();

  static glm::vec3 toVec3(const btVector3& vec){
    glm::vec3 vec3(vec.x(),vec.y(),vec.z());
    return vec3;
  }
};

#endif /* MFPHYSICSHELPER_H_ */
