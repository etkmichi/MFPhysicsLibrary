#ifndef MFBULLETSTRUCTS_H_
#define MFBULLETSTRUCTS_H_

#include "btBulletDynamicsCommon.h"
#include <glm/glm.hpp>

struct PhysicalObjectData{
	btCollisionShape *pShape;
	btRigidBody *pBody;
	btDefaultMotionState *pMotionState;
	btRigidBody::btRigidBodyConstructionInfo *pConstructionInfo;
	btVector3 initPosition;
	btVector3 initLinearVelocity;
	btVector3 initRotVelocity;
	btQuaternion initRotQuat;
	btTransform initialTransform;
	float *initModelMatrix=nullptr;
	btVector3 localInertia;
	btScalar mass;
	uint32_t index;

	void init(){
//		initTransform();
		initInertia();
	}
	/**
	 * Set up the transformation using the initial porsition, rotation axe and rot angle.
	 */
	void initTransform(){
	  if(initModelMatrix==nullptr){
	    initialTransform.setIdentity();
	    initialTransform.setOrigin(initPosition);
	  }else{
      initialTransform.setIdentity();
      initialTransform.setOrigin(initPosition);
	    initialTransform.setFromOpenGLMatrix(initModelMatrix);
	  }
	}
	void initInertia(){
		if(mass==0.0f){
			localInertia=btVector3(0,0,0);
			return;
		}
		pShape->calculateLocalInertia(
				mass,
				localInertia);
	}
	void initConstructionInfo(){
    pConstructionInfo = new
      btRigidBody::btRigidBodyConstructionInfo(
        mass,
        pMotionState,
        pShape,
        localInertia);
    pConstructionInfo->m_additionalDamping=false;
    pConstructionInfo->m_angularDamping=0.001f;
    pConstructionInfo->m_linearDamping=0.001f;
	}
	void initBody(){
	  if(pBody==nullptr)pBody=new btRigidBody(*pConstructionInfo);
    pBody->setCcdMotionThreshold(1e-7);
    pBody->setCcdSweptSphereRadius(0.01f);
    /*if DISABLE_DEACTIVATION not used,
     * it can happen that a player object gets caught at the current position
     * and input (with setVelocity) wont change the position of this object
     * (maybe the object gets deactivated by bullet, if bullet registrates
     * external manipulation)*/
    pBody->setActivationState(DISABLE_DEACTIVATION);
    pBody->setCollisionFlags(pBody->getCollisionFlags() |
        btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	  pBody->setLinearVelocity(initLinearVelocity);
	  pBody->setAngularVelocity(initRotVelocity);
	  pBody->setMassProps(mass, localInertia);
	}
};

#endif
