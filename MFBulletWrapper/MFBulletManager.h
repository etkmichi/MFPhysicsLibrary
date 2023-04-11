/*
 * MFBulletManager.h
 *
 *  Created on: 17.10.2019
 *      Author: michl
 */

#ifndef MFBULLETWRAPPER_MFBULLETMANAGER_H_
#define MFBULLETWRAPPER_MFBULLETMANAGER_H_
#include "btBulletDynamicsCommon.h"

class MFBulletManager {
protected:
	btCollisionConfiguration
		*mp_collisionConfiguration;
	btCollisionDispatcher
		*mp_dispatcher;
	btBroadphaseInterface
		*mp_overlappingPairCache;
	btSequentialImpulseConstraintSolver
		*mp_solver;
	btDiscreteDynamicsWorld
		*mp_dynamicsWorld;
public:
	MFBulletManager();
	virtual ~MFBulletManager();

	virtual bool initBullet(){return false;};
};

#endif /* MFBULLETWRAPPER_MFBULLETMANAGER_H_ */
