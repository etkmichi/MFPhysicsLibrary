/*
 * MFBulletManager.cpp
 *
 *  Created on: 17.10.2019
 *      Author: michl
 */

#include "MFBulletManager.h"

MFBulletManager::MFBulletManager() {
	// TODO Auto-generated constructor stub
	mp_dynamicsWorld=nullptr;
	mp_collisionConfiguration=nullptr;
	mp_solver=nullptr;
	mp_overlappingPairCache=nullptr;
	mp_dispatcher=nullptr;
}

MFBulletManager::~MFBulletManager() {
	// TODO Auto-generated destructor stub
}

