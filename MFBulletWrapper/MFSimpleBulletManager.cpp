/*
 * MFSimpleBulletManager.cpp
 *
 *  Created on: 17.10.2019
 *      Author: michl
 */

#include "MFSimpleBulletManager.h"
#include "MFObjects/MFObject.h"
#include "btQuickprof.h"

MFSimpleBulletManager::MFSimpleBulletManager() {
	mp_vecDynamicCollisionShapes=new std::vector<PhysicalObjectData*>();
	mp_vecGroundCollisionShapes=new std::vector<PhysicalObjectData*>();
	m_timeStep=1.0f/60.0f;
	m_lastStep=1.0f/60.0f;
	mp_vecDeletables=new std::vector<S_Deletable*>();
}

MFSimpleBulletManager::~MFSimpleBulletManager() {
	delete mp_collisionConfiguration;
	delete mp_dispatcher;
	delete mp_solver;
	delete mp_dynamicsWorld;
	delete mp_overlappingPairCache;
	for(S_Deletable* pDel:*mp_vecDeletables){
		if(pDel!=nullptr){
			pDel->deleteData();
			delete pDel;
			pDel=nullptr;
		}
	}
	delete mp_vecDeletables;
	delete mp_vecGroundCollisionShapes;
}

bool MFSimpleBulletManager::initBullet(){
	mp_collisionConfiguration= new btDefaultCollisionConfiguration();

	mp_dispatcher = new btCollisionDispatcher(mp_collisionConfiguration);
	mp_overlappingPairCache = new btDbvtBroadphase();

	mp_solver = new btSequentialImpulseConstraintSolver();
	mp_dynamicsWorld = new btDiscreteDynamicsWorld (
			mp_dispatcher,
			mp_overlappingPairCache,
			mp_solver,
			mp_collisionConfiguration);
	this->setGravity(glm::vec3(0.0f,0.0f,-9.8f));
	return true;
}

void MFSimpleBulletManager::setGravity(glm::vec3 gVector){
	MFObject::printInfo("MFSimpleBulletManager::setGravity:"+
			std::to_string(gVector.x)+"/"+std::to_string(gVector.y)+"/"+std::to_string(gVector.z));
	mp_dynamicsWorld->setGravity(btVector3(gVector.x,gVector.y,gVector.z));
}

glm::vec3 MFSimpleBulletManager::getGravity(){
  glm::vec3 grav=glm::vec3(
      mp_dynamicsWorld->getGravity().x(),
      mp_dynamicsWorld->getGravity().y(),
      mp_dynamicsWorld->getGravity().z());
  return grav;
}

PhysicalObjectData* MFSimpleBulletManager::addGroundShape(
		glm::vec3 position,
		btCollisionShape* shape){
	PhysicalObjectData* nextGroundObject=new PhysicalObjectData();

	nextGroundObject->pShape=shape;
	nextGroundObject->mass=btScalar(0.0f);
	nextGroundObject->localInertia=btVector3(0,0,0);
	nextGroundObject->initPosition=btVector3(position.x,position.y,position.z);
	nextGroundObject->init();

	/*This will add the PhysicalObjectData for deletion.*/
	S_Deletable* pD=createBulletComponents(nextGroundObject);
	pD->pPO=nextGroundObject;

	nextGroundObject->index=mp_vecGroundCollisionShapes->size();
	mp_vecGroundCollisionShapes->push_back(nextGroundObject);
	mp_dynamicsWorld->addRigidBody(nextGroundObject->pBody);

	return nextGroundObject;
}

bool MFSimpleBulletManager::addGroundShape(PhysicalObjectData* pObject){
	if(pObject->pShape==nullptr){
		MFObject::printErr("MFSimpleBulletManager::addGroundShape - failed,"
				" nextGroundObject->pShape==nullptr");
		return false;
	}
	pObject->mass=btScalar(0.0f);
	pObject->localInertia=btVector3(0,0,0);
	createBulletComponents(pObject);
	pObject->index=mp_vecGroundCollisionShapes->size();
	mp_vecGroundCollisionShapes->push_back(pObject);
	mp_dynamicsWorld->addRigidBody(pObject->pBody);
//	pObject->pBody->setLinearVelocity(pObject->initVelocity);
	return true;
}

S_Deletable* MFSimpleBulletManager::createBulletComponents(PhysicalObjectData* pObject){
	pObject->initTransform();
	S_Deletable *pDel=nullptr;
	if(pObject->pMotionState==nullptr){
		pObject->pMotionState = new btDefaultMotionState(pObject->initialTransform);
		pDel=new S_Deletable();
		pDel->pMS=pObject->pMotionState;
	}else{
	  pObject->pMotionState->setWorldTransform(pObject->initialTransform);
	}
	if(pObject->pConstructionInfo==nullptr){
	  pObject->initConstructionInfo();
		if(pDel==nullptr)pDel=new S_Deletable();
		pDel->pRBI=pObject->pConstructionInfo;
	}
	if(pObject->pBody==nullptr){
		pObject->initBody();
		if(pDel==nullptr)pDel=new S_Deletable();
		pDel->pRB=pObject->pBody;
	}else{
    pObject->initBody();
	}
	if(pDel!=nullptr)mp_vecDeletables->push_back(pDel);
	return pDel;
}

PhysicalObjectData* MFSimpleBulletManager::addDynamicShape(
		glm::vec3 position,
		float mass,
		btCollisionShape* shape){
	glm::vec3 orientation(1.0f,0.0f,0.0f);
	PhysicalObjectData* nextDynamicObject=createObject(
			position,
			mass,
			shape);
	addDynamicObject(nextDynamicObject);
	shape->setMargin(0.4f);
	return nextDynamicObject;
}

PhysicalObjectData* MFSimpleBulletManager::createObject(
		glm::vec3 &position,
		float mass,
		btCollisionShape* shape){
	PhysicalObjectData* nextDynamicObject=new PhysicalObjectData();
	nextDynamicObject->pShape=shape;
	nextDynamicObject->mass=btScalar(mass);
	nextDynamicObject->initPosition=btVector3(position.x,position.y,position.z);
	nextDynamicObject->init();

	S_Deletable* pDel=createBulletComponents(nextDynamicObject);
	pDel->pPO=nextDynamicObject;

	return nextDynamicObject;
}

bool MFSimpleBulletManager::addDynamicObject(PhysicalObjectData* pObject){
	if(pObject->pShape==nullptr){
		MFObject::printErr("MFSimpleBulletManager::addDynamicObject - failed,"
				" nextGroundObject->pShape");
		return false;
	}
	createBulletComponents(pObject);
	pObject->index=mp_vecDynamicCollisionShapes->size();
	mp_vecDynamicCollisionShapes->push_back(pObject);
	//TODO change for 3D simulation!
	pObject->pBody->setLinearFactor(btVector3(1,0,1));
  pObject->pBody->setAngularFactor(btVector3(0,1,0));
	mp_dynamicsWorld->addRigidBody(pObject->pBody);
	return true;
}

bool MFSimpleBulletManager::addObject(PhysicalObjectData* pObject){
	if(pObject->mass>0.0f){
		return addDynamicObject(pObject);
	}else{
		return addGroundShape(pObject);
	}
}

int MFSimpleBulletManager::stepSimulation(float simTimeSeconds){
	m_lastStep+=simTimeSeconds;
	if(m_lastStep>=m_timeStep){
		int steps=mp_dynamicsWorld->stepSimulation(m_lastStep,5);
		m_lastStep=0.0f;
		return steps;
	}
	return 0;
}

void MFSimpleBulletManager::setCollsisionDebugDrawer(btIDebugDraw* pDebugDrawer){
	if(mp_dynamicsWorld!=nullptr){
		mp_dynamicsWorld->setDebugDrawer(pDebugDrawer);
	}
}
