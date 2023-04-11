/*
 * MFSimpleBulletManager.h
 *
 *  Created on: 17.10.2019
 *      Author: michl
 */

#ifndef MFBULLETWRAPPER_MFSIMPLEBULLETMANAGER_H_
#define MFBULLETWRAPPER_MFSIMPLEBULLETMANAGER_H_

#include "../MFPhysicStructs.h"
#include <vector>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
struct S_Deletable{
	btMotionState *pMS=nullptr;
	btRigidBody::btRigidBodyConstructionInfo *pRBI=nullptr;
	btRigidBody *pRB=nullptr;
	PhysicalObjectData* pPO=nullptr;
	void deleteData(){
		if(pMS!=nullptr)delete pMS;
		if(pRBI!=nullptr)delete pRBI;
		if(pRB!=nullptr)delete pRB;
		if(pPO!=nullptr)delete pPO;
	}

};
class MFSimpleBulletManager {
//public:
//	static bool mfContactAddedCallback(
//			btManifoldPoint& cp,
//			const btCollisionObjectWrapper* colObj0Wrap,
//			int partId0,
//			int index0,
//			const btCollisionObjectWrapper* colObj1Wrap,
//			int partId1,
//			int index1){
//		if(colObj1Wrap->getCollisionShape()->getShapeType()==TRIANGLE_SHAPE_PROXYTYPE){
//			const btTriangleShape* triShape = static_cast<const btTriangleShape*>
//				(colObj1Wrap->getCollisionShape());
//			const btVector3* v = triShape->m_vertices1;
//					;
//			btVector3 faceNormals= btCross(v[1]-v[0],v[2]-v[0]);
//			faceNormals.normalize();
//			btVector3 faceNormalsWs=
//					colObj1Wrap->getWorldTransform().getBasis()*faceNormals;
//			float nDotF=btDot(faceNormalsWs,cp.m_normalWorldOnB);
//			if(nDotF<=.0f){
//				cp.m_normalWorldOnB+=-2.0f*nDotF*faceNormalsWs;
//			}
//		}
//		return false;
//	}
//    gContactAddedCallback=MFSimpleBulletManager::mfContactAddedCallback;
private:
	btCollisionConfiguration
		*mp_collisionConfiguration=nullptr;
	btCollisionDispatcher
		*mp_dispatcher=nullptr;
	btBroadphaseInterface
		*mp_overlappingPairCache=nullptr;
	btSequentialImpulseConstraintSolver
		*mp_solver=nullptr;
	btDynamicsWorld
		*mp_dynamicsWorld=nullptr;
	std::vector<S_Deletable*>
		*mp_vecDeletables;
	std::vector<PhysicalObjectData*>
		*mp_vecDynamicCollisionShapes,
		*mp_vecGroundCollisionShapes;
	float
		m_timeStep,
		m_lastStep;
	/**
	 * This function creates some components needed for a bullet physical object.
	 * It will add all create resources to a tracker which is used for deletion.
	 * @param pObject
	 */
	S_Deletable* createBulletComponents(PhysicalObjectData* pObject);
public:
	MFSimpleBulletManager();
	virtual ~MFSimpleBulletManager();

	bool initBullet();

	void setGravity(glm::vec3 gVector);
	glm::vec3 getGravity();

	btDynamicsWorld* getWorld(){
	  return mp_dynamicsWorld;
	}

	//TODO shape fabricaiton -> in MFAbstractGeometry
	PhysicalObjectData* addDynamicShape(
			glm::vec3 position,
			float mass,
			btCollisionShape* shape);

	/**
	 * Creates an physical object and adds it to the physics calculation.
	 * @param position
	 * @param orientation
	 * @param mass
	 * @param shape
	 * @return
	 */
	PhysicalObjectData* createObject(
			glm::vec3& position,
			float mass,
			btCollisionShape* shape);

	bool addGroundShape(PhysicalObjectData* pObject);
	PhysicalObjectData* addGroundShape(glm::vec3 position, btCollisionShape* shape);
	bool addDynamicObject(PhysicalObjectData* pObject);

	bool addObject(PhysicalObjectData* pObject);
	//TODO addDynShape(pos, ObjectSetup)
	/**
	 *
	 * @param simTimeSeconds
	 * @return the count of steps which were made.
	 */
	int stepSimulation(float simTimeSeconds);

	std::vector<PhysicalObjectData*>* getDynamicObjects(){return mp_vecDynamicCollisionShapes;};

	std::vector<PhysicalObjectData*>* getGroundObjects(){return mp_vecGroundCollisionShapes;};

	void setCollsisionDebugDrawer(btIDebugDraw* pDebugDrawer);
};

#endif /* MFBULLETWRAPPER_MFSIMPLEBULLETMANAGER_H_ */
