#include <vector>

#include "PxPhysicsAPI.h"
#include "PxDefaultAllocator.h"
#include "PxDefaultErrorCallback.h"
#include "PxPvd.h"
#include "PxPvdTransport.h"
#include "PxPvdSceneClient.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "PxRigidDynamic.h"
#include "PxRigidBody.h"
#include "PxRigidBodyExt.h"
#include "PxRigidActor.h"
#include "PxRigidStatic.h"
#include "PxShape.h"
#include "PxBoxGeometry.h"
#include "PxMaterial.h"
#include "PxCpuDispatcher.h"
#include "PxSimulationEventCallback.h"
#include "PxFiltering.h"
#include "PxContactModifyCallback.h"
#include "PxConstraint.h"
#include "PxFiltering.h"

using namespace physx;

PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}

std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;

class ContactReportCallback : public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) { PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count) { PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
	{
		PX_UNUSED((pairHeader));
		std::vector<PxContactPairPoint> contactPoints;

		for (PxU32 i = 0; i < nbPairs; i++)
		{
			PxU32 contactCount = pairs[i].contactCount;
			if (contactCount)
			{
				contactPoints.resize(contactCount);
				pairs[i].extractContacts(&contactPoints[0], contactCount);

				for (PxU32 j = 0; j < contactCount; j++)
				{
					gContactPositions.push_back(contactPoints[j].position);
					gContactImpulses.push_back(contactPoints[j].impulse);
				}
			}
		}
	}
};

ContactReportCallback gContactReportCallback;

int main()
{
	PxFoundation* mFoundation;
	PxPvd* mPvd;
	PxPhysics* mPhysics;
	PxScene* mScene;
	static PxDefaultErrorCallback gDefaultErrorCallback;
	static PxDefaultAllocator gDefaultAllocatorCallback;

	float frictioncoeff = 0.5;
	float gravityacc = -0.5;

	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback,
		gDefaultErrorCallback);
	if (!mFoundation) {
		printf("PxCreateFoundation failed!\n");
	}

	bool recordMemoryAllocations = true;

	mPvd = PxCreatePvd(*mFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);


	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation,
		PxTolerancesScale(), recordMemoryAllocations, mPvd);
	if (!mPhysics)
		printf("PxCreatePhysics failed!\n");

	PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
	PxDefaultCpuDispatcher* mDis = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = mDis;
	sceneDesc.gravity = PxVec3(0.0f, gravityacc, 0.0f);
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	mScene = mPhysics->createScene(sceneDesc);
	if (!mScene)
		printf("PxCreateScene failed!\n");

	PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	PxTransform origin(PxVec3(0.0, 1.0, 0.0));
	PxMaterial* mMaterial;
	mMaterial = mPhysics->createMaterial(frictioncoeff, frictioncoeff, 0.6f);
	if (!mMaterial)
		printf("PxCreateMaterial failed!\n");

	PxRigidStatic* groundPlane = PxCreatePlane(*mPhysics, PxPlane(0, 1, 0, 0), *mMaterial);
	mScene->addActor(*groundPlane);

	PxShape* box = mPhysics->createShape(PxBoxGeometry(1, 1, 1), *mMaterial);
	if (!box)
		printf("PxCreateShape failed!\n");
	PxRigidDynamic* mObject = mPhysics->createRigidDynamic(origin);
	if (!mObject)
		printf("PxCreateRigidDynamic failed!\n");
	box->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	box->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
	mObject->attachShape(*box);
	mScene->addActor(*mObject);

	//mObject->addForce(PxVec3(0.2, 0.0, 0.0));

	PxTransform anotherO(PxVec3(10.0, 1.0, 0.0));
	PxShape* anotherBox = mPhysics->createShape(PxBoxGeometry(1, 1, 1), *mMaterial);
	PxRigidDynamic* anotherObject = mPhysics->createRigidDynamic(anotherO);
	anotherBox->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	anotherBox->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
	anotherObject->attachShape(*anotherBox);
	mScene->addActor(*anotherObject);

	//anotherObject->addForce(PxVec3(-0.0, 0.0, 0.0));

	for (PxU32 i = 0; i < 20000; i++) {
		if (i >= 10000 && i<= 10020) {
			mObject->addForce(PxVec3(10.0, 0.0, 0.0));
		}

		gContactPositions.clear();
		gContactImpulses.clear();

		mScene->simulate(1.0f / 60.0f);
		mScene->fetchResults(true);
		if (gContactPositions.size() != 0) {
			for (size_t i = 0; i < gContactImpulses.size(); i++) {
				printf("Contact! Impulse: <%f, %f, %f> \n", gContactImpulses[i].x, gContactImpulses[i].y, gContactImpulses[i].z);
			}
			printf("\n");
		}
	}
	
	mMaterial->release();
	box->release();
	mObject->release();
	groundPlane->release();
	mScene->release();
	mPhysics->release();
	mPvd->release();
	mFoundation->release();
	printf("Done. Exiting with status 0.\n");
}
