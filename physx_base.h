#pragma once
#include <vector>
#include <physx/PxPhysicsAPI.h>

class PhysxBase {
public:
	PhysxBase();
	virtual ~PhysxBase();
	
		
	// called after simulate() has completed
	virtual			void									OnSubstep(float dtime) {}

	// called after simulate() has completed, but before fetchResult() is called
	virtual			void									OnSubstepPreFetchResult() {}

	// called before simulate() is called
	virtual			void									OnSubstepSetup(float dtime, physx::PxBaseTask* cont) {}
	// called after simulate() has started
	virtual			void									OnSubstepStart(float dtime) {}
public:
	inline void AddPhysicsActors(physx::PxRigidActor* actor);
	void RemoveActor(physx::PxRigidActor* actor);
	void RemoveAllActor();

protected:
	std::vector<physx::PxRigidActor*>				physicsActors_;

};

void PhysxBase::AddPhysicsActors(physx::PxRigidActor* actor){
	physicsActors_.push_back(actor);
}

