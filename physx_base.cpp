
#include <algorithm>
#include <ctype.h>

#include "physx_base.h"

PhysxBase::PhysxBase() {}

PhysxBase::~PhysxBase() {}

void PhysxBase::RemoveActor(physx::PxRigidActor* actor) {
  std::vector<physx::PxRigidActor*>::iterator actorIter =
      std::find(physicsActors_.begin(), physicsActors_.end(), actor);
  if (actorIter != physicsActors_.end()) {
    physicsActors_.erase(actorIter);
    actor->release();
  }
}

void PhysxBase::RemoveAllActor() {
  for (std::vector<physx::PxRigidActor*>::iterator actorIter =
           physicsActors_.begin();
       actorIter != physicsActors_.end(); actorIter++) {
    (*actorIter)->release();
  }

  physicsActors_.clear();
}
