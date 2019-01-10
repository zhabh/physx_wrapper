#include "physx_world.h"

#include "crucis/base/Engine/KG_Time.h"
#include "triggers/triggers.h"

#include <ctype.h>
#include <vector>
#include <algorithm>

class SpellData{};
class KHero{};

PhysxObjectType PhysxGetUserType(const physx::PxRigidActor* actor);

/////////////////////////////////////////////////////////////////////////
// PhysxWorldRaycastResultCb
/////////////////////////////////////////////////////////////////////////

struct PhysxWorldRaycastResultCb : public physx::PxQueryFilterCallback {
  PhysxWorldRaycastResultCb(const physx::PxVec3& rayFromWorld,
                            const physx::PxVec3& rayToWorld)
      : m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld) {}

  physx::PxVec3
      m_rayFromWorld;  // used to calculate hitPointWorld from hitFraction
  physx::PxVec3 m_rayToWorld;

  physx::PxVec3 m_hitNormalWorld;
  physx::PxVec3 m_hitPointWorld;

  SpellData* spell_;

  /***********************************************************
  返回值PxQueryHitType::Enum
    eNONE，表示shape不许参与后面的query处理。
    eBLOCK，表示shape是hit最终结束在的物体blocking
  hit，凡是比该shape更远的shape都不必参与query。(该shape并没有被过滤掉，仍然是参与query的。)
    eTOUCH，表示shape是一个touching hit，参与query处理，除非后续filter
  callback给它过滤掉了。
  ************************************************************/
  virtual physx::PxQueryHitType::Enum preFilter(
      const physx::PxFilterData& filterData, const physx::PxShape* shape,
      const physx::PxRigidActor* actor, physx::PxHitFlags&) {
    ////filterData need adjust？？？

    if (spell_) {
      const physx::PxRigidActor* obj0 = actor;
      physx::PxRigidActor* obj1 =
          (physx::PxRigidActor*)spell_->spell_collision_obj2;

      if (obj0 != nullptr && obj1 != nullptr) {
        int type0 = PhysxGetUserType(obj0);
        int type1 = PhysxGetUserType(obj1);
        if ((type0 == PhysxObjectType::PX_SPELL) &&
            (type1 == PhysxObjectType::PX_AIRSHIP)) {
          SpellData* spell =
              static_cast<ClassType*>(obj0->userData)->is<SpellData*>();
          KHero* air_ship =
              static_cast<ClassType*>(obj1->userData)->is<KHero*>();
		  
		  return physx::PxQueryHitType::eNONE;
        }
      }
    }
    return physx::PxQueryHitType::eBLOCK;
  }

  virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData&,
                                                 const physx::PxQueryHit&) {
    return physx::PxQueryHitType::eNONE;
  }
};

/////////////////////////////////////////////////////////////////////////
// global tools
/////////////////////////////////////////////////////////////////////////

PhysxObjectType PhysxGetUserType(const physx::PxRigidActor* actor) {
  if (actor->userData == nullptr) {
    return PhysxObjectType::PX_STATICOBJECT;
  } else {
    ClassType* actor0 = static_cast<ClassType*>(actor->userData);
    if (actor0 == nullptr) {
      return PhysxObjectType::PX_INVALIDE;
    }
    return actor0->getPhysxType();
  }
  return PhysxObjectType::PX_INVALIDE;
}

physx::PxFilterFlags PhysxWorldFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags& pairFlags,
    const void* constantBlock, physx::PxU32 constantBlockSize) {
  // let triggers through
  //   if (physx::PxFilterObjectIsTrigger(attributes0) ||
  //       physx::PxFilterObjectIsTrigger(attributes1)) {
  //     pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
  //     return physx::PxFilterFlag::eDEFAULT;
  //   }
  //   pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
  //
  //   if ((filterData0.word0 & filterData1.word1) &&
  //       (filterData1.word0 & filterData0.word1))
  //     pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
  //
  //   return physx::PxFilterFlag::eDEFAULT | physx::PxFilterFlag::eCALLBACK;
  return physx::PxFilterFlag::eCALLBACK;
}
class MyOwnerFilterCallbackPhysx : public physx::PxSimulationFilterCallback {
  virtual physx::PxFilterFlags pairFound(
      physx::PxU32 pairID, physx::PxFilterObjectAttributes attributes0,
      physx::PxFilterData filterData0, const physx::PxActor* a0,
      const physx::PxShape* s0, physx::PxFilterObjectAttributes attributes1,
      physx::PxFilterData filterData1, const physx::PxActor* a1,
      const physx::PxShape* s1, physx::PxPairFlags& pairFlags) {
    // let triggers through
    if (physx::PxFilterObjectIsTrigger(attributes0) ||
        physx::PxFilterObjectIsTrigger(attributes1)) {
      pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
    }

    pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;

    if ((filterData0.word0 & filterData1.word1) &&
        (filterData1.word0 & filterData0.word1))
      pairFlags = physx::PxPairFlag::eSOLVE_CONTACT |
                  physx::PxPairFlag::eDETECT_DISCRETE_CONTACT |
                  physx::PxPairFlag::eNOTIFY_TOUCH_FOUND |
                  physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS |
                  physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;

    auto first = static_cast<const physx::PxRigidActor*>(a0);
    auto second = static_cast<const physx::PxRigidActor*>(a1);
    if (first == nullptr || second == nullptr)
      return physx::PxFilterFlag::eKILL;
    PhysxObjectType type0 = PhysxGetUserType(first);
    PhysxObjectType type1 = PhysxGetUserType(second);
    if ((type0 == PX_SPELL) && (type1 == PX_AIRSHIP)) {
      SpellData* spell =
          static_cast<ClassType*>(first->userData)->is<SpellData*>();
      KHero* air_ship = static_cast<ClassType*>(second->userData)->is<KHero*>();

	  return physx::PxFilterFlag::eKILL;
    }
    return physx::PxFilterFlag::eDEFAULT;
  }
  virtual void pairLost(physx::PxU32 pairID,
                        physx::PxFilterObjectAttributes attributes0,
                        physx::PxFilterData filterData0,
                        physx::PxFilterObjectAttributes attributes1,
                        physx::PxFilterData filterData1, bool objectRemoved) {}
  virtual bool statusChange(physx::PxU32& pairID, physx::PxPairFlags& pairFlags,
                            physx::PxFilterFlags& filterFlags) {
    return false;
  }
};

/////////////////////////////////////////////////////////////////////////
// PhysxWorld
/////////////////////////////////////////////////////////////////////////

PhysxWorld::PhysxWorld() {
  stamp_ = KG_GetTimeStamp();
  filter_callback_ = nullptr;
  px_scene_ = nullptr;
  trigger_callbacks_.clear();
  is_init_world_ = false;
}

PhysxWorld::~PhysxWorld() {
  if (is_init_world_) {
    DeleteCollisionWorld();
  }
}

physx::PxBoxGeometry* PhysxWorld::CreateBoxShap(physx::PxReal lon,
                                                physx::PxReal wide,
                                                physx::PxReal height) {
  return new physx::PxBoxGeometry(lon, wide, height);
}

physx::PxSphereGeometry* PhysxWorld::CreateSphereShap(physx::PxReal radius) {
  return new physx::PxSphereGeometry(radius);
}

physx::PxCapsuleGeometry* PhysxWorld::CreateCapsuleShape(physx::PxReal radius,
                                                         physx::PxReal height,
                                                         int capsuleAxis) {
  return new physx::PxCapsuleGeometry(radius, height * 0.5f);
}

physx::PxGeometry* PhysxWorld::CreateGeometry(int shape_type,
                                              physx::PxVec3& min_position,
                                              physx::PxVec3& max_position) {
  physx::PxGeometry* geometry = nullptr;

  physx::PxVec3 size;
  size.x = (fabs(max_position.x - min_position.x));
  size.y = (fabs(max_position.y - min_position.y));
  size.z = (fabs(max_position.z - min_position.z));

  switch (shape_type) {
    case PhysxShapeType::PX_SPHERE: {
      float max_r = std::max(size.x, size.y);

      max_r = std::max(max_r, size.z);

      geometry = CreateSphereShap(max_r * 0.5f);

      break;
    }

    case PhysxShapeType::PX_BOX: {
      geometry = CreateBoxShap(size.x * 0.5f, size.y * 0.5f, size.z * 0.5f);

      break;
    }

    case PhysxShapeType::PX_CAPSULE: {
      float len = std::max(size.x, size.y);
      len = std::max(len, size.z);
      int direction = 0;
      float radius = 0.0;
      float height = len;

      if (len == size.x) {
        direction = 0;
        radius = std::max(size.y, size.z) * 0.5f;
      } else if (len == size.y) {
        direction = 1;
        radius = std::max(size.x, size.z) * 0.5f;
      } else if (len == size.z) {
        direction = 2;
        radius = std::max(size.x, size.y) * 0.5f;
      }

      geometry = CreateCapsuleShape(radius, height - 2 * radius, direction);

      break;
    }
  }

  return geometry;
}

physx::PxRigidActor* PhysxWorld::AddCollisionToDynamicsWord(
    int mass, void* userPointer, int userIndex, int userIndex2, int shape_type,
    physx::PxVec3& min_position, physx::PxVec3& max_position,
    physx::PxVec3 startPosition, physx::PxQuat startOrientation,
    KVector3D* hero_position) {
  physx::PxGeometry* geometry =
      CreateGeometry(shape_type, min_position, max_position);

  if (geometry == nullptr) return nullptr;

  physx::PxVec3 center;
  center.x = ((max_position.x + min_position.x) * 0.5f);
  center.y = ((max_position.y + min_position.y) * 0.5f);
  center.z = ((max_position.z + min_position.z) * 0.5f);

  if (mass <= 0) {
    startPosition = center;
  }

  physx::PxSceneWriteLock scopedLock(*px_scene_);

  physx::PxRigidDynamic* dynamic = physx::PxCreateDynamic(
      physx_common::GetInst()->GetPhysics(),
      physx::PxTransform(startPosition, startOrientation), *geometry,
      physx_common::GetInst()->GetDefaultMaterial(), 10.0f);

  if (dynamic == nullptr) {
    delete geometry;
    return nullptr;
  }

  // add collision mask
  if (mass > 0)
    SetupFiltering(dynamic, PX_COL_AIR_SHIP, MaskAirShipCollidesWith);
  else
    SetupFiltering(dynamic, PX_COL_STATIC_OBJ, MaskStaticCollidesWith);

  AddPhysicsActors(dynamic);

  dynamic->userData =
      ClassType::CreateClassType(userPointer, (PhysxObjectType)userIndex);
  dynamic->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
  dynamic->setAngularDamping(0.5f);
  dynamic->setLinearVelocity(physx::PxVec3(0, 0, 0));
  px_scene_->addActor(*dynamic);

  return dynamic;
}

physx::PxRigidActor* PhysxWorld::AddSpellCollison(
    int mass, void* userPointer, int userIndex, int shape_type, int range,
    physx::PxVec3 startPosition, physx::PxQuat startOrientation,
    KVector3D* spell_position) {
  if (userIndex == PhysxObjectType::PX_SPELL) {
    if (userPointer == nullptr || mass <= 0.) return nullptr;
  }

  physx::PxGeometry* geometry = nullptr;

  switch (shape_type) {
    case PhysxShapeType::PX_SPHERE: {
      geometry = CreateSphereShap(range);
      break;
    }
    case PhysxShapeType::PX_BOX: {
      break;
    }
    case PhysxShapeType::PX_CAPSULE: {
      geometry = CreateCapsuleShape(10, 80, 2);
      break;
    }
    default:
      return nullptr;
  }

  physx::PxSceneWriteLock scopedLock(*px_scene_);

  physx::PxRigidDynamic* dynamic = physx::PxCreateDynamic(
      physx_common::GetInst()->GetPhysics(),
      physx::PxTransform(startPosition, startOrientation), *geometry,
      physx_common::GetInst()->GetDefaultMaterial(), 10.0f);

  if (dynamic == nullptr) return nullptr;

  SetupFiltering(dynamic, PX_COL_SPELL_OBJ, MaskSpellCollidesWith);

  AddPhysicsActors(dynamic);
  dynamic->userData =
      ClassType::CreateClassType(userPointer, (PhysxObjectType)userIndex);
  dynamic->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
  dynamic->setAngularDamping(0.5f);
  dynamic->setLinearVelocity(physx::PxVec3(0, 0, 0));
  px_scene_->addActor(*dynamic);

  return dynamic;
}

void PhysxWorld::SetTrigger(physx::PxRigidDynamic& actor) {
  actor.setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
  physx::PxShape* treasureShape;
  actor.getShapes(&treasureShape, 1);
  treasureShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, false);
  treasureShape->setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE, true);
}

int PhysxWorld::StepSimulation() {
  uint64_t time_start = KG_GetTimeStamp();

  uint64_t delta_time = (time_start - stamp_);

  uint64_t old_stamp = stamp_;

  if (delta_time < STEP_MILISECOND) return 0;

  int size = int(delta_time / STEP_MILISECOND);

  int delta_stamp = delta_time - size * STEP_MILISECOND;

  int coun = 0;

  // physx::PxSceneWriteLock lock(*px_scene_);

  for (int i = 0; i < size; i++) {
    secene->UpdateHeroSpeed();
    secene->UpdateSpellSpeed();

    px_scene_->simulate(STEP_SECOND);
    px_scene_->fetchResults(true);

    SyncUser();
  }

  stamp_ = stamp_ + size * STEP_MILISECOND;

  return size;
}

void PhysxWorld::OnInit() {
  if (filter_callback_ != nullptr) {
    delete filter_callback_;
    filter_callback_ = nullptr;
  }

  filter_callback_ = new MyOwnerFilterCallbackPhysx();

  trigger_callbacks_.clear();

  physx_common::GetInst();

  CreatePxScene();

  is_init_world_ = true;
}

void PhysxWorld::CreatePxScene() {
  if (px_scene_) {
    px_scene_->release();
    px_scene_ = nullptr;
  }
  physx::PxSceneDesc sceneDesc(
      physx_common::GetInst()->GetPhysics().getTolerancesScale());
  sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, 0.0f);
  sceneDesc.cpuDispatcher = physx_common::GetInst()->GetCpuDispatcher();
  sceneDesc.filterShader = PhysxWorldFilterShader;
  sceneDesc.simulationEventCallback = this;
  sceneDesc.filterCallback = filter_callback_;
  px_scene_ = physx_common::GetInst()->GetPhysics().createScene(sceneDesc);
  if (px_scene_ == nullptr) {
    LogMgr::GetInstance()->Error(
        "PhysxWorld::CreatePxScene() error:px_scene_ is null");
    return;
  }
#ifdef _DEBUG
  physx::PxPvdSceneClient* pvdClient = px_scene_->getScenePvdClient();
  if (pvdClient) {
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS,
                               true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES,
                               true);
  }
#endif  // _DEBUG
}

void PhysxWorld::SyncUser() {
  physx::PxU32 nbDynamics =
      px_scene_->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);

  if (nbDynamics < 1) return;

  std::vector<physx::PxRigidActor*> actors(nbDynamics);

  px_scene_->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC,
                       reinterpret_cast<physx::PxActor**>(&actors[0]),
                       nbDynamics);

  physx::PxSceneWriteLock scopedLock(*px_scene_);

  for (physx::PxU32 a = 0; a < nbDynamics; ++a) {
    physx::PxRigidActor* actor = actors[a]->is<physx::PxRigidActor>();
    if (actor == nullptr) continue;
    physx::PxShape* shape;
    actor->getShapes(&shape, 1);

    PhysxObjectType type0 = PhysxGetUserType(actor);
    if (type0 == PhysxObjectType::PX_AIRSHIP) {
      KHero* hero = static_cast<ClassType*>(actor->userData)->is<KHero*>();
      if (hero) {
        /*hero->SetPosition(actor->getGlobalPose().p.x,
                          actor->getGlobalPose().p.y,
                          actor->getGlobalPose().p.z);*/
      }
    } else if (type0 == PhysxObjectType::PX_SPELL) {
      SpellData* spell =
          static_cast<ClassType*>(actor->userData)->is<SpellData*>();
      /*spell->spell_start_position_.ConvertFromPhysxVecPos(
          actor->getGlobalPose().p);*/
      
    }
  }
}
void PhysxWorld::DeleteCollisionWorld() {
  if (!is_init_world_ || px_scene_ == nullptr) return;
  physx::PxU32 error;
  px_scene_->simulate(STEP_SECOND);
  px_scene_->fetchResults(true, &error);

  trigger_callbacks_.clear();

  for (std::vector<physx::PxRigidActor*>::iterator actorIter =
           physicsActors_.begin();
       actorIter != physicsActors_.end(); actorIter++) {
    if ((*actorIter) && (*actorIter)->userData) {
      delete (*actorIter)->userData;
      (*actorIter)->userData = NULL;
    }
  }

  RemoveAllActor();

  if (filter_callback_ != nullptr) {
    delete filter_callback_;
    filter_callback_ = nullptr;
  }

  px_scene_->release();
  px_scene_ = nullptr;

  is_init_world_ = false;
}

void PhysxWorld::RemoveAirShipOrSpell(physx::PxRigidActor* obj) {
  if (obj != nullptr) {
    if (obj->userData) {
      delete obj->userData;
      obj->userData = nullptr;
    }

    RemoveActor(obj);
  }
}

void PhysxWorld::SetupFiltering(physx::PxRigidActor* actor,
                                physx::PxU32 filterGroup,
                                physx::PxU32 filterMask) {
  physx::PxSceneWriteLock scopedLock(*px_scene_);

  physx::PxFilterData filterData;
  filterData.word0 = filterGroup;  // word0 = own ID
  filterData.word1 = filterMask;   // word1 = ID mask to filter pairs that
                                   // trigger a contact callback;
  const physx::PxU32 numShapes = actor->getNbShapes();
  physx::PxShape** shapes = new physx::PxShape*[numShapes];
  actor->getShapes(shapes, numShapes);
  for (physx::PxU32 i = 0; i < numShapes; i++) {
    physx::PxShape* shape = shapes[i];
    shape->setSimulationFilterData(filterData);
  }
  delete[] shapes;
}

bool PhysxWorld::RayTest(void* object, physx::PxVec3 start_position,
                         physx::PxVec3 target_position, int mask, int group,
                         PhysxCollisionHitResult& hit_result,
                         physx::PxReal distance) {
  if (object == nullptr) return false;

  SpellData* spellData = (SpellData*)object;

  physx::PxRaycastBuffer rayHit;

  const physx::PxQueryFlags qf(physx::PxQueryFlags(
      physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC |
      physx::PxQueryFlag::ePREFILTER));
  const physx::PxQueryFilterData filterData(
      physx::PxFilterData(group, mask, 0, 0), qf);

  PhysxWorldRaycastResultCb CB(start_position, target_position);
  CB.spell_ = spellData;
  px_scene_->raycast(start_position, target_position, distance, rayHit,
                     physx::PxHitFlag::eDEFAULT, filterData, &CB);

  if (rayHit.hasBlock) {
    hit_result.hit_type = PhysxGetUserType(rayHit.block.actor);
    hit_result.pos = rayHit.block.position;
    hit_result.user_ptr = rayHit.block.actor->userData;
    return true;
  }
  return false;
}

void PhysxWorld::onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) {
  for (physx::PxU32 i = 0; i < count; i++) {
    // ignore pairs when shapes have been deleted
    if (pairs[i].flags & (physx::PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER |
                          physx::PxTriggerPairFlag::eREMOVED_SHAPE_OTHER)) {
      continue;
    }

    if (pairs[i].otherActor == nullptr || pairs[i].triggerActor == nullptr) {
      continue;
    }

    auto Func = std::make_shared<TriggerCallBack>(pairs[i].triggerActor,
                                                  pairs[i].otherActor);

    if (std::find(trigger_callbacks_.begin(), trigger_callbacks_.end(), Func) ==
        trigger_callbacks_.end()) {
      trigger_callbacks_.emplace_back(Func);
    }
  }
}

void PhysxWorld::onContact(const physx::PxContactPairHeader& pairHeader,
                           const physx::PxContactPair* pairs,
                           physx::PxU32 nbPairs) {
  std::vector<physx::PxContactPairPoint> contactPoints;

  for (physx::PxU32 i = 0; i < nbPairs; i++) {
    const physx::PxContactPair& cp = pairs[i];
    PhysxObjectType type0 = PhysxGetUserType(pairHeader.actors[0]);
    PhysxObjectType type1 = PhysxGetUserType(pairHeader.actors[1]);

    if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND) {
      physx::PxU32 contactCount = cp.contactCount;
      if (contactCount < 1) continue;
      contactPoints.resize(contactCount);
      cp.extractContacts(&contactPoints[0], contactCount);

      for (physx::PxU32 j = 0; j < contactCount; j++) {
        if ((type0 == PhysxObjectType::PX_AIRSHIP &&
             type1 == PhysxObjectType::PX_STATICOBJECT) ||
            (type1 == PhysxObjectType::PX_AIRSHIP &&
             type0 == PhysxObjectType::PX_STATICOBJECT)) {
          continue;
        } else if (type0 == PhysxObjectType::PX_AIRSHIP &&
                   type1 == PhysxObjectType::PX_SPELL) {
          SpellData* spell =
              static_cast<ClassType*>(pairHeader.actors[1]->userData)
                  ->is<SpellData*>();
          //if (spell) {
          //  if (spell->hit_result_list2_.size() <= 0) {
          //    PhysxCollisionHitResult hit;
          //    hit.hit_type = (PhysxObjectType)(type0);
          //    hit.pos = contactPoints[j].position;
          //    hit.user_ptr =
          //        static_cast<ClassType*>(pairHeader.actors[0]->userData)
          //            ->is<KHero*>();
          //    spell->hit_result_list2_.push_back(hit);
          //  }
          }
      }
    }
  }
}

void PhysxWorld::UpdateTriggers() {
  const size_t nCount = trigger_callbacks_.size();

  for (physx::PxU32 j = 0; j < nCount; j++) {
    std::shared_ptr<TriggerCallBack> call = trigger_callbacks_[j];

    int trigger_obj_type = PhysxGetUserType(call->trigger_actor_);

	(*call.get())();

    call.reset();
  }
  trigger_callbacks_.clear();
}
