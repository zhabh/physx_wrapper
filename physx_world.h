#pragma once
#include "stdafx.h"
#include "physx_base.h"
#include "physx_common.h"

#include "KVector.h"

#define BIT(x) (1 << (x))
#define PX_SAFE_RELEASE(x) \
  if (x) {                 \
    x->release();          \
    x = NULL;              \
  }

#define PVD_HOST "127.0.0.1"

enum PhysxShapeType { PX_BOX = 1, PX_SPHERE = 2, PX_CAPSULE = 3 };

enum PhysxObjectType {
  PX_INVALIDE,
  PX_STATICOBJECT,
  PX_AIRSHIP,
  PX_SPELL,
  PX_SPELLTARGETSTATOCOBJECT
};

enum PhysxCollisionTypes {
  PX_COL_NOTHING = 0,                      //<Collide with nothing
  PX_COL_AIR_SHIP = BIT(1),                //<Collide with ships
  PX_COL_STATIC_OBJ = BIT(2),              //<Collide with static_obj
  PX_COL_SPELL_OBJ = BIT(3),               //<Collide with spell data
  PX_COL_SPELL_TARGET_STATIC_OBJ = BIT(4)  // Collide with spell target obj
};

//碰撞结果
struct PhysxCollisionHitResult {
  PhysxCollisionHitResult() {
    hit_type = PhysxObjectType::PX_INVALIDE;
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    user_ptr = nullptr;
  }
  inline bool HasResult() { return hit_type != PhysxObjectType::PX_INVALIDE; }
  PhysxObjectType hit_type;
  physx::PxVec3 pos;
  void* user_ptr;
};

static const int MaskAirShipCollidesWith = PX_COL_STATIC_OBJ | PX_COL_SPELL_OBJ;
static const int MaskStaticCollidesWith = PX_COL_SPELL_OBJ | PX_COL_AIR_SHIP;
static const int MaskSpellCollidesWith =
    PX_COL_STATIC_OBJ | PX_COL_AIR_SHIP | PX_COL_SPELL_TARGET_STATIC_OBJ;
static const int MaskSpellTargetCollidesWith = PX_COL_SPELL_OBJ;
static const int MaskAirShipNoColldierWith = PX_COL_NOTHING;

struct ClassType {
  ClassType(void* p, const PhysxObjectType type) : ptr_(p), type_(type) {}
  PhysxObjectType getPhysxType() const { return type_; }

  static ClassType* CreateClassType(void* p, const PhysxObjectType type) {
    ClassType* ret = new ClassType(p, type);
    return ret;
  }

  template <typename T>
  T is() {
    if (ptr_ == nullptr || type_ == PhysxObjectType::PX_INVALIDE)
      return nullptr;

    return static_cast<T>(ptr_);
  }

  const PhysxObjectType type_;

 private:
  ClassType& operator=(const ClassType&);
  void* ptr_;
};
class MyOwnerFilterCallbackPhysx;
class PhysxWorldContactReportCallback;
struct TriggerCallBack;

class PhysxWorld : public PhysxBase, public physx::PxSimulationEventCallback {
 public:
  PhysxWorld();
  ~PhysxWorld();

  void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count);
  void onContact(const physx::PxContactPairHeader& pairHeader,
                 const physx::PxContactPair* pairs, physx::PxU32 nbPairs);

  void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) {}
  void onWake(physx::PxActor** actors, physx::PxU32 count) {}
  void onSleep(physx::PxActor** actors, physx::PxU32 count) {}
  void onAdvance(const physx::PxRigidBody* const* ppRigid,
                 const physx::PxTransform* pTrans, const physx::PxU32 nb) {}

  
  //长方体
  physx::PxBoxGeometry* CreateBoxShap(physx::PxReal lon, physx::PxReal wide,
                                      physx::PxReal height);
  //球体
  physx::PxSphereGeometry* CreateSphereShap(physx::PxReal radius);

  //胶囊体
  physx::PxCapsuleGeometry* CreateCapsuleShape(physx::PxReal radius,
                                               physx::PxReal height,
                                               int capsuleAxis);

  //创建几何体
  physx::PxGeometry* CreateGeometry(int shape_type, physx::PxVec3& min_position,
                                    physx::PxVec3& max_position);

  physx::PxRigidActor* AddCollisionToDynamicsWord(
      int mass, void* userPointer, int userIndex, int userIndex2,
      int shape_type, physx::PxVec3& min_position, physx::PxVec3& max_position,
      physx::PxVec3 startPosition, physx::PxQuat startOrientation,
      KVector3D* hero_position = nullptr);

  physx::PxRigidActor* AddSpellCollison(int mass, void* userPointer,
                                        int userIndex, int shape_type,
                                        int range, physx::PxVec3 startPosition,
                                        physx::PxQuat startOrientation,
                                        KVector3D* spell_position = nullptr);

  void SetTrigger(physx::PxRigidDynamic& actor);

  int StepSimulation();

  void OnInit();

  void CreatePxScene();

  physx::PxScene& GetActiveScene() const { return *px_scene_; }

  bool IsInitWorld() const { return is_init_world_; }

  void SyncUser();

  void DeleteCollisionWorld();

  void RemoveAirShipOrSpell(physx::PxRigidActor* obj);

  void SetupFiltering(physx::PxRigidActor* actor, physx::PxU32 filterGroup,
                      physx::PxU32 filterMask);

  bool RayTest(
      void* object, physx::PxVec3 start_position, physx::PxVec3 target_position,
      int mask, int group, PhysxCollisionHitResult& hit_result,
      physx::PxReal distance = std::numeric_limits<physx::PxReal>::max());

  void UpdateTriggers();

 private:
  std::vector<std::shared_ptr<TriggerCallBack> > trigger_callbacks_;
  bool is_init_world_;
  MyOwnerFilterCallbackPhysx* filter_callback_;
  physx::PxScene* px_scene_;
  uint64_t stamp_;
};
