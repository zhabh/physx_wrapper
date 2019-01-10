#pragma once
#include <physx/PxPhysicsAPI.h>

/////////////////////////////////////////////////////////////////////////
// physx_common
/////////////////////////////////////////////////////////////////////////
struct KVector3D{ int x;int y;int z;};
#define PI_F 3.1415926f

class physx_common {
 public:
  physx_common();
  ~physx_common();

  static physx_common* GetInst() {
    if (instance_ == nullptr) {
      instance_ = new physx_common();
    }

    return instance_;
  }

  bool InitPhyxsSDK();

  void ReleaseSDK();

  void Destory();

  physx::PxPhysics& GetPhysics() const { return *px_physics_; }
  physx::PxDefaultCpuDispatcher* GetCpuDispatcher() const { return px_dispatcher_; }
  physx::PxMaterial& GetDefaultMaterial() const { return *px_material_; }

  bool IsConnectedPvd() const {
    return px_pvd_ ? px_pvd_->isConnected() : false;
  }

 private:
  static physx_common* instance_;

 protected:
  physx::PxFoundation* px_foundation_;
  physx::PxPhysics* px_physics_;
  physx::PxMaterial* px_material_;
  physx::PxDefaultCpuDispatcher* px_dispatcher_;
  physx::PxPvd* px_pvd_;

  physx::PxDefaultAllocator px_allocator_;
  physx::PxDefaultErrorCallback px_error_callback_;
};


/////////////////////////////////////////////////////////////////////////
// maths tools
/////////////////////////////////////////////////////////////////////////

void SetEulerYXZ(const physx::PxReal& yaw, const physx::PxReal& pitch,
                   const physx::PxReal& roll, physx::PxQuat& quat);


physx::PxVec3 GetQuatRotate(KVector3D& dir, KVector3D& velocity);
physx::PxVec3 QuatToEuler(const physx::PxQuat& q);

void QuatToMatrix(const physx::PxReal *quat,physx::PxReal *matrix);

physx::PxVec3 ToEulerAngle(physx::PxReal* quat);
