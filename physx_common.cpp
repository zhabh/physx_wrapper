#include "physx_common.h"

physx_common* physx_common::instance_ = nullptr;

physx_common::physx_common() { InitPhyxsSDK(); }

physx_common::~physx_common() { Destory(); }

bool physx_common::InitPhyxsSDK() {
  px_foundation_ = PxCreateFoundation(PX_FOUNDATION_VERSION, px_allocator_,
                                      px_error_callback_);
  if (px_foundation_ == nullptr) {
    LogMgr::GetInstance()->Error(
        "physx_common::InitPhyxsSDK() error:px_foundation_ is null");
    return false;
  }

#ifdef _DEBUG
  px_pvd_ = physx::PxCreatePvd(*px_foundation_);
  if (px_pvd_ == nullptr) return false;

  physx::PxPvdTransport* transport =
      physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
  if (transport == nullptr) return false;

  bool is_connected =
      px_pvd_->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
  if (!is_connected) {
    LogMgr::GetInstance()->Error(
        "physx_common::InitPhyxsSDK() error:pvd connect error.");
  }
#endif  // _DEBUG

  px_physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *px_foundation_,
                                physx::PxTolerancesScale(), true, px_pvd_);
  if (px_physics_ == nullptr) {
    LogMgr::GetInstance()->Error(
        "physx_common::InitPhyxsSDK() error: px_physics_ is null");
    return false;
  }

  px_dispatcher_ = physx::PxDefaultCpuDispatcherCreate(2);

  px_material_ = px_physics_->createMaterial(0.5f, 0.5f, 0.6f);

  return true;
}

void physx_common::Destory() {
  if (instance_ != nullptr) {
    ReleaseSDK();
    delete instance_;
    instance_ = nullptr;
  }
}
void physx_common::ReleaseSDK() {
  if (px_physics_ != nullptr) {
    px_dispatcher_->release();
    px_physics_->release();
    if (px_pvd_) {
      physx::PxPvdTransport* transport = px_pvd_->getTransport();
      if (transport) {
        transport->release();
        transport = nullptr;
      }
      px_pvd_->release();
      px_pvd_ = nullptr;
    }

    px_foundation_->release();

    px_dispatcher_ = nullptr;
    px_physics_ = nullptr;
    px_foundation_ = nullptr;
  }
}

/////////////////////////////////////////////////////////////////////////
// maths tools
/////////////////////////////////////////////////////////////////////////
void SetEulerYXZ(const physx::PxReal& yaw, const physx::PxReal& pitch,
                 const physx::PxReal& roll, physx::PxQuat& quat) {
  physx::PxReal halfYaw = physx::PxReal(yaw) * physx::PxReal(0.5);
  physx::PxReal halfPitch = physx::PxReal(pitch) * physx::PxReal(0.5);
  physx::PxReal halfRoll = physx::PxReal(roll) * physx::PxReal(0.5);
  physx::PxReal cosYaw = physx::PxCos(halfYaw);
  physx::PxReal sinYaw = physx::PxSin(halfYaw);
  physx::PxReal cosPitch = physx::PxCos(halfPitch);
  physx::PxReal sinPitch = physx::PxSin(halfPitch);
  physx::PxReal cosRoll = physx::PxCos(halfRoll);
  physx::PxReal sinRoll = physx::PxSin(halfRoll);
  quat =
      physx::PxQuat(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                    cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
                    sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                    cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
}

physx::PxVec3 GetQuatRotate(KVector3D& dir, KVector3D& velocity) {
  physx::PxQuat px_qua;
  SetEulerYXZ((PI_F * dir.nY) / (1000 * 180), (PI_F * dir.nX) / (1000 * 180),
              (PI_F * dir.nZ) / (1000 * 180), px_qua);

  return px_qua.rotate(physx::PxVec3((float)velocity.nX, (float)velocity.nY,
                                     (float)velocity.nZ));
}

physx::PxVec3 QuatToEuler(const physx::PxQuat& q) {
  physx::PxVec3 dir = -physx::PxMat33(q)[2];
  physx::PxReal r = physx::PxSqrt(dir.x * dir.x + dir.z * dir.z);
  physx::PxVec3 rot(0.0f, physx::PxHalfPi, 0.0f);
  if (r != 0.0f) {
    rot.x = -physx::PxAtan(dir.y / r);
    rot.y = physx::PxAsin(dir.x / r);
    if (dir.z > 0.0f) rot.y = physx::PxPi - rot.y;
  }

  return rot;
}

void QuatToMatrix(const physx::PxReal* quat, physx::PxReal* matrix) {
  physx::PxReal xx = quat[0] * quat[0];
  physx::PxReal yy = quat[1] * quat[1];
  physx::PxReal zz = quat[2] * quat[2];
  physx::PxReal xy = quat[0] * quat[1];
  physx::PxReal xz = quat[0] * quat[2];
  physx::PxReal yz = quat[1] * quat[2];
  physx::PxReal wx = quat[3] * quat[0];
  physx::PxReal wy = quat[3] * quat[1];
  physx::PxReal wz = quat[3] * quat[2];

  matrix[0 * 4 + 0] = 1 - 2 * (yy + zz);
  matrix[1 * 4 + 0] = 2 * (xy - wz);
  matrix[2 * 4 + 0] = 2 * (xz + wy);

  matrix[0 * 4 + 1] = 2 * (xy + wz);
  matrix[1 * 4 + 1] = 1 - 2 * (xx + zz);
  matrix[2 * 4 + 1] = 2 * (yz - wx);

  matrix[0 * 4 + 2] = 2 * (xz - wy);
  matrix[1 * 4 + 2] = 2 * (yz + wx);
  matrix[2 * 4 + 2] = 1 - 2 * (xx + yy);

  matrix[3 * 4 + 0] = matrix[3 * 4 + 1] = matrix[3 * 4 + 2] =
      (physx::PxReal)0.0f;
  matrix[0 * 4 + 3] = matrix[1 * 4 + 3] = matrix[2 * 4 + 3] =
      (physx::PxReal)0.0f;
  matrix[3 * 4 + 3] = (physx::PxReal)1.0f;
}

physx::PxVec3 ToEulerAngle(physx::PxReal* quat) {
  physx::PxReal matrix[16];

  QuatToMatrix(quat, matrix);

  auto m11 = matrix[0], m12 = matrix[4], m13 = matrix[8];

  auto m21 = matrix[1], m22 = matrix[5], m23 = matrix[9];

  auto m31 = matrix[2], m32 = matrix[6], m33 = matrix[10];

  if (m23 < -1) {
    m23 = -1;
  }

  if (m23 > 1) {
    m23 = 1;
  }

  physx::PxReal _x, _y, _z;
  _x = asin(-m23);

  if (fabs(m23) < 0.99999) {
    _y = atan2(m13, m33);

    _z = atan2(m21, m22);

  } else {
    _y = atan2(-m31, m11);

    _z = 0;
  }

  return physx::PxVec3(_x, _y, _z);
}
