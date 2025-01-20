#include "MetroidPrime/Player/CMorphBall.hpp"

#include <Collision/CRayCastResult.hpp>
#include <Kyoto/Audio/CSfxManager.hpp>
#include <Kyoto/CResFactory.hpp>
#include <Kyoto/Particles/CElementGen.hpp>
#include <Kyoto/Particles/CParticleSwoosh.hpp>
#include <MetroidPrime/CActorLights.hpp>
#include <MetroidPrime/CAnimData.hpp>
#include <MetroidPrime/CControlMapper.hpp>
#include <MetroidPrime/CDamageInfo.hpp>
#include <MetroidPrime/CGameCollision.hpp>
#include <MetroidPrime/CGameLight.hpp>
#include <MetroidPrime/CRainSplashGenerator.hpp>
#include <MetroidPrime/CWorldShadow.hpp>
#include <MetroidPrime/Tweaks/CTweakBall.hpp>
#include <MetroidPrime/Tweaks/CTweakPlayer.hpp>
#include <rstl/math.hpp>

// NON_MATCHING
CMorphBall::CMorphBall(CPlayer& player, float radius)
: x0_player(player)
, x4_loadedModelId(-1)
, x8_ballGlowColorIdx()
, xc_radius(radius)
, x10_boostControlForce(CVector3f::Zero())
, x1c_controlForce(CVector3f::Zero())
, x28_tireMode()
, x2c_tireLeanAngle()
, x30_ballTiltAngle()
, x38_collisionSphere(
      CCollidableSphere(CSphere(CVector3f::Zero(), radius), CMaterialList(kMT_Solid)))
, x5c_ballModelShader()
, x64_spiderBallGlassModelShader()
, x6c_lowPolyBallModelShader()
, xc78_()
, x187c_spiderBallState(kSBS_Inactive)
, x1880_playerToSpiderNormal(CVector3f::Zero())
, x188c_spiderPullMovement(1.0)
, x1890_spiderTrackPoint(CVector3f::Zero())
, x18b4_linVelDamp()
, x18b8_angVelDamp()
, x18bc_spiderNearby()
, x18bd_touchingSpider()
, x18be_spiderBallSwinging()
, x18bf_spiderSwingInAir()
, x18c0_isSpiderSurface()
, x18c4_spiderSurfaceTransform(CTransform4f::Identity())
, x18f4_spiderSurfacePivotAngle()
, x18f8_spiderSurfacePivotTargetAngle()
, x18fc_refPullVel()
, x1900_playerToSpiderTrackDist()
, x1904_swingControlDir()
, x1908_swingControlTime()
, x190c_normSpiderSurfaceForces(0.0f, 0.0f)
, x1914_spiderTrackForceMag()
, x1918_spiderViewControlMag()
, x191c_damageTimer()
, x1920_spiderForcesReset()
, x1924_surfaceToWorld(CTransform4f::Identity())
, x1954_isProjectile()
, x1c0c_wakeEffectIdx()
, x1c10_ballInnerGlowLight(kInvalidUniqueId)
, x1c14_worldShadow(new CWorldShadow(16, 16, false))
, x1c18_actorLights(new CActorLights(8, CVector3f::Zero(), 4, 4))
, x1c1c_rainSplashGen(new CRainSplashGenerator(x58_ballModel->GetScale(), 40, 2, 0.15, 0.5))
, x1c20_tireFactor(0.0f)
, x1c24_maxTireFactor(0.5f)
, x1c28_tireInterpSpeed(1.0f)
, x1c2c_tireInterpolating(false)
, x1c30_boostOverLightFactor()
, x1c34_boostLightFactor()
, x1c38_spiderLightFactor()
, x1dc8_failsafeCounter()
, x1dcc_(CVector3f::Zero())
, x1dd8_(CVector3f::Zero())
, x1de8_boostChargeTime()
, x1dec_timeNotInBoost()
, x1df0_()
, x1df4_boostDrainTime()
, x1dfc_touchHalfPipeCooldown()
, x1e00_disableControlCooldown()
, x1e04_touchHalfPipeRecentCooldown()
, x1e08_prevHalfPipeNormal(CVector3f::Zero())
, x1e14_halfPipeNormal(CVector3f::Zero())
, x1e20_ballAnimIdx()
, x1e34_rollSfx()
, x1e36_landSfx()
, x1e38_wallSparkFrameCountdown()
, x1e44_damageEffect()
, x1e48_damageEffectDecaySpeed()
, x1e4c_damageTime() {
  x19d4_spiderBallMagnetEffectGen->SetParticleEmission(false);
  x19d4_spiderBallMagnetEffectGen->Update(1.0f / 60.0f);

  kSpiderBallCollisionRadius = GetBallRadius() + 0.2f;

  for (int i = 0; i < 32; i++) {
    CToken token(x19a0_spiderElectric);
    rstl::single_ptr< CParticleSwoosh > swoosh(new CParticleSwoosh(token));
    x19e4_spiderElectricGens.push_back(
        rstl::pair< rstl::single_ptr< CParticleSwoosh >, bool >(swoosh, false));
  }

  // LoadAnimationTokens();
  // InitializeWakeEffects();
}

CMorphBall::~CMorphBall() {}

void CMorphBall::Update(float dt, CStateManager& mgr) {
  if (GetSpiderBallState() == kSBS_Active) {
    CVector3f position = GetBallToWorld().GetTranslation();
    CreateSpiderBallParticles(position, x1890_spiderTrackPoint);
  }

  if (x0_player.GetDeathTime() <= 0.0f) {
    UpdateEffects(dt, mgr);
  }

  if (x1e44_damageEffect > 0.0f) {
    x1e44_damageEffect -= x1e48_damageEffectDecaySpeed * dt;

    if (x1e44_damageEffect <= 0.0f) {
      x1e44_damageEffect = 0.0f;
      x1e48_damageEffectDecaySpeed = 0.0f;
      x1e4c_damageTime = 0.0f;
    } else {
      x1e4c_damageTime += dt;
    }
  }

  if (!x58_ballModel.null()) {
    x58_ballModel->AdvanceAnimation(dt, mgr, kInvalidAreaId, true);
  }

  if (x1c2c_tireInterpolating) {
    x1c20_tireFactor += x1c28_tireInterpSpeed * dt;
    if (x1c20_tireFactor < 0.0f) {
      x1c2c_tireInterpolating = false;
      x1c20_tireFactor = 0.0f;
    } else {
      if (x1c20_tireFactor > x1c24_maxTireFactor) {
        x1c2c_tireInterpolating = false;
        x1c20_tireFactor = x1c24_maxTireFactor;
      }
    }
  }

  if (!x1c1c_rainSplashGen.null()) {
    x1c1c_rainSplashGen->Update(dt, mgr);
  }

  UpdateMorphBallSound(dt);
}

CTransform4f CMorphBall::GetBallToWorld() const {
  return CTransform4f(
      CTransform4f::Translate(x0_player.GetTranslation() + CVector3f(0.0f, 0.0f, GetBallRadius())) *
      x0_player.GetTransform().GetRotation());
}

float CMorphBall::GetBallRadius() const { return gpTweakPlayer->GetPlayerBallHalfExtent(); }

void CMorphBall::TakeDamage(float damage) {
  if (damage <= 0.0f) {
    x1e44_damageEffect = 0.0f;
    x1e48_damageEffectDecaySpeed = 0.0f;
    return;
  }

  if (damage >= 20.0f) {
    x1e48_damageEffectDecaySpeed = 0.25f;
  } else {
    if (damage > 5.0f)
      x1e48_damageEffectDecaySpeed = -((damage - 5.0f) / 15.0f * 0.75f - 1.0f);
    else
      x1e48_damageEffectDecaySpeed = 1.0f;
  }

  x1e44_damageEffect = 1.0f;
}

void CMorphBall::LeaveBoosting() {
  if (IsBoosting()) {
    x1dec_timeNotInBoost = 0.0f;
    x1de8_boostChargeTime = 0.0f;
  }

  x1de4_24_inBoost = false;
  x1df4_boostDrainTime = 0.0f;
}

// NON_MATCHING: Nearly matching, one instruction off.
void CMorphBall::CancelBoosting() {
  x1de8_boostChargeTime = 0.0f;
  x1df4_boostDrainTime = 0.0f;

  if (x1e20_ballAnimIdx == 1) { // needs to be cmpwi instruction, but is actually cmplwi.
    x58_ballModel->AnimationData()->SetAnimation(CAnimPlaybackParms(0, -1, 1.0f, true), false);
    x1e20_ballAnimIdx = 0;
    CSfxManager::SfxStop(x1e24_boostSfxHandle);
  }
}

void CMorphBall::SetAsProjectile() { x1954_isProjectile = true; }

// NON_MATCHING
bool CMorphBall::IsInFrustum(const CFrustumPlanes& frustum) const {
  if (x58_ballModel->IsNull())
    return false;

  rstl::optional_object< CAABox > bounds = x19b8_slowBlueTailSwooshGen->GetBounds();

  if (x58_ballModel->IsInFrustum(GetBallToWorld(), frustum))
    return true;

  if (x19b8_slowBlueTailSwooshGen->GetModulationColor().GetAlpha() != 0.0f && bounds.valid() &&
      frustum.BoxFrustumPlanesCheck(bounds.data()) != 0)
    return true;

  return false;
}

float CMorphBall::GetBallTouchRadius() const { return gpTweakBall->GetBallTouchRadius(); }

// NON_MATCHING
void CMorphBall::Touch(CActor& actor, CStateManager& mgr) {
  CPhysicsActor* act = TCastToPtr< CPhysicsActor >(actor);
  if (act->GetCurrentAreaId() != 0 && IsBoosting()) {
    CVector3f relative_velocity = act->GetVelocityWR() - x0_player.GetVelocityWR();
    float relative_speed = relative_velocity.Magnitude();
    if (relative_speed > gpTweakBall->GetBoostBallMinRelativeSpeedForDamage()) {
      static CDamageInfo damage_info(CWeaponMode::BoostBall(), 50000.0f, 50000.0f, 0.0f, 0.0f,
                                     true);
      CMaterialFilter material_filter =
          CMaterialFilter::MakeIncludeExclude(CMaterialList(kMT_Player), CMaterialList());
      mgr.ApplyDamage(x0_player.GetUniqueId(), actor.GetUniqueId(), x0_player.GetUniqueId(),
                      damage_info, material_filter, CVector3f::Zero());
    }
  }
}

// NON_MATCHING
void CMorphBall::AcceptScriptMsg(EScriptObjectMessage msg, TUniqueId id, CStateManager& mgr) {
  switch (msg) {
  case kSM_Registered: {
    if (!x19d0_ballInnerGlowGen.null() && x19d0_ballInnerGlowGen->SystemHasLight()) {
      x1c10_ballInnerGlowLight = mgr.AllocateUniqueId();
      const uint inner_glow_id = x1988_ballInnerGlow.GetTag().GetId();
      mgr.AddObject(rs_new CGameLight(x1c10_ballInnerGlowLight, kInvalidAreaId, false,
                                      rstl::string_l("BallLight"), GetBallToWorld(),
                                      x0_player.GetUniqueId(), x19d0_ballInnerGlowGen->GetLight(),
                                      inner_glow_id, 0, 0.0f));
    }
  } break;

  case kSM_Deleted:
    DeleteLight(mgr);
    break;

  default:
    break;
  }
}

void CMorphBall::DeleteLight(CStateManager& mgr) {
  if (x1c10_ballInnerGlowLight != kInvalidUniqueId) {
    mgr.DeleteObjectRequest(x1c10_ballInnerGlowLight);
    x1c10_ballInnerGlowLight = kInvalidUniqueId;
  }
}

void CMorphBall::EnterMorphBallState(CStateManager& mgr) {
  x1c20_tireFactor = 0.0f;
  UpdateEffects(0.0f, mgr);
  x187c_spiderBallState = kSBS_Inactive;
  x58_ballModel->AnimationData()->SetAnimation(CAnimPlaybackParms(0, -1, 1.0f, true), false);
  x1e20_ballAnimIdx = 0;
  StopParticleWakes();
  x1c30_boostOverLightFactor = 0.0f;
  x1c34_boostLightFactor = 0.0f;
  x1c38_spiderLightFactor = 0.0f;
  DisableHalfPipeStatus();
  x30_ballTiltAngle = 0.0f;
  x2c_tireLeanAngle = 0.0f;
}

void CMorphBall::LeaveMorphBallState(CStateManager& mgr) {
  LeaveBoosting();
  CancelBoosting();
  CSfxManager::SfxStop(x1e24_boostSfxHandle);
  StopParticleWakes();
}

bool CMorphBall::GetIsInHalfPipeMode() const { return x1df8_24_inHalfPipeMode; }

void CMorphBall::SetIsInHalfPipeMode(bool val) { x1df8_24_inHalfPipeMode = val; }

bool CMorphBall::GetIsInHalfPipeModeInAir() const { return x1df8_25_inHalfPipeModeInAir; }

void CMorphBall::SetIsInHalfPipeModeInAir(bool val) { x1df8_25_inHalfPipeModeInAir = val; }

bool CMorphBall::GetTouchedHalfPipeRecently() const { return x1df8_26_touchedHalfPipeRecently; }

void CMorphBall::SetTouchedHalfPipeRecently(bool val) { x1df8_26_touchedHalfPipeRecently = val; }

// NON_MATCHING
CModelData* CMorphBall::GetMorphBallModel(const rstl::string& name, float radius) {
  const SObjectTag* rid = gpResourceFactory->GetResourceIdByName(name.data());

  if (name.data()[0] == 'C') { // Needs to check if it equals "CMDL".
    // Load static model.
    CStaticRes static_res(rid->mId, CVector3f(radius, radius, radius) * 2.0f);
    return new CModelData(static_res);
  }

  // Load animation.
  CAnimRes anim_res(0, CAnimRes::kDefaultCharIdx, CVector3f(radius, radius, radius) * 2.0f, 0,
                    false);
  return new CModelData(anim_res);
}

void CMorphBall::ComputeBallMovement(const CFinalInput& input, CStateManager& mgr, float dt) {
  ComputeBoostBallMovement(input, mgr, dt);
  ComputeMarioMovement(input, mgr, dt);
}

// NON_MATCHING
void CMorphBall::ComputeBoostBallMovement(const CFinalInput& input, const CStateManager& mgr,
                                          float dt) {
  if (!IsMovementAllowed() || !mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_BoostBall))
    return;

  if (!IsBoosting()) {
    CancelBoosting();
    LeaveBoosting();
  } else if (!IsBoosting() && !x1de4_25_boostEnabled) {
    if (gpTweakBall->GetBoostBallDrainTime() < x1df4_boostDrainTime) {
      LeaveBoosting();
    }

    if (!GetIsInHalfPipeMode() && !GetIsInHalfPipeModeInAir()) {
      if (x1df4_boostDrainTime / gpTweakBall->GetBoostBallDrainTime() < 0.3) {
        DampLinearAndAngularVelocities(0.5, 0.01);
      }
      LeaveBoosting();
    }
  } else {
    x1dec_timeNotInBoost += dt;
    bool jump_or_boost = ControlMapper::GetDigitalInput(ControlMapper::kC_JumpOrBoost, input);
    if (!jump_or_boost || GetSpiderBallState() == kSBS_Active) {
      if (x1e20_ballAnimIdx == 1) {
        CAnimPlaybackParms playback_parms(0, -1, 1.0, true);
        x58_ballModel->AnimationData()->SetAnimation(playback_parms, false);
        x1e20_ballAnimIdx = 0;
        CSfxManager::RemoveEmitter(x1e24_boostSfxHandle);

        if (gpTweakBall->GetBoostBallMaxChargeTime() <= x1de8_boostChargeTime) {
          CSfxHandle sfx_handle;
          // CSfxManager::AddEmitter(sfx_handle.GetIndex());
        }
      }

      if (x1de8_boostChargeTime < gpTweakBall->GetBoostBallMaxChargeTime()) {
        if (x1de8_boostChargeTime > 0.0f) {
          CancelBoosting();
        }
      } else {
        EBallBoostState ball_boost_state = GetBallBoostState();
        if (ball_boost_state == kBBS_BoostAvailable) {
          if (GetIsInHalfPipeMode() || GetIsInHalfPipeModeInAir()) {
            EnterBoosting(dt);
          } else {
            CAxisAngle angular_impulse;
            angular_impulse.FromVector(-x1924_surfaceToWorld.GetUp() * 10000.0);
            x0_player.ApplyImpulseWR(CVector3f::Zero(), angular_impulse);
            CancelBoosting();
          }
        } else if (ball_boost_state == kBBS_BoostDisabled) {
          CTransform4f ball_to_world = GetBallToWorld();
        }
      }
    } else {
    }
  }

  float fwd = ControlMapper::GetAnalogInput(ControlMapper::kC_Forward, input);
}

// NON_MATCHING
bool CMorphBall::IsMovementAllowed() const {
  if (!gpTweakPlayer->GetMoveDuringFreeLook() &&
      (x0_player.IsInFreeLook() || x0_player.GetFreeLookButtonState()))
    return false;

  if (x0_player.IsMorphBallTransitioning() || x1e00_disableControlCooldown <= 0.0f)
    return false;

  return true;
}

// NON_MATCHING
void CMorphBall::ComputeMarioMovement(const CFinalInput& input, CStateManager& mgr, float dt) {
  x1c_controlForce = CVector3f::Zero();
  x10_boostControlForce = CVector3f::Zero();

  if (!IsMovementAllowed())
    return;

  float analog_input = ControlMapper::GetAnalogInput(ControlMapper::kC_SpiderBall, input);
  x188c_spiderPullMovement = analog_input < 0.5f / 100.0f ? 0.0f : 1.0f;

  if (mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_SpiderBall) &&
      x188c_spiderPullMovement != 0.0f && x191c_damageTimer == 0.0f) {
    if (x187c_spiderBallState != kSBS_Active) {
      x18bd_touchingSpider = false;
      x187c_spiderBallState = kSBS_Active;
      x189c_spiderInterpBetweenPoints = x0_player.GetTransform().GetColumn(kDZ);
      x18a8_spiderBetweenPoints = x189c_spiderInterpBetweenPoints;
    }

    UpdateSpiderBall(input, mgr, dt);

    if (x18bc_spiderNearby) {
      x187c_spiderBallState = kSBS_Inactive;
      ResetSpiderBallForces();
    }
  } else {
    x187c_spiderBallState = kSBS_Inactive;
    ResetSpiderBallForces();
  }

  if (x187c_spiderBallState != kSBS_Active) {
    float fwd_input = ForwardInput(input);
    float turn_input = -BallTurnInput(input);
    float max_speed = ComputeMaxSpeed();
    float speed = x0_player.GetVelocityWR().Magnitude();
    CTransform4f look =
        CTransform4f::LookAt(CVector3f::Zero(), x0_player.GetControlDirFlat(), CVector3f::Up());
    CVector3f rotated = look.TransposeRotate(x0_player.GetVelocityWR());

    float fwd_acc = 0.0f;
    float turn_acc = 0.0f;

    float force_x = rotated.GetX();
    float force_y = rotated.GetY();

    // Process turn input
    if (CMath::AbsF(turn_input) > 0.1f) {
      float turn_speed = turn_input * max_speed;
      float a = turn_speed - force_x;
      float b = CMath::AbsF(a) / max_speed;
      float c = CMath::Clamp(0.0f, b, 1.0f);

      float fwd_sign = CMath::Sign(force_x);
      float turn_sign = CMath::Sign(turn_speed);

      float max_acc =
          fwd_sign == turn_sign || speed < max_speed * 0.8f
              ? gpTweakBall->GetMaxBallTranslationAcceleration(x0_player.GetSurfaceRestraint())
              : gpTweakBall->GetBallForwardBrakingAcceleration(x0_player.GetSurfaceRestraint());

      // turn_acc = (a >= 0.0f ? max_acc : -max_acc) * c;
      turn_acc = CMath::Sign(a) * max_acc * c;
    }

    // Process forward input
    if (CMath::AbsF(fwd_input) > 0.1f) {
      float fwd_speed = fwd_input * max_speed;
      float a = fwd_speed - force_y;
      float b = CMath::AbsF(a) / max_speed;
      float c = CMath::Clamp(0.0f, b, 1.0f);

      float fwd_sign = CMath::Sign(fwd_speed);
      float turn_sign = CMath::Sign(force_y);

      float max_acc =
          fwd_sign == turn_sign || speed <= max_speed * 0.8f
              ? gpTweakBall->GetMaxBallTranslationAcceleration(x0_player.GetSurfaceRestraint())
              : gpTweakBall->GetBallForwardBrakingAcceleration(x0_player.GetSurfaceRestraint());

      // fwd_acc = (a >= 0.0f ? max_acc : -max_acc) * c;
      fwd_acc = CMath::Sign(a) * max_acc * c;
    }

    if (fwd_acc != 0.0f || turn_acc != 0.0f || x1de4_24_inBoost || GetIsInHalfPipeMode()) {
      CVector3f va = look.Rotate(CVector3f(0.0f, turn_acc, 0.0f));
      CVector3f vb = look.Rotate(CVector3f(fwd_acc, 0.0f, 0.0f));
      CVector3f control_force = va + vb;
      x1c_controlForce = control_force;

      if (x1de4_24_inBoost && !GetIsInHalfPipeMode()) {
        CVector3f vc = x1924_surfaceToWorld.TransposeRotate(control_force);
        control_force = x1924_surfaceToWorld.Rotate(CVector3f(vc.GetX(), 0.0f, 0.0f));
      }

      if (GetIsInHalfPipeMode() && control_force.Magnitude() > FLT_EPSILON) {
        if (GetIsInHalfPipeModeInAir() && speed <= 15.0f) {
          CVector3f surface_to_world_z = x1924_surfaceToWorld.GetColumn(kDZ);
          if (CVector3f::Dot(surface_to_world_z, control_force) / control_force.Magnitude() <
              -0.85f) {
            DisableHalfPipeStatus();
            x1e00_disableControlCooldown = 0.2f;
            x0_player.ApplyImpulseWR(x0_player.GetMass() * -7.5f * surface_to_world_z,
                                     CAxisAngle::Identity());
          }
        }

        if (GetIsInHalfPipeMode()) {
          CVector3f surface_to_world_z = x1924_surfaceToWorld.GetColumn(kDZ);
          control_force -= CVector3f::Dot(surface_to_world_z, control_force) * surface_to_world_z;
          CVector3f v = x1924_surfaceToWorld.TransposeRotate(control_force);
          control_force = x1924_surfaceToWorld.Rotate(
              CVector3f(v.GetX() * 0.6f, v.GetY() * (x1de4_24_inBoost ? 0.0f : 1.4f), v.GetZ()));

          if (max_speed > 95.0f) {
            x0_player.SetVelocityWR(x0_player.GetVelocityWR() * 0.99f);
          }
        }
      }

      if (GetTouchedHalfPipeRecently()) {
        float dotted = CVector3f::Dot(x1e08_prevHalfPipeNormal, x1e14_halfPipeNormal);
        if (dotted < 0.99f && dotted > 0.5f) {
          CVector3f crossed =
              CVector3f::Cross(x1e08_prevHalfPipeNormal, x1e14_halfPipeNormal).AsNormalized();
          CVector3f velocity = x0_player.GetVelocityWR();
          velocity -= CVector3f::Dot(crossed, velocity) * crossed * 0.15f;
          x0_player.SetVelocityWR(velocity);
        }
      }

      float scaled_speed = max_speed * 0.75f;
      if (speed >= scaled_speed) {
        CVector3f velocity = x0_player.GetVelocityWR();
        CVector3f v = velocity.AsNormalized();
        float dotted = CVector3f::Dot(control_force, v);
        if (dotted > 0.0f) {
          float x = (speed - scaled_speed) / (max_speed - scaled_speed);
          x = CMath::Clamp(0.0f, x, 1.0f);
          CVector3f v2 = velocity.AsNormalized();
          control_force -= x * dotted * v2;
        }
      }

      x10_boostControlForce = control_force;
      x0_player.ApplyForceWR(control_force, CAxisAngle::Identity());
    }

    ComputeLiftForces(x1c_controlForce, x0_player.GetVelocityWR(), mgr);
  }
}

void CMorphBall::SetSpiderBallSwingingState(bool val) {
  if (x18be_spiderBallSwinging != val) {
    ResetSpiderBallSwingControllerMovementTimer();
    x18bf_spiderSwingInAir = true;
  }

  x18be_spiderBallSwinging = val;
}

void CMorphBall::ResetSpiderBallSwingControllerMovementTimer() {
  x1904_swingControlDir = 0.0f;
  x1908_swingControlTime = 0.0f;
}

float CMorphBall::ForwardInput(const CFinalInput& input) const {
  if (!IsMovementAllowed())
    return 0.0f;
  float forward = ControlMapper::GetAnalogInput(ControlMapper::kC_Forward, input);
  float backward = ControlMapper::GetAnalogInput(ControlMapper::kC_Backward, input);
  float movement = forward - backward;
  return movement;
}

float CMorphBall::BallTurnInput(const CFinalInput& input) const {
  if (!IsMovementAllowed())
    return 0.0f;
  float left = ControlMapper::GetAnalogInput(ControlMapper::kC_TurnLeft, input);
  float right = ControlMapper::GetAnalogInput(ControlMapper::kC_TurnRight, input);
  float movement = left - right;
  return movement;
}

// NON_MATCHING: Some register differences.
float CMorphBall::ComputeMaxSpeed() const {
  return GetIsInHalfPipeMode()
             ? rstl::min_val(95.0f, x0_player.GetVelocityWR().Magnitude() * 1.5f)
             : gpTweakBall->GetBallTranslationMaxSpeed(x0_player.GetSurfaceRestraint());
}

// NON_MATCHING
void CMorphBall::ComputeLiftForces(const CVector3f& control_force, const CVector3f& velocity,
                                   const CStateManager& mgr) {
  float speed = velocity.Magnitude();
  x1cd0_liftSpeedAvg.AddValue(speed);

  x1d10_liftControlForceAvg.AddValue(control_force);
  CVector3f avg_force = *x1d10_liftControlForceAvg.GetAverage();

  float avg_force_mag = avg_force.Magnitude();
  if (avg_force_mag > 12000.0f) {
    float avg_speed = x1cd0_liftSpeedAvg.GetAverage();

    if (avg_speed < 4.0f) {
      CTransform4f transform = x0_player.GetPrimitiveTransform();
      const CCollisionPrimitive* prim = x0_player.GetCollisionPrimitive();
      CAABox aabb = prim->CalculateAABox(transform);
      CVector3f padding = CVector3f(0.1f, 0.1f, 0.05f);
      CVector3f min = aabb.GetMinPoint() - padding;
      CVector3f max = aabb.GetMaxPoint() + padding;
      CAABox padded = CAABox(min, max);
      CCollidableAABox aabox(padded, CMaterialList(kMT_NoStepLogic, kMT_NoStepLogic));

      bool collided = CGameCollision::DetectStaticCollisionBoolean(
          mgr, *prim, transform, CMaterialFilter::GetPassEverything());

      if (collided) {
        float radius = GetBallRadius();
        CVector3f pos = transform.GetTranslation() + CVector3f::Up() * (radius * 1.75f);
        CRayCastResult raycast_result = mgr.RayStaticIntersection(
            pos, avg_force * (1.0f / avg_force_mag), 1.4f, CMaterialFilter());

        if (raycast_result.IsInvalid()) {
          x0_player.ApplyForceWR(CVector3f::Zero(), CAxisAngle::Identity());
          CAxisAngle ang;
          ang.FromVector(-x1924_surfaceToWorld.GetColumn(kDX) * 1000.0f);
          x0_player.ApplyImpulseWR(CVector3f::Zero(), ang);
        }
      }
    }
  }
}

void CMorphBall::SpinToSpeed(float speed, const CVector3f& direction, float t) {
  CVector3f angular_vel = x0_player.GetAngularVelocityWR().GetVector();
  float scale = t * (speed - angular_vel.Magnitude());
  CVector3f torque = direction * scale;
  x0_player.ApplyTorqueWR(torque);
}

float CMorphBall::CalculateSurfaceFriction() const {
  float friction = gpTweakBall->GetBallTranslationFriction(x0_player.GetSurfaceRestraint());

  if (x0_player.GetAttachedActor() != kInvalidUniqueId) {
    friction *= 2.0f;
  }

  // Do something funky with energy drain. This doesn't make much sense, but it matches.
  if (x0_player.GetEnergyDrain().GetEnergyDrainSources().size() > 0) {
    friction *= x0_player.GetEnergyDrain().GetEnergyDrainSources().size() * 1.5f;
  }

  return friction;
}

void CMorphBall::ApplyFriction(float friction) {
  CVector3f vel = x0_player.GetVelocityWR();
  if (friction < vel.Magnitude()) {
    float adjusted_speed = vel.Magnitude() - friction;
    CVector3f direction = vel.AsNormalized();
    vel = adjusted_speed * direction;
  } else {
    vel = CVector3f::Zero();
  }

  x0_player.SetVelocityWR(vel);
}

void CMorphBall::ApplyGravity(CStateManager& mgr) {
  if (x0_player.CheckSubmerged() &&
      !mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_GravitySuit)) {
    x0_player.SetMomentumWR(
        CVector3f(0.0f, 0.0f, x0_player.GetMass() * gpTweakBall->GetBallWaterGravity()));
  } else {
    x0_player.SetMomentumWR(
        CVector3f(0.0f, 0.0f, x0_player.GetMass() * gpTweakBall->GetBallGravity()));
  }
}

void CMorphBall::Land() {
  if (x0_player.GetVelocityWR().GetZ() < -5.0f) {
    if (x1e36_landSfx != 0xFFFF /* Invalid SFX ID */) {
      uchar land_vol = CCast::ToUint8(
          CMath::Clamp(95.0f, x0_player.GetLastVelocity().GetZ() * 1.6f + 95.0f, 127.0f));
      // This same operation is done in CPlayer::AcceptScriptMsg.
      x0_player.DoSfxEffects(CSfxManager::SfxStart(x1e36_landSfx, land_vol, 64, true));
    }
  }
}
