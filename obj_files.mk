INIT_O_FILES :=\
	$(BUILD_DIR)/src/os/__start.o\
	$(BUILD_DIR)/src/os/__ppc_eabi_init.o\
	$(BUILD_DIR)/asm/Runtime/__mem.o

METROTRK_FILES :=\
	$(BUILD_DIR)/src/MetroTRK/mslsupp.o
	
METROIDPRIME :=\
	$(BUILD_DIR)/asm/MetroidPrime/main.o\
	$(BUILD_DIR)/asm/MetroidPrime/text_80008894_80009144.o\
	$(BUILD_DIR)/asm/MetroidPrime/IRenderer.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CCameraManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/CControlMapper.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CFirstPersonCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/CObjectList.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayer.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAxisAngle.o\
	$(BUILD_DIR)/asm/MetroidPrime/CEulerAngles.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFrontEndUI.o\
	$(BUILD_DIR)/asm/MetroidPrime/CInputGenerator.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMainFlow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMFGame.o\
	$(BUILD_DIR)/asm/MetroidPrime/CCredits.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSplashScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/CInstruction.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAnimData.o\
	$(BUILD_DIR)/asm/MetroidPrime/Factories/CCharacterFactory.o\
	$(BUILD_DIR)/asm/MetroidPrime/Factories/CAssetFactory.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakPlayer.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweaks.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakGame.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGameProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerGun.o\
	$(BUILD_DIR)/asm/MetroidPrime/CStateManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/CEntity.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArchMsgParmInt32.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArchMsgParmInt32Int32VoidPtr.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArchMsgParmNull.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArchMsgParmReal32.o\
	$(BUILD_DIR)/asm/MetroidPrime/Decode.o\
	$(BUILD_DIR)/asm/MetroidPrime/CIOWinManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/CIOWin.o\
	$(BUILD_DIR)/asm/MetroidPrime/CActor.o\
	$(BUILD_DIR)/asm/MetroidPrime/CWorld.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakParticle.o\
	$(BUILD_DIR)/asm/MetroidPrime/Clamp_int.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArchMsgParmControllerStatus.o\
	$(BUILD_DIR)/asm/MetroidPrime/CExplosion.o\
	$(BUILD_DIR)/asm/MetroidPrime/CEffect.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CGameCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGameArea.o\
	$(BUILD_DIR)/asm/MetroidPrime/HUD/CSamusHud.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAnimationDatabaseGame.o\
	$(BUILD_DIR)/asm/MetroidPrime/CTransitionDatabaseGame.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakPlayerControl.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakPlayerGun.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPauseScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakGui.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptActor.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptTrigger.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptWaypoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPatterned.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDoor.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CStateMachine.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMapArea.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CBallCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptEffect.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CBomb.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakBall.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerState.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptTimer.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CCinematicCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAutoMapper.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCounter.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMapWorld.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CAi.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/PatternedCastTo.o\
	$(BUILD_DIR)/asm/MetroidPrime/TCastTo.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSound.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPlatform.o\
	$(BUILD_DIR)/src/MetroidPrime/UserNames.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptGenerator.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraWaypoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGameLight.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakTargeting.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakAutoMapper.o\
	$(BUILD_DIR)/asm/MetroidPrime/CParticleGenInfoGeneric.o\
	$(BUILD_DIR)/asm/MetroidPrime/CParticleGenInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/CParticleDatabase.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakGunRes.o\
	$(BUILD_DIR)/asm/MetroidPrime/CTargetReticles.o\
	$(BUILD_DIR)/asm/MetroidPrime/CWeaponMgr.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPickup.o\
	$(BUILD_DIR)/asm/MetroidPrime/CDamageInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMemoryDrawEnum.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDock.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraHint.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptLoader.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSamusDoll.o\
	$(BUILD_DIR)/asm/MetroidPrime/Factories/CStateMachineFactory.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CPlasmaBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CPowerBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CWaveBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CIceBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/CScriptMailbox.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptRelay.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSpawnPoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptRandomRelay.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBeetle.o\
	$(BUILD_DIR)/src/MetroidPrime/HUD/CHUDMemoParms.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptHUDMemo.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMappableObject.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerCameraBob.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraFilterKeyframe.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraBlurKeyframe.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CCameraFilter.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CMorphBall.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDamageableTrigger.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDebris.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraShaker.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptActorKeyframe.o\
	$(BUILD_DIR)/asm/MetroidPrime/CConsoleOutputWindow.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptWater.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CWeapon.o\
	$(BUILD_DIR)/asm/MetroidPrime/CDamageVulnerability.o\
	$(BUILD_DIR)/asm/MetroidPrime/CActorLights.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPatternedInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSimpleShadow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CActorParameters.o\
	$(BUILD_DIR)/asm/MetroidPrime/CInGameGuiManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CWarWasp.o\
	$(BUILD_DIR)/asm/MetroidPrime/CWorldShadow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAudioStateWin.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerVisor.o\
	$(BUILD_DIR)/asm/MetroidPrime/CModelData.o\
	$(BUILD_DIR)/asm/MetroidPrime/CDecalManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSpiderBallWaypoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBloodFlower.o\
	$(BUILD_DIR)/asm/MetroidPrime/TGameTypes.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPhysicsActor.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPhysicsState.o\
	$(BUILD_DIR)/asm/MetroidPrime/CRipple.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidUVMotion.o\
	$(BUILD_DIR)/asm/MetroidPrime/CRippleManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CGrappleArm.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CSpacePirate.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCoverPoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CPathCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidPlane.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidPlaneManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptGrapplePoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CHUDBillboardEffect.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFlickerBat.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBodyStateCmdMgr.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBodyStateInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSAttack.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSDie.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSFall.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSGetup.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSKnockBack.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSLieOnGround.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSLocomotion.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSStep.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSTurn.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBodyController.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSLoopAttack.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CTargetableProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSLoopReaction.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSteeringBehaviors.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSGroundHit.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CChozoGhost.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFireFlea.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSSlide.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSHurled.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSJump.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSGenerate.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPuddleSpore.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSTaunt.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSortedLists.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDebugCameraWaypoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSpiderBallAttractionSurface.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSScripted.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPuddleToadGamma.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDistanceFog.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSProjectileAttack.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CPowerBomb.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetaree.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptDockAreaChange.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSpecialFunction.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptActorRotate.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CFidget.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CSpankWeed.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CParasite.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CSamusFaceReflection.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPlayerHint.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CRipper.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CCameraShakeData.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPickupGenerator.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPointOfInterest.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CDrone.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMapWorldInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/Factories/CScannableObjectInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroid.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CScanDisplay.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSteam.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptRipple.o\
	$(BUILD_DIR)/asm/MetroidPrime/CBoneTracking.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CFaceplateDecoration.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSCover.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptBallTrigger.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CPlasmaProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerOrbit.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGameCollision.o\
	$(BUILD_DIR)/asm/MetroidPrime/CBallFilter.o\
	$(BUILD_DIR)/asm/MetroidPrime/CAABoxFilter.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGroundMovement.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CNewIntroBoss.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CPhazonBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptTargetingPoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CBSWallHang.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptEMPulse.o\
	$(BUILD_DIR)/asm/MetroidPrime/HUD/CHudDecoInterface.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CFlameThrower.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CBeamProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidPlaneCPU.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidPlaneDoor.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptRoomAcoustics.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CIceSheegoth.o\
	$(BUILD_DIR)/asm/MetroidPrime/CCollisionActorManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/CCollisionActor.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPlayerActor.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakPlayerRes.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBurstFire.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFlaahgra.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerEnergyDrain.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFlameWarp.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CIceImpact.o\
	$(BUILD_DIR)/asm/MetroidPrime/GameObjectLists.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CAuxWeapon.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CGunWeapon.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptAreaAttributes.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CWaveBuster.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CStaticInterference.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroidBeta.o\
	$(BUILD_DIR)/asm/MetroidPrime/PathFinding/CPathFindSearch.o\
	$(BUILD_DIR)/asm/MetroidPrime/PathFinding/CPathFindRegion.o\
	$(BUILD_DIR)/asm/MetroidPrime/PathFinding/CPathFindArea.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/GunController/CGunController.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/GunController/CGSFreeLook.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/GunController/CGSComboFire.o\
	$(BUILD_DIR)/asm/MetroidPrime/HUD/CHudBallInterface.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakGuiColors.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CFishCloud.o\
	$(BUILD_DIR)/asm/MetroidPrime/CHealthInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CGameState.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptVisorFlare.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptWorldTeleporter.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptVisorGoo.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CJellyZap.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptControllerAction.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/GunController/CGunMotion.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSwitch.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CABSIdle.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CABSFlinch.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CABSAim.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptPlayerStateChange.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CThardus.o\
	$(BUILD_DIR)/asm/MetroidPrime/CActorParticles.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CWallCrawlerSwarm.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptAiJumpPoint.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMessageScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFlaahgraTentacle.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/GunController/CGSFidget.o\
	$(BUILD_DIR)/asm/MetroidPrime/BodyState/CABSReaction.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CIceProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFlyingPirate.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptColorModulate.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMapUniverse.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CThardusRockProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/CInventoryScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/CVisorFlare.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CFlaahgraPlants.o\
	$(BUILD_DIR)/asm/MetroidPrime/CWorldTransManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptMidi.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptStreamedAudio.o\
	$(BUILD_DIR)/asm/MetroidPrime/CRagDoll.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CGameOptions.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CRepulsor.o\
	$(BUILD_DIR)/asm/MetroidPrime/CEnvFxManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CEnergyProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptGunTurret.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CProjectileInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/CInGameTweakManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBabygoth.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CEyeBall.o\
	$(BUILD_DIR)/asm/MetroidPrime/CIkChain.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraPitchVolume.o\
	$(BUILD_DIR)/asm/MetroidPrime/RumbleFxTable.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CElitePirate.o\
	$(BUILD_DIR)/asm/MetroidPrime/CRumbleManager.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBouncyGrenade.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CGrenadeLauncher.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CShockWave.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CRipperControlledPlatform.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CKnockBackController.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CWorldLayerState.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMagdolite.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CTeamAiMgr.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CSnakeWeedSwarm.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CBallCameraFailsafeState.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CActorContraption.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptSpindleCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptMemoryRelay.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPauseScreenFrame.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CAtomicAlpha.o\
	$(BUILD_DIR)/asm/MetroidPrime/CLogBookScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGBASupport.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CSaveWorld.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptCameraHintTrigger.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CAmbientAI.o\
	$(BUILD_DIR)/asm/MetroidPrime/CMemoryCardDriver.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSaveGameScreen.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CAtomicBeta.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CElectricBeamProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CRidley.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPuffer.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CFire.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPauseScreenBlur.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CTryclops.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/CNewFlameThrower.o\
	$(BUILD_DIR)/asm/MetroidPrime/Cameras/CInterpolationCamera.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CSeedling.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CGameHintInfo.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CWallWalker.o\
	$(BUILD_DIR)/asm/MetroidPrime/CErrorOutputWindow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CRainSplashGenerator.o\
	$(BUILD_DIR)/asm/MetroidPrime/Factories/CSaveWorldFactory.o\
	$(BUILD_DIR)/asm/MetroidPrime/CFluidPlaneRender.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CBurrower.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroidPrimeExo.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptBeam.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroidPrimeEssence.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroidPrimeRelay.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerDynamics.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptMazeNode.o\
	$(BUILD_DIR)/asm/MetroidPrime/Weapons/WeaponTypes.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/COmegaPirate.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPhazonPool.o\
	$(BUILD_DIR)/asm/MetroidPrime/CNESEmulator.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CPhazonHealingNodule.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CMorphBallShadow.o\
	$(BUILD_DIR)/asm/MetroidPrime/Player/CPlayerInputFilter.o\
	$(BUILD_DIR)/asm/MetroidPrime/CSlideShow.o\
	$(BUILD_DIR)/asm/MetroidPrime/Tweaks/CTweakSlideShow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CArtifactDoll.o\
	$(BUILD_DIR)/asm/MetroidPrime/CProjectedShadow.o\
	$(BUILD_DIR)/asm/MetroidPrime/CPreFrontEnd.o\
	$(BUILD_DIR)/asm/MetroidPrime/CGameCubeDoll.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CScriptProjectedShadow.o\
	$(BUILD_DIR)/asm/MetroidPrime/ScriptObjects/CEnergyBall.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/CMetroidPrimeProjectile.o\
	$(BUILD_DIR)/asm/MetroidPrime/Enemies/SPositionHistory.o\

WORLDFORMAT :=\
	$(BUILD_DIR)/asm/WorldFormat/CAreaOctTree_Tests.o\
	$(BUILD_DIR)/asm/WorldFormat/CCollisionSurface.o\
	$(BUILD_DIR)/asm/WorldFormat/CMetroidModelInstance.o\
	$(BUILD_DIR)/asm/WorldFormat/CAreaBspTree.o\
	$(BUILD_DIR)/asm/WorldFormat/CAreaOctTree.o\
	$(BUILD_DIR)/asm/WorldFormat/CMetroidAreaCollider.o\
	$(BUILD_DIR)/asm/WorldFormat/CWorldLight.o\
	$(BUILD_DIR)/asm/WorldFormat/COBBTree.o\
	$(BUILD_DIR)/asm/WorldFormat/CCollidableOBBTree.o\
	$(BUILD_DIR)/asm/WorldFormat/CCollidableOBBTreeGroup.o\
	$(BUILD_DIR)/asm/WorldFormat/CPVSVisAreaSet.o\
	$(BUILD_DIR)/asm/WorldFormat/CAreaRenderOctTree.o\
	
WEAPONS :=\
	$(BUILD_DIR)/asm/Weapons/CProjectileWeapon.o\
	$(BUILD_DIR)/asm/Weapons/CProjectileWeaponDataFactory.o\
	$(BUILD_DIR)/asm/Weapons/CCollisionResponseData.o\
	$(BUILD_DIR)/asm/Weapons/IWeaponRenderer.o\
	$(BUILD_DIR)/asm/Weapons/CDecalDataFactory.o\
	$(BUILD_DIR)/asm/Weapons/CDecal.o\
	$(BUILD_DIR)/asm/Weapons/CWeaponDescription.o\
	$(BUILD_DIR)/asm/Weapons/CDecalDescription.o\
	
METARENDER :=\
	$(BUILD_DIR)/asm/MetaRender/CCubeRenderer.o\
	
GUISYS :=\
	$(BUILD_DIR)/asm/GuiSys/CAuiMain.o\
	$(BUILD_DIR)/asm/GuiSys/CAuiMeter.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiGroup.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiHeadWidget.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiLight.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiModel.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiObject.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiPane.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiSliderGroup.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiSys.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiTableGroup.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiTextPane.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiTextSupport.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiWidget.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiWidgetIdDB.o\
	$(BUILD_DIR)/asm/GuiSys/CGuiWidgetDrawParms.o\
	$(BUILD_DIR)/asm/GuiSys/CAuiEnergyBarT01.o\
	$(BUILD_DIR)/asm/GuiSys/CAuiImagePane.o\
	$(BUILD_DIR)/asm/GuiSys/CRepeatState.o\

COLLISION :=\
	$(BUILD_DIR)/asm/Collision/CCollidableAABox.o\
	$(BUILD_DIR)/asm/Collision/CCollidableCollisionSurface.o\
	$(BUILD_DIR)/asm/Collision/CCollisionInfo.o\
	$(BUILD_DIR)/asm/Collision/InternalColliders.o\
	$(BUILD_DIR)/asm/Collision/CCollisionPrimitive.o\
	$(BUILD_DIR)/asm/Collision/CMaterialList.o\
	$(BUILD_DIR)/asm/Collision/CollisionUtil.o\
	$(BUILD_DIR)/asm/Collision/CCollidableSphere.o\
	$(BUILD_DIR)/asm/Collision/CMaterialFilter.o\
	$(BUILD_DIR)/asm/Collision/COBBox.o\
	$(BUILD_DIR)/asm/Collision/CMRay.o\

KYOTO :=\
	$(BUILD_DIR)/asm/Kyoto/Basics/CBasics.o\
	$(BUILD_DIR)/asm/Kyoto/Basics/CStopwatch.o\
	$(BUILD_DIR)/asm/Kyoto/Basics/CBasicsDolphin.o\
	$(BUILD_DIR)/asm/Kyoto/Basics/CCallStackDolphin.o\
	$(BUILD_DIR)/asm/Kyoto/Basics/COsContextDolphin.o\
	$(BUILD_DIR)/src/Kyoto/Basics/CSWDataDolphin.o\
	$(BUILD_DIR)/asm/Kyoto/Basics/RAssertDolphin.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimation.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimationManager.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimationSet.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimCharacterSet.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeLoopIn.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeSequence.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CCharacterInfo.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CCharacterSet.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimBlend.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimFactory.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimPhaseBlend.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimPlay.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimRandom.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaAnimSequence.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaTransFactory.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaTransMetaAnim.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaTransPhaseTrans.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaTransSnap.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CMetaTransTrans.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPASAnimInfo.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPASAnimParm.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPASAnimState.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPASDatabase.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPASParmInfo.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPrimitive.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CSequenceHelper.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CTransition.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CTransitionManager.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CTreeUtils.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/IMetaAnim.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CSfxHandle.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CSfxManager.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAdvancementDeltas.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimMathUtils.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimPOIData.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimSource.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimSourceReader.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimSourceReaderBase.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeAnimReaderContainer.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeBlend.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeContinuousPhaseBlend.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeDoubleChild.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeNode.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeSingleChild.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeTimeScale.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeTransition.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAnimTreeTweenBase.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CBoolPOINode.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CCharAnimMemoryMetrics.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CCharLayoutInfo.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CFBStreamedAnimReader.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CFBStreamedCompression.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CHierarchyPoseBuilder.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CInt32POINode.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CParticlePOINode.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPOINode.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CSegStatementSet.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CTimeScaleFunctions.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/IAnimReader.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAllFormatsAnimSource.o\
	$(BUILD_DIR)/asm/Kyoto/CDvdRequest.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CColorInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CColorOverrideInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CDrawStringOptions.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CFontInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CFontRenderState.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CLineExtraSpaceInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CLineInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CLineSpacingInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CPopStateInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CPushStateInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CRasterFont.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CRemoveColorOverrideInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CSavableState.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CTextExecuteBuffer.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CTextInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CTextParser.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CWordBreakTables.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CWordInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CBlockInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CFont.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CLight.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CCubeModel.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CGX.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CTevCombiners.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/DolphinCGraphics.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/DolphinCPalette.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/DolphinCTexture.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CloseEnough.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CMatrix3f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CMatrix4f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CQuaternion.o\
	$(BUILD_DIR)/src/Kyoto/CRandom16.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CTransform4f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CUnitVector3f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CVector2f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CVector2i.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CVector3d.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CVector3f.o\
	$(BUILD_DIR)/asm/Kyoto/Math/RMathUtils.o\
	$(BUILD_DIR)/src/Kyoto/CCrc32.o\
	$(BUILD_DIR)/asm/Kyoto/Alloc/CCircularBuffer.o\
	$(BUILD_DIR)/asm/Kyoto/Alloc/CMemory.o\
	$(BUILD_DIR)/src/Kyoto/Alloc/IAllocator.o\
	$(BUILD_DIR)/asm/Kyoto/PVS/CPVSVisOctree.o\
	$(BUILD_DIR)/asm/Kyoto/PVS/CPVSVisSet.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CColorElement.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CElementGen.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleGen.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleGlobals.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleSwoosh.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleSwooshDataFactory.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CRealElement.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CSpawnSystemKeyframeData.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CUVElement.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CVectorElement.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CWarp.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CPlane.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CSphere.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CAABox.o\
	$(BUILD_DIR)/asm/Kyoto/CFactoryMgr.o\
	$(BUILD_DIR)/asm/Kyoto/CResFactory.o\
	$(BUILD_DIR)/asm/Kyoto/CResLoader.o\
	$(BUILD_DIR)/asm/Kyoto/rstl/rstl_map.o\
	$(BUILD_DIR)/asm/Kyoto/rstl/rstl_strings.o\
	$(BUILD_DIR)/asm/Kyoto/rstl/RstlExtras.o\
	$(BUILD_DIR)/src/Kyoto/Streams/CInputStream.o\
	$(BUILD_DIR)/src/Kyoto/Streams/CMemoryInStream.o\
	$(BUILD_DIR)/asm/Kyoto/Streams/CMemoryStreamOut.o\
	$(BUILD_DIR)/asm/Kyoto/Streams/COutputStream.o\
	$(BUILD_DIR)/asm/Kyoto/Streams/CZipInputStream.o\
	$(BUILD_DIR)/asm/Kyoto/Streams/CZipSupport.o\
	$(BUILD_DIR)/asm/Kyoto/CSimplePool.o\
	$(BUILD_DIR)/asm/Kyoto/CToken.o\
	$(BUILD_DIR)/asm/Kyoto/IObj.o\
	$(BUILD_DIR)/src/Kyoto/zlib/adler32.o\
	$(BUILD_DIR)/src/Kyoto/zlib/infblock.o\
	$(BUILD_DIR)/src/Kyoto/zlib/infcodes.o\
	$(BUILD_DIR)/src/Kyoto/zlib/inffast.o\
	$(BUILD_DIR)/src/Kyoto/zlib/inflate.o\
	$(BUILD_DIR)/src/Kyoto/zlib/inftrees.o\
	$(BUILD_DIR)/src/Kyoto/zlib/infutil.o\
	$(BUILD_DIR)/src/Kyoto/zlib/zutil.o\
	$(BUILD_DIR)/asm/Kyoto/CARAMManager.o\
	$(BUILD_DIR)/asm/Kyoto/Math/CFrustumPlanes.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CCubeMaterial.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CCubeSurface.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CCharAnimTime.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CSegIdList.o\
	$(BUILD_DIR)/asm/Kyoto/CFinalInput.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CColor.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/DolphinCAudioGroupSet.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/DolphinCAudioSys.o\
	$(BUILD_DIR)/asm/Kyoto/DolphinCMemoryCardSys.o\
	$(BUILD_DIR)/asm/Kyoto/Input/DolphinIController.o\
	$(BUILD_DIR)/asm/Kyoto/Input/CDolphinController.o\
	$(BUILD_DIR)/asm/Kyoto/DolphinCDvdFile.o\
	$(BUILD_DIR)/asm/Kyoto/Alloc/CMediumAllocPool.o\
	$(BUILD_DIR)/asm/Kyoto/Alloc/CSmallAllocPool.o\
	$(BUILD_DIR)/asm/Kyoto/Alloc/CGameAllocator.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/DolphinCSkinnedModel.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/DolphinCSkinRules.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/DolphinCVirtualBone.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/DolphinCModel.o\
	$(BUILD_DIR)/src/Kyoto/Text/CStringTable.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CEmitterElement.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CEffectComponent.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleData.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CVertexMorphEffect.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CSkinnedModelWithAvgNormals.o\
	$(BUILD_DIR)/asm/Kyoto/CTimeProvider.o\
	$(BUILD_DIR)/asm/Kyoto/CARAMToken.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CMidiManager.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CFontImageDef.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CImageInstruction.o\
	$(BUILD_DIR)/asm/Kyoto/Text/CTextRenderBuffer.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/CCubeMoviePlayer.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CAdditiveAnimPlayback.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleElectricDataFactory.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CParticleElectric.o\
	$(BUILD_DIR)/asm/Kyoto/Graphics/DolphinCColor.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CDSPStreamManager.o\
	$(BUILD_DIR)/asm/Kyoto/CDependencyGroup.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CStreamAudioManager.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CHalfTransition.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CElectricDescription.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CSwooshDescription.o\
	$(BUILD_DIR)/asm/Kyoto/Particles/CGenDescription.o\
	$(BUILD_DIR)/asm/Kyoto/CPakFile.o\
	$(BUILD_DIR)/asm/Kyoto/Animation/CPoseAsTransforms.o\
	$(BUILD_DIR)/asm/Kyoto/Input/CRumbleVoice.o\
	$(BUILD_DIR)/asm/Kyoto/Input/RumbleAdsr.o\
	$(BUILD_DIR)/asm/Kyoto/Input/CRumbleGenerator.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/SDSPStream.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/g721.o\
	$(BUILD_DIR)/asm/Kyoto/Audio/CStaticAudioPlayer.o\
	$(BUILD_DIR)/asm/Kyoto/CFrameDelayedKiller.o\
	
AI_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/ai.o
	
AR_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/ar/ar.o\
	$(BUILD_DIR)/asm/Dolphin/ar/arq.o
	
BASE_FILES :=\
	$(BUILD_DIR)/src/Dolphin/PPCArch.o
	
DB_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/db.o
	
DSP_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/dsp/dsp.o\
	$(BUILD_DIR)/src/Dolphin/dsp/dsp_debug.o\
	$(BUILD_DIR)/asm/Dolphin/dsp/dsp_task.o
	
DVD_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvdlow.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvdfs.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvd.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvdqueue.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvderror.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvdidutils.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/dvdfatal.o\
	$(BUILD_DIR)/asm/Dolphin/dvd/fstload.o
	
GX_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/gx/GXInit.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXFifo.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXAttr.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXMisc.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXGeometry.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXFrameBuf.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXLight.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXTexture.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXBump.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXTev.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXPixel.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXStubs.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXDisplayList.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXTransform.o\
	$(BUILD_DIR)/asm/Dolphin/gx/GXPerf.o
	
MTX_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/mtx/mtx.o\
	$(BUILD_DIR)/asm/Dolphin/mtx/mtx44vec.o\
	$(BUILD_DIR)/asm/Dolphin/mtx/mtx44.o\
	$(BUILD_DIR)/asm/Dolphin/mtx/vec.o\
	$(BUILD_DIR)/asm/Dolphin/mtx/psmtx.o
	
# TODO: Move __ppc_eabi_init_text into the same source files as __ppc_eabi_init
OS_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/os/OS.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSAlarm.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSArena.o\
	$(BUILD_DIR)/src/Dolphin/os/OSAudioSystem.ep.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSCache.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSContext.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSError.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSFatal.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSFont.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSInterrupt.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSLink.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSMemory.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSMutex.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSReboot.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSReset.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSResetSW.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSRtc.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSSync.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSThread.o\
	$(BUILD_DIR)/asm/Dolphin/os/OSTime.o\
	$(BUILD_DIR)/asm/os/__ppc_eabi_init_text.o

PAD_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/pad/PadClamp.o\
	$(BUILD_DIR)/asm/Dolphin/pad/pad.o
	
VI_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/vi.o
	
MSL_PPCEABI_BARE_H :=\
	$(BUILD_DIR)/asm/Runtime/__va_arg.o\
	$(BUILD_DIR)/asm/Runtime/global_destructor_chain.o\
	$(BUILD_DIR)/asm/Runtime/CPlusLibPPC.o\
	$(BUILD_DIR)/asm/Runtime/NMWException.o\
	$(BUILD_DIR)/asm/Runtime/ptmf.o\
	$(BUILD_DIR)/asm/Runtime/runtime.o\
	$(BUILD_DIR)/asm/Runtime/__init_cpp_exceptions.o\
	$(BUILD_DIR)/asm/Runtime/sdata.o\
	$(BUILD_DIR)/asm/Runtime/Gecko_ExceptionPPC.o\
	$(BUILD_DIR)/asm/Runtime/abort_exit.o\
	$(BUILD_DIR)/asm/Runtime/alloc.o\
	$(BUILD_DIR)/asm/Runtime/ansi_files.o\
	$(BUILD_DIR)/asm/Runtime/ansi_fp.o\
	$(BUILD_DIR)/asm/Runtime/arith.o\
	$(BUILD_DIR)/asm/Runtime/buffer_io.o\
	$(BUILD_DIR)/asm/Runtime/ctype.o\
	$(BUILD_DIR)/asm/Runtime/direct_io.o\
	$(BUILD_DIR)/asm/Runtime/file_io.o\
	$(BUILD_DIR)/asm/Runtime/FILE_POS.o\
	$(BUILD_DIR)/asm/Runtime/mbstring.o\
	$(BUILD_DIR)/asm/Runtime/mem.o\
	$(BUILD_DIR)/asm/Runtime/mem_funcs.o\
	$(BUILD_DIR)/asm/Runtime/misc_io.o\
	$(BUILD_DIR)/asm/Runtime/printf.o\
	$(BUILD_DIR)/asm/Runtime/qsort.o\
	$(BUILD_DIR)/asm/Runtime/rand.o\
	$(BUILD_DIR)/asm/Runtime/sscanf.o\
	$(BUILD_DIR)/asm/Runtime/string.o\
	$(BUILD_DIR)/asm/Runtime/strtold.o\
	$(BUILD_DIR)/asm/Runtime/uart_console_io.o\
	$(BUILD_DIR)/asm/Runtime/widechar_io.o\
	$(BUILD_DIR)/asm/Runtime/e_acos.o\
	$(BUILD_DIR)/asm/Runtime/e_asin.o\
	$(BUILD_DIR)/asm/Runtime/e_atan2.o\
	$(BUILD_DIR)/asm/Runtime/e_exp.o\
	$(BUILD_DIR)/asm/Runtime/e_fmod.o\
	$(BUILD_DIR)/asm/Runtime/e_log.o\
	$(BUILD_DIR)/asm/Runtime/e_pow.o\
	$(BUILD_DIR)/asm/Runtime/e_rem_pio2.o\
	$(BUILD_DIR)/asm/Runtime/k_cos.o\
	$(BUILD_DIR)/asm/Runtime/k_rem_pio2.o\
	$(BUILD_DIR)/asm/Runtime/k_sin.o\
	$(BUILD_DIR)/asm/Runtime/k_tan.o\
	$(BUILD_DIR)/asm/Runtime/s_atan.o\
	$(BUILD_DIR)/asm/Runtime/s_copysign.o\
	$(BUILD_DIR)/asm/Runtime/s_cos.o\
	$(BUILD_DIR)/asm/Runtime/s_floor.o\
	$(BUILD_DIR)/asm/Runtime/s_frexp.o\
	$(BUILD_DIR)/asm/Runtime/s_ldexp.o\
	$(BUILD_DIR)/asm/Runtime/s_modf.o\
	$(BUILD_DIR)/asm/Runtime/s_nextafter.o\
	$(BUILD_DIR)/asm/Runtime/s_sin.o\
	$(BUILD_DIR)/asm/Runtime/s_tan.o\
	$(BUILD_DIR)/asm/Runtime/s_acos.o\
	$(BUILD_DIR)/asm/Runtime/s_asin.o\
	$(BUILD_DIR)/asm/Runtime/s_atan2.o\
	$(BUILD_DIR)/asm/Runtime/s_exp.o\
	$(BUILD_DIR)/asm/Runtime/s_fmod.o\
	$(BUILD_DIR)/asm/Runtime/s_log.o\
	$(BUILD_DIR)/asm/Runtime/s_pow.o\
	$(BUILD_DIR)/asm/Runtime/s_cosf.o\
	
MUSYX_FILES :=\
	$(BUILD_DIR)/asm/musyx/seq.o\
	$(BUILD_DIR)/asm/musyx/synth.o\
	$(BUILD_DIR)/src/musyx/seq_api.o\
	$(BUILD_DIR)/asm/musyx/snd_synthapi.o\
	$(BUILD_DIR)/asm/musyx/stream.o\
	$(BUILD_DIR)/asm/musyx/synthdata.o\
	$(BUILD_DIR)/asm/musyx/synthmacros.o\
	$(BUILD_DIR)/asm/musyx/synthvoice.o\
	$(BUILD_DIR)/asm/musyx/synth_ac.o\
	$(BUILD_DIR)/asm/musyx/synth_adsr.o\
	$(BUILD_DIR)/asm/musyx/synth_vsamples.o\
	$(BUILD_DIR)/src/musyx/synth_dbtab.o\
	$(BUILD_DIR)/asm/musyx/s_data.o\
	$(BUILD_DIR)/asm/musyx/hw_dspctrl.o\
	$(BUILD_DIR)/asm/musyx/hw_volconv.o\
	$(BUILD_DIR)/asm/musyx/snd3d.o\
	$(BUILD_DIR)/src/musyx/snd_init.o\
	$(BUILD_DIR)/asm/musyx/snd_math.o\
	$(BUILD_DIR)/asm/musyx/snd_midictrl.o\
	$(BUILD_DIR)/src/musyx/snd_service.o\
	$(BUILD_DIR)/asm/musyx/hardware.o\
	$(BUILD_DIR)/asm/musyx/hw_aramdma.o\
	$(BUILD_DIR)/src/musyx/dsp_import.o\
	$(BUILD_DIR)/asm/musyx/hw_dolphin.o\
	$(BUILD_DIR)/src/musyx/hw_memory.o\
	$(BUILD_DIR)/src/musyx/creverb_fx.o\
	$(BUILD_DIR)/asm/musyx/creverb.o\
	$(BUILD_DIR)/src/musyx/reverb_fx.o\
	$(BUILD_DIR)/asm/musyx/reverb.o\
	$(BUILD_DIR)/src/musyx/delay_fx.o\
	$(BUILD_DIR)/asm/musyx/chorus_fx.o
	
DTK_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/dtk.o
	
CARD_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/card/CARDBios.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDUnlock.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDRdwr.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDBlock.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDDir.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDCheck.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDMount.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDFormat.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDOpen.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDCreate.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDRead.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDWrite.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDDelete.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDStat.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDRename.o\
	$(BUILD_DIR)/asm/Dolphin/card/CARDNet.o
	
SI_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/si/SIBios.o\
	$(BUILD_DIR)/asm/Dolphin/si/SISamplingRate.o
	
EXI_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/exi/EXIBios.o\
	$(BUILD_DIR)/asm/Dolphin/exi/EXIUart.o

THP_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/thp/THPDec.o\
	$(BUILD_DIR)/asm/Dolphin/thp/THPAudio.o
	
GBA_FILES :=\
	$(BUILD_DIR)/asm/Dolphin/GBA/GBA.o\
	$(BUILD_DIR)/asm/Dolphin/GBA/GBARead.o\
	$(BUILD_DIR)/asm/Dolphin/GBA/GBAWrite.o\
	$(BUILD_DIR)/asm/Dolphin/GBA/GBAXfer.o\
	$(BUILD_DIR)/asm/Dolphin/GBA/GBAKey.o
