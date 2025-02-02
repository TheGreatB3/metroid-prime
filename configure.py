#!/usr/bin/env python3

###
# Generates build files for the project.
# This file also includes the project configuration,
# such as compiler flags and the object matching status.
#
# Usage:
#   python3 configure.py
#   ninja
#
# Append --help to see available options.
###

import argparse
import sys
from pathlib import Path

from tools.project import (
    Object,
    ProgressCategory,
    ProjectConfig,
    calculate_progress,
    generate_build,
    is_windows,
)

# Game versions
DEFAULT_VERSION = 0
VERSIONS = [
    "GM8E01_00",  # mp-v1.088 NTSC-U
    "GM8E01_01",  # mp-v1.093 NTSC-U
    # "GM8E01_48",  # mp-v1.097 NTSC-K
    # "GM8P01_00",  # mp-v1.110 PAL
    # "GM8J01_00",  # mp-v1.111 NTSC-J
    # "GM8E01_02",  # mp-v1.111 NTSC-U
    # "R3IJ01_00",  # mp-v3.570 New Play Controls
    # "R3ME01_00",  # mp-v3.593 Trilogy NTSC
    # "R3MP01_00",  # mp-v3.629 Trilogy PAL
]

parser = argparse.ArgumentParser()
parser.add_argument(
    "mode",
    choices=["configure", "progress"],
    default="configure",
    help="script mode (default: configure)",
    nargs="?",
)
parser.add_argument(
    "-v",
    "--version",
    choices=VERSIONS,
    type=str.upper,
    default=VERSIONS[DEFAULT_VERSION],
    help="version to build",
)
parser.add_argument(
    "--build-dir",
    metavar="DIR",
    type=Path,
    default=Path("build"),
    help="base build directory (default: build)",
)
parser.add_argument(
    "--binutils",
    metavar="BINARY",
    type=Path,
    help="path to binutils (optional)",
)
parser.add_argument(
    "--compilers",
    metavar="DIR",
    type=Path,
    help="path to compilers (optional)",
)
parser.add_argument(
    "--map",
    action="store_true",
    help="generate map file(s)",
)
parser.add_argument(
    "--debug",
    action="store_true",
    help="build with debug info (non-matching)",
)
if not is_windows():
    parser.add_argument(
        "--wrapper",
        metavar="BINARY",
        type=Path,
        help="path to wibo or wine (optional)",
    )
parser.add_argument(
    "--dtk",
    metavar="BINARY | DIR",
    type=Path,
    help="path to decomp-toolkit binary or source (optional)",
)
parser.add_argument(
    "--objdiff",
    metavar="BINARY | DIR",
    type=Path,
    help="path to objdiff-cli binary or source (optional)",
)
parser.add_argument(
    "--sjiswrap",
    metavar="EXE",
    type=Path,
    help="path to sjiswrap.exe (optional)",
)
parser.add_argument(
    "--verbose",
    action="store_true",
    help="print verbose output",
)
parser.add_argument(
    "--non-matching",
    dest="non_matching",
    action="store_true",
    help="builds equivalent (but non-matching) or modded objects",
)
parser.add_argument(
    "--no-progress",
    dest="progress",
    action="store_false",
    help="disable progress calculation",
)
args = parser.parse_args()

config = ProjectConfig()
config.version = str(args.version)
version_num = VERSIONS.index(config.version)

# Apply arguments
config.build_dir = args.build_dir
config.dtk_path = args.dtk
config.objdiff_path = args.objdiff
config.binutils_path = args.binutils
config.compilers_path = args.compilers
config.generate_map = args.map
config.non_matching = args.non_matching
config.sjiswrap_path = args.sjiswrap
config.progress = args.progress
if not is_windows():
    config.wrapper = args.wrapper
# Don't build asm unless we're --non-matching
if not config.non_matching:
    config.asm_dir = None

# Tool versions
config.binutils_tag = "2.42-1"
config.compilers_tag = "20240706"
config.dtk_tag = "v1.2.0"
config.objdiff_tag = "v2.3.4"
config.sjiswrap_tag = "v1.2.0"
config.wibo_tag = "0.6.11"

# Project
config.config_path = Path("config") / config.version / "config.yml"
config.check_sha_path = Path("config") / config.version / "build.sha1"
config.asflags = [
    "-mgekko",
    "--strip-local-absolute",
    "-I include",
    f"-I build/{config.version}/include",
    f"--defsym version={version_num}",
]
config.ldflags = [
    "-fp hardware",
    "-nodefaults",
]
if args.debug:
    config.ldflags.append("-g")
if args.map:
    config.ldflags.append("-mapunused")

config.build_rels = False

# Base flags, common to most GC/Wii games.
# Generally leave untouched, with overrides added below.
cflags_base = [
    "-nodefaults",
    "-proc gekko",
    "-align powerpc",
    "-enum int",
    "-fp hardware",
    "-Cpp_exceptions off",
    # "-W all",
    "-O4,p",
    "-inline auto",
    '-pragma "cats off"',
    '-pragma "warn_notinlined off"',
    "-maxerrors 1",
    "-nosyspath",
    "-RTTI off",
    "-fp_contract on",
    "-str reuse",
    "-i include",
    "-i extern/musyx/include",
    "-i libc",
    f"-i build/{config.version}/include",
    f"-DVERSION={version_num}",
    "-DPRIME1",
    "-DNONMATCHING=0",
]

# Debug flags
if args.debug:
    cflags_base.extend(["-sym on", "-DDEBUG=1"])
else:
    cflags_base.append("-DNDEBUG=1")

# Dolphin flags
cflags_dolphin = [
    *cflags_base,
    "-multibyte",
    "-fp_contract off",
]

# Metrowerks library flags
cflags_runtime = [
    *cflags_base,
    "-use_lmw_stmw on",
    "-str reuse,pool,readonly",
    "-gccinc",
    "-common off",
    "-char signed",
    "-inline deferred,auto",
]

cflags_retro = [
    *cflags_base,
    "-use_lmw_stmw on",
    "-str reuse,pool,readonly",
    "-gccinc",
    "-inline deferred,noauto",
    "-common on",
    "-DMUSY_TARGET=MUSY_TARGET_DOLPHIN",
]

cflags_musyx = [
    "-proc gekko",
    "-nodefaults",
    "-nosyspath",
    "-i include",
    "-i extern/musyx/include",
    "-i libc",
    "-inline auto,depth=4",
    "-O4,p",
    "-fp hard",
    "-enum int",
    "-Cpp_exceptions off",
    "-str reuse,pool,readonly",
    "-fp_contract off",
    "-DMUSY_TARGET=MUSY_TARGET_DOLPHIN",
]

cflags_musyx_debug = [
    "-proc gecko",
    "-fp hard",
    "-nodefaults",
    "-nosyspath",
    "-i include",
    "-i extern/musyx/include",
    "-i libc",
    "-g",
    "-sym on",
    "-D_DEBUG=1",
    "-fp hard",
    "-enum int",
    "-Cpp_exceptions off",
    "-DMUSY_TARGET=MUSY_TARGET_DOLPHIN",
]

# REL flags
cflags_rel = [
    "-proc gecko",
    "-fp hard",
    "-nodefaults",
    "-nosyspath",
    "-i include",
    "-i libc",
    "-O0",
    "-sdata 0",
    "-sdata2 0",
    "-str noreuse",
    "-Cpp_exceptions off",
]

config.linker_version = "GC/1.3.2"


# Helper function for Dolphin libraries
def DolphinLib(lib_name, objects):
    return {
        "lib": lib_name + "D" if args.debug else "",
        "mw_version": "GC/1.2.5n",
        "cflags": cflags_dolphin,
        "host": False,
        "progress_category": "sdk",
        "objects": objects,
        "shift_jis": True,
    }


def RetroLib(lib_name, progress_category, objects):
    return {
        "lib": lib_name + "CW" + "D" if args.debug else "",
        "mw_version": "GC/1.3.2",
        "cflags": cflags_retro,
        "host": False,
        "progress_category": progress_category,
        "objects": objects,
        "shift_jis": False,
    }


def MusyX(objects, mw_version="GC/1.3.2", debug=False, major=2, minor=0, patch=0):
    cflags = cflags_musyx if not debug else cflags_musyx_debug
    return {
        "lib": "musyx",
        "mw_version": mw_version,
        "src_dir": "extern/musyx/src",
        "host": False,
        "cflags": [
            *cflags,
            f"-DMUSY_VERSION_MAJOR={major}",
            f"-DMUSY_VERSION_MINOR={minor}",
            f"-DMUSY_VERSION_PATCH={patch}",
        ],
        "progress_category": "third_party",
        "objects": objects,
        "shift_jis": False,
    }


# Helper function for REL script objects
def Rel(lib_name, objects):
    return {
        "lib": lib_name,
        "mw_version": "GC/1.3.2",
        "cflags": cflags_rel,
        "host": True,
        "progress_category": "third_party",
        "objects": objects,
        "shift_jis": False,
    }


Matching = True  # Object matches and should be linked
NonMatching = False  # Object does not match and should not be linked
Equivalent = (
    config.non_matching
)  # Object should be linked when configured with --non-matching


# Object is only matching for specific versions
def MatchingFor(*versions):
    return config.version in versions


config.warn_missing_config = True
config.warn_missing_source = False
config.libs = [
    DolphinLib(
        "TRK_MINNOW_DOLPHIN",
        [
            Object(Matching, "MetroTRK/mslsupp.c"),
        ],
    ),
    RetroLib(
        "MetroidPrime",
        "game",
        [
            Object(NonMatching, "MetroidPrime/main.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CCameraManager.cpp"),
            Object(Matching, "MetroidPrime/CControlMapper.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CFirstPersonCamera.cpp"),
            Object(Matching, "MetroidPrime/CObjectList.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayer.cpp"),
            Object(Matching, "MetroidPrime/CAxisAngle.cpp"),
            Object(NonMatching, "MetroidPrime/CEulerAngles.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmUserInput.cpp"),
            Object(NonMatching, "MetroidPrime/CFrontEndUI.cpp"),
            Object(NonMatching, "MetroidPrime/CInputGenerator.cpp"),
            Object(NonMatching, "MetroidPrime/CMainFlow.cpp"),
            Object(NonMatching, "MetroidPrime/CMFGame.cpp"),
            Object(NonMatching, "MetroidPrime/CCredits.cpp"),
            Object(NonMatching, "MetroidPrime/CSplashScreen.cpp"),
            Object(NonMatching, "MetroidPrime/CAnimData.cpp"),
            Object(NonMatching, "MetroidPrime/Factories/CCharacterFactory.cpp"),
            Object(NonMatching, "MetroidPrime/Factories/CAssetFactory.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakPlayer.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweaks.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakGame.cpp"),
            Object(NonMatching, "MetroidPrime/CGameProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerGun.cpp"),
            Object(NonMatching, "MetroidPrime/CStateManager.cpp"),
            Object(Matching, "MetroidPrime/CEntity.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmInt32.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmInt32Int32VoidPtr.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmNull.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmReal32.cpp"),
            Object(Matching, "MetroidPrime/Decode.cpp"),
            Object(NonMatching, "MetroidPrime/CIOWinManager.cpp"),
            Object(Matching, "MetroidPrime/CIOWin.cpp"),
            Object(NonMatching, "MetroidPrime/CActor.cpp"),
            Object(NonMatching, "MetroidPrime/CWorld.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakParticle.cpp"),
            Object(Matching, "MetroidPrime/Clamp_int.cpp"),
            Object(Matching, "MetroidPrime/CArchMsgParmControllerStatus.cpp"),
            Object(Matching, "MetroidPrime/CExplosion.cpp"),
            Object(Matching, "MetroidPrime/CEffect.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CGameCamera.cpp"),
            Object(NonMatching, "MetroidPrime/CGameArea.cpp"),
            Object(NonMatching, "MetroidPrime/HUD/CSamusHud.cpp"),
            Object(NonMatching, "MetroidPrime/CAnimationDatabaseGame.cpp"),
            Object(NonMatching, "MetroidPrime/CTransitionDatabaseGame.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakPlayerControl.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakPlayerGun.cpp"),
            Object(NonMatching, "MetroidPrime/CPauseScreen.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakGui.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptActor.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptTrigger.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptWaypoint.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPatterned.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptDoor.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CStateMachine.cpp"),
            Object(NonMatching, "MetroidPrime/CMapArea.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CBallCamera.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptEffect.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CBomb.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakBall.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerState.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptTimer.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CCinematicCamera.cpp"),
            Object(NonMatching, "MetroidPrime/CAutoMapper.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCounter.cpp"),
            Object(NonMatching, "MetroidPrime/CMapWorld.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CAi.cpp"),
            Object(Matching, "MetroidPrime/Enemies/PatternedCastTo.cpp"),
            Object(Matching, "MetroidPrime/TCastTo.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptSound.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptPlatform.cpp"),
            Object(Matching, "MetroidPrime/UserNames.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptGenerator.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCameraWaypoint.cpp"),
            Object(Matching, "MetroidPrime/CGameLight.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakTargeting.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakAutoMapper.cpp"),
            Object(Matching, "MetroidPrime/CParticleGenInfoGeneric.cpp"),
            Object(Matching, "MetroidPrime/CParticleGenInfo.cpp"),
            Object(NonMatching, "MetroidPrime/CParticleDatabase.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakGunRes.cpp"),
            Object(NonMatching, "MetroidPrime/CTargetReticles.cpp"),
            Object(NonMatching, "MetroidPrime/CWeaponMgr.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptPickup.cpp"),
            Object(NonMatching, "MetroidPrime/CDamageInfo.cpp"),
            Object(Matching, "MetroidPrime/CMemoryDrawEnum.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptDock.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptCameraHint.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptLoader.cpp"),
            Object(NonMatching, "MetroidPrime/CSamusDoll.cpp"),
            Object(Matching if config.version == "GM8E01_00" else NonMatching, "MetroidPrime/Factories/CStateMachineFactory.cpp"),
            Object(Matching, "MetroidPrime/Weapons/CPlasmaBeam.cpp"),
            Object(Matching, "MetroidPrime/Weapons/CPowerBeam.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CWaveBeam.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CIceBeam.cpp"),
            Object(Matching, "MetroidPrime/CScriptMailbox.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptRelay.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptSpawnPoint.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptRandomRelay.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CBeetle.cpp"),
            Object(Matching, "MetroidPrime/HUD/CHUDMemoParms.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptHUDMemo.cpp"),
            Object(NonMatching, "MetroidPrime/CMappableObject.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerCameraBob.cpp"),
            Object(
                Matching, "MetroidPrime/ScriptObjects/CScriptCameraFilterKeyframe.cpp"
            ),
            Object(
                Matching, "MetroidPrime/ScriptObjects/CScriptCameraBlurKeyframe.cpp"
            ),
            Object(NonMatching, "MetroidPrime/Cameras/CCameraFilter.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CMorphBall.cpp"),
            Object(
                NonMatching, "MetroidPrime/ScriptObjects/CScriptDamageableTrigger.cpp"
            ),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptDebris.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCameraShaker.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptActorKeyframe.cpp"),
            Object(Matching, "MetroidPrime/CConsoleOutputWindow.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptWater.cpp"),
            Object(Matching, "MetroidPrime/Weapons/CWeapon.cpp"),
            Object(NonMatching, "MetroidPrime/CDamageVulnerability.cpp"),
            Object(NonMatching, "MetroidPrime/CActorLights.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CPatternedInfo.cpp"),
            Object(NonMatching, "MetroidPrime/CSimpleShadow.cpp"),
            Object(Matching, "MetroidPrime/CActorParameters.cpp"),
            Object(NonMatching, "MetroidPrime/CInGameGuiManager.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CWarWasp.cpp"),
            Object(NonMatching, "MetroidPrime/CWorldShadow.cpp"),
            Object(Matching, "MetroidPrime/CAudioStateWin.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerVisor.cpp"),
            Object(NonMatching, "MetroidPrime/CModelData.cpp"),
            Object(NonMatching, "MetroidPrime/CDecalManager.cpp"),
            Object(
                NonMatching, "MetroidPrime/ScriptObjects/CScriptSpiderBallWaypoint.cpp"
            ),
            Object(NonMatching, "MetroidPrime/Enemies/CBloodFlower.cpp"),
            Object(Matching, "MetroidPrime/TGameTypes.cpp"),
            Object(NonMatching, "MetroidPrime/CPhysicsActor.cpp"),
            Object(Matching, "MetroidPrime/CPhysicsState.cpp"),
            Object(NonMatching, "MetroidPrime/CRipple.cpp"),
            Object(Matching, "MetroidPrime/CFluidUVMotion.cpp"),
            Object(NonMatching, "MetroidPrime/CRippleManager.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CGrappleArm.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CSpacePirate.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCoverPoint.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CPathCamera.cpp"),
            Object(NonMatching, "MetroidPrime/CFluidPlane.cpp"),
            Object(NonMatching, "MetroidPrime/CFluidPlaneManager.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptGrapplePoint.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CHUDBillboardEffect.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CFlickerBat.cpp"),
            Object(
                NonMatching,
                "MetroidPrime/BodyState/CBodyStateCmdMgr.cpp",
                cflags=[*cflags_retro, "-inline auto"],
            ),
            Object(NonMatching, "MetroidPrime/BodyState/CBodyStateInfo.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSAttack.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSDie.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSFall.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSGetup.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSKnockBack.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSLieOnGround.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSLocomotion.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSStep.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSTurn.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBodyController.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSLoopAttack.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CTargetableProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSLoopReaction.cpp"),
            Object(NonMatching, "MetroidPrime/CSteeringBehaviors.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSGroundHit.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CChozoGhost.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CFireFlea.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSSlide.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSHurled.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSJump.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSGenerate.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPuddleSpore.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSTaunt.cpp"),
            Object(NonMatching, "MetroidPrime/CSortedLists.cpp"),
            Object(
                Matching, "MetroidPrime/ScriptObjects/CScriptDebugCameraWaypoint.cpp"
            ),
            Object(
                NonMatching,
                "MetroidPrime/ScriptObjects/CScriptSpiderBallAttractionSurface.cpp",
            ),
            Object(Matching, "MetroidPrime/BodyState/CBSScripted.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPuddleToadGamma.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptDistanceFog.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CBSProjectileAttack.cpp"),
            Object(Matching, "MetroidPrime/Weapons/CPowerBomb.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetaree.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptDockAreaChange.cpp"),
            Object(
                NonMatching, "MetroidPrime/ScriptObjects/CScriptSpecialFunction.cpp"
            ),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptActorRotate.cpp"),
            Object(Matching, "MetroidPrime/Player/CFidget.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CSpankWeed.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CParasite.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CSamusFaceReflection.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptPlayerHint.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CRipper.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CCameraShakeData.cpp"),
            Object(
                NonMatching, "MetroidPrime/ScriptObjects/CScriptPickupGenerator.cpp"
            ),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptPointOfInterest.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CDrone.cpp"),
            Object(NonMatching, "MetroidPrime/CMapWorldInfo.cpp"),
            Object(NonMatching, "MetroidPrime/Factories/CScannableObjectInfo.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetroid.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CScanDisplay.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptSteam.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptRipple.cpp"),
            Object(NonMatching, "MetroidPrime/CBoneTracking.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CFaceplateDecoration.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSCover.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptBallTrigger.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CPlasmaProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerOrbit.cpp"),
            Object(NonMatching, "MetroidPrime/CGameCollision.cpp"),
            Object(Matching, "MetroidPrime/CBallFilter.cpp"),
            Object(Matching, "MetroidPrime/CAABoxFilter.cpp"),
            Object(NonMatching, "MetroidPrime/CGroundMovement.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CNewIntroBoss.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CPhazonBeam.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptTargetingPoint.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CBSWallHang.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptEMPulse.cpp"),
            Object(NonMatching, "MetroidPrime/HUD/CHudDecoInterface.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CFlameThrower.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CBeamProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/CFluidPlaneCPU.cpp"),
            Object(NonMatching, "MetroidPrime/CFluidPlaneDoor.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptRoomAcoustics.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CIceSheegoth.cpp"),
            Object(NonMatching, "MetroidPrime/CCollisionActorManager.cpp"),
            Object(NonMatching, "MetroidPrime/CCollisionActor.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptPlayerActor.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakPlayerRes.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CBurstFire.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CFlaahgra.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerEnergyDrain.cpp"),
            Object(NonMatching, "MetroidPrime/CFlameWarp.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CIceImpact.cpp"),
            Object(Matching, "MetroidPrime/GameObjectLists.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CAuxWeapon.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CGunWeapon.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptAreaAttributes.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CWaveBuster.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CStaticInterference.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetroidBeta.cpp"),
            Object(NonMatching, "MetroidPrime/PathFinding/CPathFindSearch.cpp"),
            Object(NonMatching, "MetroidPrime/PathFinding/CPathFindRegion.cpp"),
            Object(NonMatching, "MetroidPrime/PathFinding/CPathFindArea.cpp"),
            Object(
                NonMatching, "MetroidPrime/Weapons/GunController/CGunController.cpp"
            ),
            Object(NonMatching, "MetroidPrime/Weapons/GunController/CGSFreeLook.cpp"),
            Object(Matching, "MetroidPrime/Weapons/GunController/CGSComboFire.cpp"),
            Object(NonMatching, "MetroidPrime/HUD/CHudBallInterface.cpp"),
            Object(NonMatching, "MetroidPrime/Tweaks/CTweakGuiColors.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CFishCloud.cpp"),
            Object(Matching, "MetroidPrime/CHealthInfo.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CGameState.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptVisorFlare.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptWorldTeleporter.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptVisorGoo.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CJellyZap.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptControllerAction.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/GunController/CGunMotion.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptSwitch.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CABSIdle.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CABSFlinch.cpp"),
            Object(NonMatching, "MetroidPrime/BodyState/CABSAim.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptPlayerStateChange.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CThardus.cpp"),
            Object(NonMatching, "MetroidPrime/CActorParticles.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CWallCrawlerSwarm.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptAiJumpPoint.cpp"),
            Object(NonMatching, "MetroidPrime/CMessageScreen.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CFlaahgraTentacle.cpp"),
            Object(Matching, "MetroidPrime/Weapons/GunController/CGSFidget.cpp"),
            Object(Matching, "MetroidPrime/BodyState/CABSReaction.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CIceProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CFlyingPirate.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptColorModulate.cpp"),
            Object(NonMatching, "MetroidPrime/CMapUniverse.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CThardusRockProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/CInventoryScreen.cpp"),
            Object(NonMatching, "MetroidPrime/CVisorFlare.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CFlaahgraPlants.cpp"),
            Object(NonMatching, "MetroidPrime/CWorldTransManager.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptMidi.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptStreamedAudio.cpp"),
            Object(NonMatching, "MetroidPrime/CRagDoll.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CGameOptions.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CRepulsor.cpp"),
            Object(NonMatching, "MetroidPrime/CEnvFxManager.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CEnergyProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptGunTurret.cpp"),
            Object(Matching, "MetroidPrime/Weapons/CProjectileInfo.cpp"),
            Object(NonMatching, "MetroidPrime/CInGameTweakManager.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CBabygoth.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CEyeBall.cpp"),
            Object(NonMatching, "MetroidPrime/CIkChain.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCameraPitchVolume.cpp"),
            Object(MatchingFor("GM8E01_00"), "MetroidPrime/RumbleFxTable.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CElitePirate.cpp"),
            Object(Matching, "MetroidPrime/CRumbleManager.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CBouncyGrenade.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CGrenadeLauncher.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CShockWave.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CRipperControlledPlatform.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CKnockBackController.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CWorldLayerState.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMagdolite.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CTeamAiMgr.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CSnakeWeedSwarm.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CBallCameraFailsafeState.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CActorContraption.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CScriptSpindleCamera.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptMemoryRelay.cpp"),
            Object(NonMatching, "MetroidPrime/CPauseScreenFrame.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CAtomicAlpha.cpp"),
            Object(NonMatching, "MetroidPrime/CLogBookScreen.cpp"),
            Object(Matching, "MetroidPrime/CGBASupport.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CWorldSaveGameInfo.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptCameraHintTrigger.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CAmbientAI.cpp"),
            Object(NonMatching, "MetroidPrime/CMemoryCardDriver.cpp"),
            Object(NonMatching, "MetroidPrime/CSaveGameScreen.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CAtomicBeta.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CElectricBeamProjectile.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CRidley.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPuffer.cpp"),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CFire.cpp"),
            Object(Matching, "MetroidPrime/CPauseScreenBlur.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CTryclops.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/CNewFlameThrower.cpp"),
            Object(NonMatching, "MetroidPrime/Cameras/CInterpolationCamera.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CSeedling.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CGameHintInfo.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CWallWalker.cpp"),
            Object(NonMatching, "MetroidPrime/CErrorOutputWindow.cpp"),
            Object(NonMatching, "MetroidPrime/CRainSplashGenerator.cpp"),
            Object(NonMatching, "MetroidPrime/Factories/CWorldSaveGameInfoFactory.cpp"),
            Object(NonMatching, "MetroidPrime/CFluidPlaneRender.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CBurrower.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetroidPrimeExo.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptBeam.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetroidPrimeEssence.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CMetroidPrimeRelay.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerDynamics.cpp"),
            Object(Matching, "MetroidPrime/ScriptObjects/CScriptMazeNode.cpp"),
            Object(NonMatching, "MetroidPrime/Weapons/WeaponTypes.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/COmegaPirate.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPhazonPool.cpp"),
            Object(NonMatching, "MetroidPrime/CNESEmulator.cpp"),
            Object(NonMatching, "MetroidPrime/Enemies/CPhazonHealingNodule.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CMorphBallShadow.cpp"),
            Object(NonMatching, "MetroidPrime/Player/CPlayerStuckTracker.cpp"),
            Object(NonMatching, "MetroidPrime/CSlideShow.cpp"),
            Object(Matching, "MetroidPrime/Tweaks/CTweakSlideShow.cpp"),
            Object(NonMatching, "MetroidPrime/CArtifactDoll.cpp"),
            Object(NonMatching, "MetroidPrime/CProjectedShadow.cpp"),
            Object(Matching, "MetroidPrime/CPreFrontEnd.cpp"),
            Object(Matching, "MetroidPrime/CGameCubeDoll.cpp"),
            Object(
                NonMatching, "MetroidPrime/ScriptObjects/CScriptProjectedShadow.cpp"
            ),
            Object(NonMatching, "MetroidPrime/ScriptObjects/CEnergyBall.cpp"),
            Object(Matching, "MetroidPrime/Enemies/CMetroidPrimeProjectile.cpp"),
            Object(Matching, "MetroidPrime/Enemies/SPositionHistory.cpp"),
        ],
    ),
    RetroLib(
        "WorldFormat",
        "core",
        [
            Object(NonMatching, "WorldFormat/CAreaOctTree_Tests.cpp"),
            Object(Matching, "WorldFormat/CCollisionSurface.cpp"),
            Object(Matching, "WorldFormat/CMetroidModelInstance.cpp"),
            Object(Matching, "WorldFormat/CAreaBspTree.cpp"),
            Object(NonMatching, "WorldFormat/CAreaOctTree.cpp"),
            Object(NonMatching, "WorldFormat/CMetroidAreaCollider.cpp"),
            Object(NonMatching, "WorldFormat/CWorldLight.cpp"),
            Object(NonMatching, "WorldFormat/COBBTree.cpp"),
            Object(NonMatching, "WorldFormat/CCollidableOBBTree.cpp"),
            Object(NonMatching, "WorldFormat/CCollidableOBBTreeGroup.cpp"),
            Object(NonMatching, "WorldFormat/CPVSAreaSet.cpp"),
            Object(NonMatching, "WorldFormat/CAreaRenderOctTree.cpp"),
        ],
    ),
    RetroLib(
        "Weapons",
        "core",
        [
            Object(NonMatching, "Weapons/CProjectileWeapon.cpp"),
            Object(NonMatching, "Weapons/CProjectileWeaponDataFactory.cpp"),
            Object(NonMatching, "Weapons/CCollisionResponseData.cpp"),
            Object(Matching, "Weapons/IWeaponRenderer.cpp"),
            Object(NonMatching, "Weapons/CDecalDataFactory.cpp"),
            Object(NonMatching, "Weapons/CDecal.cpp"),
            Object(NonMatching, "Weapons/CWeaponDescription.cpp"),
            Object(Matching, "Weapons/CDecalDescription.cpp"),
        ],
    ),
    RetroLib(
        "MetaRender",
        "core",
        [
            Object(NonMatching, "MetaRender/CCubeRenderer.cpp"),
        ],
    ),
    RetroLib(
        "GuiSys",
        "core",
        [
            Object(Matching, "GuiSys/CAuiMain.cpp"),
            Object(NonMatching, "GuiSys/CAuiMeter.cpp"),
            Object(NonMatching, "GuiSys/CGuiGroup.cpp"),
            Object(NonMatching, "GuiSys/CGuiHeadWidget.cpp"),
            Object(Matching, "GuiSys/CGuiLight.cpp"),
            Object(NonMatching, "GuiSys/CGuiModel.cpp"),
            Object(NonMatching, "GuiSys/CGuiObject.cpp"),
            Object(NonMatching, "GuiSys/CGuiPane.cpp"),
            Object(NonMatching, "GuiSys/CGuiSliderGroup.cpp"),
            Object(Matching, "GuiSys/CGuiSys.cpp"),
            Object(NonMatching, "GuiSys/CGuiTableGroup.cpp"),
            Object(NonMatching, "GuiSys/CGuiTextPane.cpp"),
            Object(NonMatching, "GuiSys/CGuiTextSupport.cpp"),
            Object(NonMatching, "GuiSys/CGuiWidget.cpp"),
            Object(Matching, "GuiSys/CGuiWidgetIdDB.cpp"),
            Object(Matching, "GuiSys/CGuiWidgetDrawParms.cpp"),
            Object(NonMatching, "GuiSys/CAuiEnergyBarT01.cpp"),
            Object(NonMatching, "GuiSys/CAuiImagePane.cpp"),
            Object(Matching, "GuiSys/CRepeatState.cpp"),
        ],
    ),
    RetroLib(
        "Collision",
        "core",
        [
            Object(NonMatching, "Collision/CCollidableAABox.cpp"),
            Object(Matching, "Collision/CCollidableCollisionSurface.cpp"),
            Object(Matching, "Collision/CCollisionInfo.cpp"),
            Object(Matching, "Collision/InternalColliders.cpp"),
            Object(NonMatching, "Collision/CCollisionPrimitive.cpp"),
            Object(Matching, "Collision/CMaterialList.cpp"),
            Object(NonMatching, "Collision/CollisionUtil.cpp"),
            Object(NonMatching, "Collision/CCollidableSphere.cpp"),
            Object(Matching, "Collision/CMaterialFilter.cpp"),
            Object(NonMatching, "Collision/COBBox.cpp"),
            Object(Matching, "Collision/CMRay.cpp"),
        ],
    ),
    RetroLib(
        "Kyoto1",
        "core",
        [
            Object(Matching, "Kyoto/Basics/CBasics.cpp"),
            Object(Matching, "Kyoto/Basics/CStopwatch.cpp"),
            Object(Matching, "Kyoto/Basics/CBasicsDolphin.cpp"),
            Object(Matching, "Kyoto/Alloc/CCallStackDolphin.cpp"),
            Object(Matching, "Kyoto/Basics/COsContextDolphin.cpp"),
            Object(Matching, "Kyoto/Basics/CSWDataDolphin.cpp"),
            Object(Matching, "Kyoto/Basics/RAssertDolphin.cpp"),
            Object(Matching, "Kyoto/Animation/CAnimation.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimationManager.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimationSet.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimCharacterSet.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeLoopIn.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeSequence.cpp"),
            Object(NonMatching, "Kyoto/Animation/CCharacterInfo.cpp"),
            Object(NonMatching, "Kyoto/Animation/CCharacterSet.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimBlend.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimFactory.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimPhaseBlend.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimPlay.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimRandom.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaAnimSequence.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaTransFactory.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaTransMetaAnim.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaTransPhaseTrans.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaTransSnap.cpp"),
            Object(NonMatching, "Kyoto/Animation/CMetaTransTrans.cpp"),
            Object(Matching, "Kyoto/Animation/CPASAnimInfo.cpp"),
            Object(Matching, "Kyoto/Animation/CPASAnimParm.cpp"),
            Object(NonMatching, "Kyoto/Animation/CPASAnimState.cpp"),
            Object(NonMatching, "Kyoto/Animation/CPASDatabase.cpp"),
            Object(Matching, "Kyoto/Animation/CPASParmInfo.cpp"),
            Object(Matching, "Kyoto/Animation/CPrimitive.cpp"),
            Object(NonMatching, "Kyoto/Animation/CSequenceHelper.cpp"),
            Object(Matching, "Kyoto/Animation/CTransition.cpp"),
            Object(Matching, "Kyoto/Animation/CTransitionManager.cpp"),
            Object(NonMatching, "Kyoto/Animation/CTreeUtils.cpp"),
            Object(NonMatching, "Kyoto/Animation/IMetaAnim.cpp"),
            Object(Matching, "Kyoto/Audio/CSfxHandle.cpp"),
            Object(NonMatching, "Kyoto/Audio/CSfxManager.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAdvancementDeltas.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimMathUtils.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimPOIData.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimSource.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimSourceReader.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimSourceReaderBase.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeAnimReaderContainer.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeBlend.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeContinuousPhaseBlend.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeDoubleChild.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeNode.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeSingleChild.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeTimeScale.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeTransition.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAnimTreeTweenBase.cpp"),
            Object(Matching, "Kyoto/Animation/CBoolPOINode.cpp"),
            Object(NonMatching, "Kyoto/Animation/CCharAnimMemoryMetrics.cpp"),
            Object(NonMatching, "Kyoto/Animation/CCharLayoutInfo.cpp"),
            Object(NonMatching, "Kyoto/Animation/CFBStreamedAnimReader.cpp"),
            Object(NonMatching, "Kyoto/Animation/CFBStreamedCompression.cpp"),
            Object(NonMatching, "Kyoto/Animation/CHierarchyPoseBuilder.cpp"),
            Object(Matching, "Kyoto/Animation/CInt32POINode.cpp"),
            Object(Matching, "Kyoto/Animation/CParticlePOINode.cpp"),
            Object(Matching, "Kyoto/Animation/CPOINode.cpp"),
            Object(NonMatching, "Kyoto/Animation/CSegStatementSet.cpp"),
            Object(NonMatching, "Kyoto/Animation/CTimeScaleFunctions.cpp"),
            Object(NonMatching, "Kyoto/Animation/IAnimReader.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAllFormatsAnimSource.cpp"),
            Object(Matching, "Kyoto/CDvdRequestManager.cpp"),
            Object(Matching, "Kyoto/CDvdRequest.cpp"),
            Object(Matching, "Kyoto/Text/CColorInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CColorOverrideInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CDrawStringOptions.cpp"),
            Object(NonMatching, "Kyoto/Text/CFontInstruction.cpp"),
            Object(NonMatching, "Kyoto/Text/CFontRenderState.cpp"),
            Object(Matching, "Kyoto/Text/CLineExtraSpaceInstruction.cpp"),
            Object(NonMatching, "Kyoto/Text/CLineInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CLineSpacingInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CPopStateInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CPushStateInstruction.cpp"),
            Object(NonMatching, "Kyoto/Text/CRasterFont.cpp"),
            Object(Matching, "Kyoto/Text/CRemoveColorOverrideInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CSaveableState.cpp"),
            Object(NonMatching, "Kyoto/Text/CTextExecuteBuffer.cpp"),
            Object(NonMatching, "Kyoto/Text/CTextInstruction.cpp"),
            Object(NonMatching, "Kyoto/Text/CTextParser.cpp"),
            Object(NonMatching, "Kyoto/Text/CWordBreakTables.cpp"),
            Object(NonMatching, "Kyoto/Text/CWordInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CBlockInstruction.cpp"),
            Object(Matching, "Kyoto/Text/CFont.cpp"),
            Object(Matching, "Kyoto/Graphics/CLight.cpp"),
            Object(NonMatching, "Kyoto/Graphics/CCubeModel.cpp"),
            Object(Matching, "Kyoto/Graphics/CGX.cpp"),
            Object(Matching, "Kyoto/Graphics/CTevCombiners.cpp"),
            Object(NonMatching, "Kyoto/Graphics/DolphinCGraphics.cpp"),
            Object(Matching, "Kyoto/Graphics/DolphinCPalette.cpp"),
            Object(Matching, "Kyoto/Graphics/DolphinCTexture.cpp"),
            Object(Matching, "Kyoto/Math/CloseEnough.cpp"),
            Object(NonMatching, "Kyoto/Math/CMatrix3f.cpp"),
            Object(NonMatching, "Kyoto/Math/CMatrix4f.cpp"),
            Object(NonMatching, "Kyoto/Math/CQuaternion.cpp"),
            Object(Matching, "Kyoto/CRandom16.cpp"),
            Object(NonMatching, "Kyoto/Math/CTransform4f.cpp"),
            Object(Matching, "Kyoto/Math/CUnitVector3f.cpp"),
            Object(Matching, "Kyoto/Math/CVector2f.cpp"),
            Object(Matching, "Kyoto/Math/CVector2i.cpp"),
            Object(Matching, "Kyoto/Math/CVector3d.cpp"),
            Object(Matching, "Kyoto/Math/CVector3f.cpp"),
            Object(Matching, "Kyoto/Math/CVector3i.cpp"),
            Object(NonMatching, "Kyoto/Math/RMathUtils.cpp"),
            Object(Matching, "Kyoto/CCrc32.cpp"),
            Object(Matching, "Kyoto/Alloc/CCircularBuffer.cpp"),
            Object(Matching, "Kyoto/Alloc/CMemory.cpp"),
            Object(Matching, "Kyoto/Alloc/IAllocator.cpp"),
            Object(NonMatching, "Kyoto/PVS/CPVSVisOctree.cpp"),
            Object(NonMatching, "Kyoto/PVS/CPVSVisSet.cpp"),
            Object(Matching, "Kyoto/Particles/CColorElement.cpp"),
            Object(NonMatching, "Kyoto/Particles/CElementGen.cpp"),
            Object(Matching, "Kyoto/Particles/CIntElement.cpp"),
            Object(Matching, "Kyoto/Particles/CModVectorElement.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleDataFactory.cpp"),
            Object(Matching, "Kyoto/Particles/CParticleGen.cpp"),
            Object(Matching, "Kyoto/Particles/CParticleGlobals.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleSwoosh.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleSwooshDataFactory.cpp"),
            Object(Matching, "Kyoto/Particles/CRealElement.cpp"),
            Object(NonMatching, "Kyoto/Particles/CSpawnSystemKeyframeData.cpp"),
            Object(NonMatching, "Kyoto/Particles/CUVElement.cpp"),
            Object(NonMatching, "Kyoto/Particles/CVectorElement.cpp"),
            Object(Matching, "Kyoto/Particles/CWarp.cpp"),
            Object(Matching, "Kyoto/Math/CPlane.cpp"),
            Object(Matching, "Kyoto/Math/CSphere.cpp"),
            Object(NonMatching, "Kyoto/Math/CAABox.cpp"),
            Object(NonMatching, "Kyoto/CFactoryMgr.cpp"),
            Object(NonMatching, "Kyoto/CResFactory.cpp"),
            Object(NonMatching, "Kyoto/CResLoader.cpp"),
            Object(NonMatching, "Kyoto/rstl/rstl_map.cpp"),
            Object(NonMatching, "Kyoto/rstl/rstl_strings.cpp"),
            Object(NonMatching, "Kyoto/rstl/RstlExtras.cpp"),
            Object(Matching, "Kyoto/Streams/CInputStream.cpp"),
            Object(Matching, "Kyoto/Streams/CMemoryInStream.cpp"),
            Object(Matching, "Kyoto/Streams/CMemoryStreamOut.cpp"),
            Object(Matching, "Kyoto/Streams/COutputStream.cpp"),
            Object(Matching, "Kyoto/Streams/CZipInputStream.cpp"),
            Object(Matching, "Kyoto/Streams/CZipOutputStream.cpp"),
            Object(Matching, "Kyoto/Streams/CZipSupport.cpp"),
            Object(Matching, "Kyoto/CFactoryStore.cpp"),
            Object(Matching, "Kyoto/CObjectReference.cpp"),
            Object(NonMatching, "Kyoto/CSimplePool.cpp"),
            Object(Matching, "Kyoto/CToken.cpp"),
            Object(Matching, "Kyoto/IObj.cpp"),
        ],
    ),
    # TODO: Merge back into Kyoto
    {
        "lib": "zlib",
        "mw_version": "GC/1.3.2",
        "cflags": cflags_runtime,
        "host": False,
        "progress_category": "third_party",
        "shift_jis": False,
        "objects": [
            Object(Matching, "Kyoto/zlib/adler32.c"),
            Object(Matching, "Kyoto/zlib/deflate.c"),
            Object(Matching, "Kyoto/zlib/infblock.c"),
            Object(Matching, "Kyoto/zlib/infcodes.c"),
            Object(Matching, "Kyoto/zlib/inffast.c"),
            Object(Matching, "Kyoto/zlib/inflate.c"),
            Object(Matching, "Kyoto/zlib/inftrees.c"),
            Object(Matching, "Kyoto/zlib/infutil.c"),
            Object(Matching, "Kyoto/zlib/trees.c"),
            Object(Matching, "Kyoto/zlib/zutil.c"),
        ],
    },
    # TODO: Merge this with zlib and Kyoto1
    RetroLib(
        "Kyoto2",
        "core",
        [
            Object(Matching, "Kyoto/CARAMManager.cpp"),
            Object(NonMatching, "Kyoto/Math/CFrustumPlanes.cpp"),
            Object(NonMatching, "Kyoto/Graphics/CCubeMaterial.cpp"),
            Object(Matching, "Kyoto/Graphics/CCubeSurface.cpp"),
            Object(Matching, "Kyoto/Animation/CCharAnimTime.cpp"),
            Object(Matching, "Kyoto/Animation/CSegIdList.cpp"),
            Object(Matching, "Kyoto/Input/CFinalInput.cpp"),
            Object(Matching, "Kyoto/Graphics/CColor.cpp"),
            Object(NonMatching, "Kyoto/Audio/DolphinCAudioGroupSet.cpp"),
            Object(NonMatching, "Kyoto/Audio/DolphinCAudioSys.cpp"),
            Object(NonMatching, "Kyoto/DolphinCMemoryCardSys.cpp"),
            Object(Matching, "Kyoto/Input/DolphinIController.cpp"),
            Object(Matching, "Kyoto/Input/CDolphinController.cpp"),
            Object(Matching, "Kyoto/DolphinCDvdFile.cpp"),
            Object(NonMatching, "Kyoto/Alloc/CMediumAllocPool.cpp"),
            Object(Matching, "Kyoto/Alloc/CSmallAllocPool.cpp"),
            Object(NonMatching, "Kyoto/Alloc/CGameAllocator.cpp"),
            Object(NonMatching, "Kyoto/Animation/DolphinCSkinnedModel.cpp"),
            Object(NonMatching, "Kyoto/Animation/DolphinCSkinRules.cpp"),
            Object(NonMatching, "Kyoto/Animation/DolphinCVirtualBone.cpp"),
            Object(NonMatching, "Kyoto/Graphics/DolphinCModel.cpp"),
            Object(Matching, "Kyoto/Text/CStringTable.cpp"),
            Object(NonMatching, "Kyoto/Particles/CEmitterElement.cpp"),
            Object(Matching, "Kyoto/Particles/CEffectComponent.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleData.cpp"),
            Object(NonMatching, "Kyoto/Animation/CVertexMorphEffect.cpp"),
            Object(NonMatching, "Kyoto/Animation/CSkinnedModelWithAvgNormals.cpp"),
            Object(Matching, "Kyoto/CTimeProvider.cpp"),
            Object(Matching, "Kyoto/CARAMToken.cpp"),
            Object(Matching, "Kyoto/Audio/CMidiManager.cpp"),
            Object(Matching, "Kyoto/Text/CFontImageDef.cpp"),
            Object(NonMatching, "Kyoto/Text/CImageInstruction.cpp"),
            Object(NonMatching, "Kyoto/Text/CTextRenderBuffer.cpp"),
            Object(NonMatching, "Kyoto/Graphics/CCubeMoviePlayer.cpp"),
            Object(NonMatching, "Kyoto/Animation/CAdditiveAnimPlayback.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleElectricDataFactory.cpp"),
            Object(NonMatching, "Kyoto/Particles/CParticleElectric.cpp"),
            Object(Matching, "Kyoto/Graphics/DolphinCColor.cpp"),
            Object(NonMatching, "Kyoto/Audio/CDSPStreamManager.cpp"),
            Object(Matching, "Kyoto/CDependencyGroup.cpp"),
            Object(NonMatching, "Kyoto/Audio/CStreamAudioManager.cpp"),
            Object(Matching, "Kyoto/Animation/CHalfTransition.cpp"),
            Object(NonMatching, "Kyoto/Particles/CElectricDescription.cpp"),
            Object(NonMatching, "Kyoto/Particles/CSwooshDescription.cpp"),
            Object(NonMatching, "Kyoto/Particles/CGenDescription.cpp"),
            Object(NonMatching, "Kyoto/CPakFile.cpp"),
            Object(NonMatching, "Kyoto/Animation/CPoseAsTransformsVariableSize.cpp"),
            Object(NonMatching, "Kyoto/Input/CRumbleVoice.cpp"),
            Object(Matching, "Kyoto/Input/RumbleAdsr.cpp"),
            Object(Matching, "Kyoto/Input/CRumbleGenerator.cpp"),
            Object(NonMatching, "Kyoto/Audio/CDSPStream.cpp"),
            Object(Matching, "Kyoto/Audio/g721.cpp"),
            Object(NonMatching, "Kyoto/Audio/CStaticAudioPlayer.cpp"),
            Object(NonMatching, "Kyoto/CFrameDelayedKiller.cpp"),
        ],
    ),
    DolphinLib(
        "ai",
        [
            Object(Matching, "Dolphin/ai.c"),
        ],
    ),
    DolphinLib(
        "ar",
        [
            Object(Matching, "Dolphin/ar/ar.c"),
            Object(Matching, "Dolphin/ar/arq.c"),
        ],
    ),
    DolphinLib(
        "base",
        [
            Object(Matching, "Dolphin/PPCArch.c"),
        ],
    ),
    DolphinLib(
        "db",
        [
            Object(Matching, "Dolphin/db.c"),
        ],
    ),
    DolphinLib(
        "dsp",
        [
            Object(Matching, "Dolphin/dsp/dsp.c"),
            Object(Matching, "Dolphin/dsp/dsp_debug.c"),
            Object(Matching, "Dolphin/dsp/dsp_task.c"),
        ],
    ),
    DolphinLib(
        "dvd",
        [
            Object(Matching, "Dolphin/dvd/dvdlow.c"),
            Object(Matching, "Dolphin/dvd/dvdfs.c"),
            Object(Matching, "Dolphin/dvd/dvd.c"),
            Object(Matching, "Dolphin/dvd/dvdqueue.c"),
            Object(Matching, "Dolphin/dvd/dvderror.c"),
            Object(Matching, "Dolphin/dvd/dvdidutils.c"),
            Object(Matching, "Dolphin/dvd/dvdfatal.c"),
            Object(Matching, "Dolphin/dvd/fstload.c"),
        ],
    ),
    DolphinLib(
        "gx",
        [
            Object(NonMatching, "Dolphin/gx/GXInit.c"),
            Object(NonMatching, "Dolphin/gx/GXFifo.c"),
            Object(NonMatching, "Dolphin/gx/GXAttr.c"),
            Object(NonMatching, "Dolphin/gx/GXMisc.c"),
            Object(NonMatching, "Dolphin/gx/GXGeometry.c"),
            Object(NonMatching, "Dolphin/gx/GXFrameBuf.c"),
            Object(NonMatching, "Dolphin/gx/GXLight.c"),
            Object(NonMatching, "Dolphin/gx/GXTexture.c"),
            Object(NonMatching, "Dolphin/gx/GXBump.c"),
            Object(NonMatching, "Dolphin/gx/GXTev.c"),
            Object(NonMatching, "Dolphin/gx/GXPixel.c"),
            Object(NonMatching, "Dolphin/gx/GXStubs.c"),
            Object(NonMatching, "Dolphin/gx/GXDisplayList.c"),
            Object(NonMatching, "Dolphin/gx/GXTransform.c"),
            Object(NonMatching, "Dolphin/gx/GXPerf.c"),
        ],
    ),
    DolphinLib(
        "mtx",
        [
            Object(
                Matching, "Dolphin/mtx/mtx.c", cflags=[*cflags_base, "-fp_contract off"]
            ),
            Object(NonMatching, "Dolphin/mtx/mtx44vec.c"),
            Object(NonMatching, "Dolphin/mtx/mtx44.c"),
            Object(NonMatching, "Dolphin/mtx/vec.c"),
            Object(NonMatching, "Dolphin/mtx/psmtx.c"),
        ],
    ),
    DolphinLib(
        "os",
        [
            Object(Matching, "Dolphin/os/__start.c"),
            Object(Matching, "Dolphin/os/OS.c"),
            Object(Matching, "Dolphin/os/OSAlarm.c"),
            Object(Matching, "Dolphin/os/OSArena.c"),
            Object(Matching, "Dolphin/os/OSAudioSystem.c"),
            Object(Matching, "Dolphin/os/OSCache.c"),
            Object(Matching, "Dolphin/os/OSContext.c"),
            Object(Matching, "Dolphin/os/OSError.c"),
            Object(NonMatching, "Dolphin/os/OSFatal.c"),
            Object(NonMatching, "Dolphin/os/OSFont.c"),
            Object(Matching, "Dolphin/os/OSInterrupt.c"),
            Object(Matching, "Dolphin/os/OSLink.c"),
            Object(Matching, "Dolphin/os/OSMessage.c"),
            Object(Matching, "Dolphin/os/OSMemory.c"),
            Object(Matching, "Dolphin/os/OSMutex.c"),
            Object(NonMatching, "Dolphin/os/OSReboot.c"),
            Object(Matching, "Dolphin/os/OSReset.c"),
            Object(Matching, "Dolphin/os/OSResetSW.c"),
            Object(Matching, "Dolphin/os/OSRtc.c"),
            Object(Matching, "Dolphin/os/OSSync.c"),
            Object(Matching, "Dolphin/os/OSThread.c"),
            Object(Matching, "Dolphin/os/OSTime.c"),
            Object(Matching, "Dolphin/os/__ppc_eabi_init.cpp"),
        ],
    ),
    DolphinLib(
        "pad",
        [
            Object(Matching, "Dolphin/pad/PadClamp.c"),
            Object(Matching, "Dolphin/pad/pad.c"),
        ],
    ),
    DolphinLib(
        "vi",
        [
            Object(Matching, "Dolphin/vi.c"),
        ],
    ),
    {
        "lib": "MSL_C.PPCEABI.bare.H",
        "mw_version": "GC/1.3",
        "cflags": cflags_runtime,
        "host": False,
        "progress_category": "sdk",
        "shift_jis": False,
        "objects": [
            Object(Matching, "Runtime/__mem.c"),
            Object(Matching, "Runtime/__va_arg.c"),
            Object(Matching, "Runtime/global_destructor_chain.c"),
            Object(Matching, "Runtime/CPlusLibPPC.cpp"),
            Object(Matching, "Runtime/NMWException.cpp"),
            Object(Matching, "Runtime/ptmf.c"),
            Object(Matching, "Runtime/runtime.c"),
            Object(Matching, "Runtime/__init_cpp_exceptions.cpp"),
            Object(Matching, "Runtime/Gecko_ExceptionPPC.cpp"),
            Object(Matching, "Runtime/abort_exit.c"),
            Object(NonMatching, "Runtime/alloc.c"),
            Object(Matching, "Runtime/ansi_files.c"),
            Object(Matching, "Runtime/ansi_fp.c"),
            Object(Matching, "Runtime/arith.c"),
            Object(Matching, "Runtime/buffer_io.c"),
            Object(Matching, "Runtime/ctype.c"),
            Object(Matching, "Runtime/locale.c"),
            Object(Matching, "Runtime/direct_io.c"),
            Object(Matching, "Runtime/file_io.c"),
            Object(Matching, "Runtime/errno.c"),
            Object(Matching, "Runtime/FILE_POS.c"),
            Object(Matching, "Runtime/mbstring.c"),
            Object(Matching, "Runtime/mem.c"),
            Object(Matching, "Runtime/mem_funcs.c"),
            Object(Matching, "Runtime/misc_io.c"),
            Object(NonMatching, "Runtime/printf.c"),
            Object(Matching, "Runtime/qsort.c"),
            Object(Matching, "Runtime/rand.c"),
            Object(Matching, "Runtime/sscanf.c"),
            Object(Matching, "Runtime/string.c"),
            Object(Matching, "Runtime/float.c"),
            Object(Matching, "Runtime/strtold.c"),
            Object(Matching, "Runtime/uart_console_io.c"),
            Object(Matching, "Runtime/wchar_io.c"),
            Object(Matching, "Runtime/e_acos.c"),
            Object(Matching, "Runtime/e_asin.c"),
            Object(Matching, "Runtime/e_atan2.c"),
            Object(Matching, "Runtime/e_exp.c"),  
            Object(Matching, "Runtime/e_fmod.c"),
            Object(Matching, "Runtime/e_log.c"),
            Object(Matching, "Runtime/e_pow.c"),  
            Object(Matching, "Runtime/e_rem_pio2.c"),
            Object(Matching, "Runtime/k_cos.c"),
            Object(Matching, "Runtime/k_rem_pio2.c"),
            Object(Matching, "Runtime/k_sin.c"),
            Object(Matching, "Runtime/k_tan.c"),
            Object(Matching, "Runtime/s_atan.c"), 
            Object(Matching, "Runtime/s_copysign.c"),
            Object(Matching, "Runtime/s_cos.c"),
            Object(Matching, "Runtime/s_floor.c"),
            Object(Matching, "Runtime/s_frexp.c"),
            Object(Matching, "Runtime/s_ldexp.c"),
            Object(Matching, "Runtime/s_modf.c"),
            Object(Matching, "Runtime/s_nextafter.c"),
            Object(Matching, "Runtime/s_sin.c"),
            Object(Matching, "Runtime/s_tan.c"),
            Object(Matching, "Runtime/w_acos.c"),
            Object(Matching, "Runtime/w_asin.c"),
            Object(Matching, "Runtime/w_atan2.c"),
            Object(Matching, "Runtime/w_exp.c"),
            Object(Matching, "Runtime/w_fmod.c"),
            Object(Matching, "Runtime/w_log.c"),
            Object(Matching, "Runtime/w_pow.c"),
            Object(Matching, "Runtime/math_ppc.c"),
        ],
    },
    MusyX(
        # debug=True,
        # mw_version="GC/1.2.5",
        # major=1,
        # minor=5,
        # patch=3,
        objects=[
            Object(Matching, "musyx/runtime/seq.c"),
            Object(Matching, "musyx/runtime/synth.c"),
            Object(Matching, "musyx/runtime/seq_api.c"),
            Object(Matching, "musyx/runtime/snd_synthapi.c"),
            Object(Matching, "musyx/runtime/stream.c"),
            Object(Matching, "musyx/runtime/synthdata.c"),
            Object(Matching, "musyx/runtime/synthmacros.c"),
            Object(Matching, "musyx/runtime/synthvoice.c"),
            Object(Matching, "musyx/runtime/synth_ac.c"),
            Object(Matching, "musyx/runtime/synth_adsr.c"),
            Object(Matching, "musyx/runtime/synth_vsamples.c"),
            Object(Matching, "musyx/runtime/synth_dbtab.c"),
            Object(Matching, "musyx/runtime/s_data.c"),
            Object(NonMatching, "musyx/runtime/hw_dspctrl.c"),
            Object(Matching, "musyx/runtime/hw_volconv.c"),
            Object(Matching, "musyx/runtime/snd3d.c"),
            Object(Matching, "musyx/runtime/snd_init.c"),
            Object(Matching, "musyx/runtime/snd_math.c"),
            Object(NonMatching, "musyx/runtime/snd_midictrl.c"),
            Object(Matching, "musyx/runtime/snd_service.c"),
            Object(Matching, "musyx/runtime/hardware.c"),
            Object(Matching, "musyx/runtime/hw_aramdma.c"),
            Object(Matching, "musyx/runtime/dsp_import.c"),
            Object(Matching, "musyx/runtime/hw_dolphin.c"),
            Object(Matching, "musyx/runtime/hw_memory.c"),
            Object(Matching, "musyx/runtime/hw_lib_dummy.c"),
            Object(Matching, "musyx/runtime/CheapReverb/creverb_fx.c"),
            Object(Matching, "musyx/runtime/CheapReverb/creverb.c"),
            Object(Matching, "musyx/runtime/StdReverb/reverb_fx.c"),
            Object(Matching, "musyx/runtime/StdReverb/reverb.c"),
            Object(Matching, "musyx/runtime/Delay/delay_fx.c"),
            Object(Matching, "musyx/runtime/Chorus/chorus_fx.c"),
            Object(Matching, "musyx/runtime/profile.c"),
        ],
    ),
    DolphinLib(
        "dtk",
        [
            Object(Matching, "Dolphin/dtk.c"),
        ],
    ),
    DolphinLib(
        "card",
        [
            Object(Matching, "Dolphin/card/CARDBios.c"),
            Object(Matching, "Dolphin/card/CARDUnlock.c"),
            Object(Matching, "Dolphin/card/CARDRdwr.c"),
            Object(Matching, "Dolphin/card/CARDBlock.c"),
            Object(Matching, "Dolphin/card/CARDDir.c"),
            Object(Matching, "Dolphin/card/CARDCheck.c"),
            Object(Matching, "Dolphin/card/CARDMount.c"),
            Object(Matching, "Dolphin/card/CARDFormat.c"),
            Object(Matching, "Dolphin/card/CARDOpen.c"),
            Object(Matching, "Dolphin/card/CARDCreate.c"),
            Object(Matching, "Dolphin/card/CARDRead.c"),
            Object(Matching, "Dolphin/card/CARDWrite.c"),
            Object(Matching, "Dolphin/card/CARDDelete.c"),
            Object(Matching, "Dolphin/card/CARDStat.c"),
            Object(Matching, "Dolphin/card/CARDRename.c"),
            Object(Matching, "Dolphin/card/CARDNet.c"),
        ],
    ),
    DolphinLib(
        "si",
        [
            Object(Matching, "Dolphin/si/SIBios.c"),
            Object(Matching, "Dolphin/si/SISamplingRate.c"),
        ],
    ),
    DolphinLib(
        "exi",
        [
            Object(Matching, "Dolphin/exi/EXIBios.c"),
            Object(Matching, "Dolphin/exi/EXIUart.c"),
        ],
    ),
    DolphinLib(
        "thp",
        [
            Object(Matching, "Dolphin/thp/THPDec.c"),
            Object(NonMatching, "Dolphin/thp/THPAudio.c"),
        ],
    ),
    DolphinLib(
        "gba",
        [
            Object(Matching, "Dolphin/GBA/GBA.c"),
            Object(Matching, "Dolphin/GBA/GBAGetProcessStatus.c"),
            Object(Matching, "Dolphin/GBA/GBAJoyBoot.c"),
            Object(Matching, "Dolphin/GBA/GBARead.c"),
            Object(Matching, "Dolphin/GBA/GBAWrite.c"),
            Object(Matching, "Dolphin/GBA/GBAXfer.c"),
            Object(Matching, "Dolphin/GBA/GBAKey.c"),
        ],
    ),
    Rel(
        "NESemuP",
        [
            Object(
                Matching,
                "NESemu/modwrapper.c",
            ),
        ],
    ),
]

# Disable missing return type warnings for incomplete objects
for lib in config.libs:
    for obj in lib["objects"]:
        if not obj.completed:
            obj.options["extra_clang_flags"].append("-Wno-return-type")


# Optional extra categories for progress tracking
config.progress_categories = [
    ProgressCategory("game", "Game"),
    ProgressCategory("core", "Core Engine (Kyoto)"),
    ProgressCategory("sdk", "SDK"),
    ProgressCategory("third_party", "Third Party"),
]
config.progress_all = False
config.progress_each_module = args.verbose
config.progress_modules = False
config.progress_use_fancy = True
config.progress_code_fancy_frac = 1499
config.progress_code_fancy_item = "Energy"
config.progress_data_fancy_frac = 250
config.progress_data_fancy_item = "Missiles"

if args.mode == "configure":
    # Write build.ninja and objdiff.json
    generate_build(config)
elif args.mode == "progress":
    # Print progress and write progress.json
    calculate_progress(config)
else:
    sys.exit("Unknown mode: " + args.mode)
