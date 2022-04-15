#ifndef _CGAMESTATE_HPP
#define _CGAMESTATE_HPP

#include "types.h"

#include "rstl/rc_ptr.hpp"
#include "rstl/reserved_vector.hpp"
#include "rstl/vector.hpp"

#include "CGameOptions.hpp"
#include "CGameState.hpp"
#include "CHintOptions.hpp"
#include "CPlayerState.hpp"
#include "CSystemOptions.hpp"
#include "CWorldState.hpp"
#include "CWorldTransManager.hpp"
#include "TGameTypes.hpp"

class CGameState {
public:
  CGameState();
  CGameState(CInputStream& in, int saveIdx);

  rstl::rc_ptr< CPlayerState >& PlayerState();
  CAssetId CurrentWorldAssetId();

  CSystemOptions& SystemOptions() { return xa8_systemOptions; }
  CGameOptions& GameOptions() { return x17c_gameOptions; }
  CHintOptions& HintOptions() { return x1f8_hintOptions; }
  u32& SaveIdx() { return x20c_saveIdx; }
  u64& CardSerial() { return x210_cardSerial; }
  rstl::vector< u8 >& BackupBuf() { return x218_backupBuf; }

private:
  rstl::reserved_vector< bool, 128 > x0_;
  CAssetId x84_mlvlId;
  rstl::vector< CWorldState > x88_worldStates;
  rstl::rc_ptr< CPlayerState > x98_playerState;
  rstl::rc_ptr< CWorldTransManager > x9c_transManager;
  f64 xa0_playTime;
  CSystemOptions xa8_systemOptions;
  CGameOptions x17c_gameOptions;
  CHintOptions x1f8_hintOptions;
  u32 x20c_saveIdx;
  u64 x210_cardSerial;
  rstl::vector< u8 > x218_backupBuf;
  bool x228_24_hardMode : 1;
  bool x228_25_initPowerupsAtFirstSpawn : 1;
};

extern CGameState* gpGameState;

#endif
