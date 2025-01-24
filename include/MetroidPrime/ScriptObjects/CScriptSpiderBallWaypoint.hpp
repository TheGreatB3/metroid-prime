#ifndef _CSCRIPTSPIDERBALLWAYPOINT
#define _CSCRIPTSPIDERBALLWAYPOINT

#include "MetroidPrime/CActor.hpp"

class CScriptSpiderBallWaypoint : public CActor {
public:
  void ClearWaypoints();
  void BuildWaypointListAndBounds(CStateManager& mgr);
  void GetClosestPointAlongWaypoints(CStateManager& mgr, const CVector3f& ballCenter,
                                     float maxPointToBallDist,
                                     const CScriptSpiderBallWaypoint** closestWaypoints,
                                     CVector3f& closestPoint, CVector3f& deltaBetweenPoints,
                                     float deltaBetweenInterpDist,
                                     CVector3f& interpDeltaBetweenPoints) const;

private:
  enum ECheckActiveWaypoint { kCAW_Check, kCAW_SkipCheck };

  uint xe8_;
  rstl::vector< TUniqueId > xec_waypoints;
  rstl::optional_object< CAABox > xfc_aabox;
};

#endif // _CSCRIPTSPIDERBALLWAYPOINT
