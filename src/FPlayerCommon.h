/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include "FEntity.h"

#include <memory>

//#include "FTeam.h"
// class FTeam;

struct FPlayerCommon;
using FPlayerCommonPtr = std::shared_ptr<FPlayerCommon>;
struct FPlayerCommon // Flyweight pattern: commom property for Player.h
{
        std::vector<sdf::ElementPtr> animations;
        // TMP  by using gz ptr
        // gazebo::physics::ModelPtr ptrGzBall{nullptr};
        // other list ...

        // Supervise   It has the list of players with its distance to ball
        // std::map listPlayersTeamA_BallDistance(Player,distanceBall)
        // std::map<FPlayerCommonPtr,double> mapPlayers_BallDistance;
        // PlayerPtr playerAClosestToBall(listPlayersTeamA_BallDistance)        get the player with the minimun distance
};