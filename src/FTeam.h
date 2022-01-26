/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FScene.h"
#include "FPlayer.h"

#include <memory>
class FTeam;
using FTeamPtr = std::shared_ptr<FTeam>;

class FPlayer;
class FTeam : public FScene //, public std::enable_shared_from_this<FTeam>
{
        // members:
        //-Stratety type formation: 442,433,541,etc
        //-nameTeam, uniformWear, etc

public:
  //FEntityPtr playerPtr;//tmp playerPtr, children_ in same scope
  FEntityPtr playerPtr1,playerPtr2,playerPtr3;  //TODO: debe estar aqui???
  FEntityPtr playerPtr1b,playerPtr2b,playerPtr3b; 

        FTeam(const std::string &_m_name);
        virtual ~FTeam();

        virtual std::string Operation() const override;

        virtual void testLevelFTeam() const;

        //virtual void SetParentPlayers(); // unused. FPlayer uses GetParent() method

        //__________new


        virtual void SetVecPlayers_BallDistance();
        virtual void EnablePlayerMovement();

        //  Debe ser el equipo que inicie la poses de los jugadores de acuerdo a sus posiciones
        // virtual void InitPlayerPoses(formationStratey442)

        /*
                virtual void Update() override // based on show
                {
                        //createMapDistancesBall; //once
                        //UpdateMapDistancesBall(); //sort map
                        //playerAClosestToBall(); //ko
                        // pour chaque élément du vecteur, on l'affiche
                        for (auto &element : GetEntities())
                        {
                                element->Update();
                        }
                }
        */
        //std::map<FPlayerPtr, double> mapPlayers_BallDistance; // UNUSED
        std::vector<std::pair<FPlayerPtr, double>> vecPlayers_BallDistance;
        std::vector<std::pair<FPlayerPtr, double>> vecPlayers_BallDistanceSort; // USED!!!

        struct opOrderAZ //order AZ by distace ball (double)
        {
                bool operator()(std::pair<FPlayerPtr, double> i, std::pair<FPlayerPtr, double> j)
                {return (i.second < j.second); }
        } criteriusOrderAZ;
};