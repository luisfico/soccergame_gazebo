/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FTeam.h"
//#include "FScene.h"
#include "FPlayer.h"

FTeam::FTeam(const std::string &_m_name) : FScene(_m_name)
{
  std::cout << "Building FTeam " << this->Name() << " level: " << typeid(*this).name() << " \n";

  // Build
  std::uint8_t playersTotal= this->Size() ; //va de la mano con static_cast<uint8_t>()
  vecPlayers_BallDistance.reserve(playersTotal); //to avoids weird behaviur of std::vector
  vecPlayers_BallDistanceSort.reserve(playersTotal); //to avoids weird behaviur of std::vector
  // init empty with size3
  std::vector<std::pair<FPlayerPtr, double>> vecPlayers_BallDistance0(3, std::make_pair(nullptr, -1));
  // std::map<std::string, int> m { {"CPU", 10}, {"GPU", 15}, {"RAM", 20}, };

  vecPlayers_BallDistance = vecPlayers_BallDistance0;
  // for(int i=1,i<=3,i++)
  //{vecPlayers_BallDistance.push_back(std::pair<nullptr, 0>);}
}

FTeam::~FTeam()
{
  std::cout << "DestroyingD " << this->Name() << " level: " << typeid(*this).name() << " \n";
}

/**
 * The FScene executes its primary logic in a particular way. It traverses
 * recursively through all its children, collecting and summing their results.
 * Since the FScene's children pass these calls to their children and so
 * forth, the whole object tree is traversed as a result.
 */
std::string FTeam::Operation() const
{
  std::string result;
  for (const std::weak_ptr<FEntity> weakPtr : children_)
  {
    // if (c == children_.back()){ result += c->Operation();}
    // else
    if (const auto &ptr = weakPtr.lock())
    {
      result += ptr->Operation() + "+";
    }
  }
  // return "Branch(" + result + ")";
  return m_name + "(" + result + ")";
}

void FTeam::testLevelFTeam() const
{
  std::cout << "testLevelFTeam \n";
}


// void FTeam::SetParentPlayers() //unused
// {
//   for (const std::weak_ptr<FEntity> weakPtr : this->children_) // GetEntities
//   {
//     if (const auto &ptr = weakPtr.lock())
//     {
//       FPlayerPtr ptrPLayer = std::dynamic_pointer_cast<FPlayer>(ptr); // downcast inheritance
//       ptrPLayer->mTeam = this->shared_from_this();                    // SetTeam(this)
//     }
//   }
// }



void FTeam::SetVecPlayers_BallDistance()
{
  int ind=0;
  for (const std::weak_ptr<FEntity> weakPtr : this->children_) // GetEntities
  {
    if (const auto &ptr = weakPtr.lock())
    {
      FPlayerPtr ptrPLayer = std::dynamic_pointer_cast<FPlayer>(ptr); // downcast inheritance
      this->vecPlayers_BallDistance[ind]= std::make_pair(ptrPLayer,ptrPLayer->distBallPlayer) ;                    // SetTeam(this)
    }
    ind++;
  }

  vecPlayers_BallDistanceSort=vecPlayers_BallDistance; //copy and the sort
  //std::sort(vecPlayers_BallDistanceSort.begin(),vecPlayers_BallDistanceSort.end());
  std::sort(vecPlayers_BallDistanceSort.begin(),vecPlayers_BallDistanceSort.end(),criteriusOrderAZ);
  
  //std::reverse(vecPlayers_BallDistanceSort.begin(),vecPlayers_BallDistanceSort.end());

}

  //NOTE: FGame is an intermediare between FPlayer to access to FTeam data(example vecPlayers_BallDistanceSort)
  //      FTeam does not need of intermediare to acces to FPlayers data 
  //      So FTeam can be its own intermediare, so EnablePlayerMovement() is performed by FTeam
  //Enable the movement of the player closest to the ball
  void  FTeam::EnablePlayerMovement() //In this case: FGame access to FTeam::vecPlayers_BallDistanceSort
  {
    for (auto &val : this->vecPlayers_BallDistanceSort)
    {
      val.first->mModeManual=false;//and then it selects the FPlayer 
    }
    this->vecPlayers_BallDistanceSort[0].first->mModeManual=true; //only can moves the player closest to the ball
  }

//__________new

