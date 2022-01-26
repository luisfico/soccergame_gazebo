/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

/*
NOTE: Spawn actor1(static) and actor2(moving with SetLinearVel method)
TODO: add actors+collision attached

Class factoryWorldPlugin (called by main gz_server) is equivalent to Class CGame (called by main )
*/

//#include "ControllerKeyboard.hpp"  //DISABLE_KEYBOARD
//#include "FEntity.h"   //HOW TO ADD in the project

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "models.hh"
//-----------------------TrajectoryActorPlugin.hh----------ini
#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

//-----------------------TrajectoryActorPlugin.hh----------end

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

// GZtransport
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "MsgCommand.hpp"
#include "FEntity.h"
#include "FScene.h"
#include "FPlayer.h"
#include "FTeam.h"
#include "FBall.h"
#include "FStadium.h"

#include <memory>

// Global varaible indirectly called by linking using EXTERN keyword
// extern gazebo::physics::WorldPtr& _mWorldPtr;

// Global varaible directly called by includes
// gazebo::physics::WorldPtr _mWorldPtr{nullptr};
// sdf::ElementPtr mSdfPtr{nullptr};
/*
MsgCommandPtr msgCmd{nullptr};
gazebo::physics::WorldPtr _mWorldPtr = boost::make_shared<gazebo::physics::World>();
sdf::ElementPtr mSdfPtr = std::make_shared<sdf::Element>(); // global ptr
*/

using namespace gazebo;
// using namespace servicesim;

// NOTE: Scene is (worldGz_ptr+name,state+.... ,      Player is modelGz_ptr+name,state..

// TODO:class game, entity, scene, player,    composite pattern
struct FGame // TODO a class, with accesers get,set
{
  FScenePtr _mMatch = std::make_shared<FScene>("matchAB/");
  FEntityPtr _mStadium = std::make_shared<FStadium>("Stadio", "Paris");
  FEntityPtr _mTeamA = std::make_shared<FTeam>("teamA/");
  FEntityPtr _mTeamB = std::make_shared<FTeam>("teamB/");
  FEntityPtr _mBall = std::make_shared<FBall>("bola1", "nike");
  FEntityPtr _playerPtr1 = std::make_shared<FPlayer>("player_1", FPlayer::Position::DEFENSE);
  FEntityPtr _playerPtr2 = std::make_shared<FPlayer>("player_2", FPlayer::Position::MIDFIELDER);
  FEntityPtr _playerPtr3 = std::make_shared<FPlayer>("player_3", FPlayer::Position::FORWARD);
  FEntityPtr _playerPtr1b = std::make_shared<FPlayer>("player_1b", FPlayer::Position::DEFENSE);
  FEntityPtr _playerPtr2b = std::make_shared<FPlayer>("player_2b", FPlayer::Position::MIDFIELDER);
  FEntityPtr _playerPtr3b = std::make_shared<FPlayer>("player_3b", FPlayer::Position::FORWARD);
  

  /*
    FStadiumPtr mStadium = std::dynamic_pointer_cast<FEntity>(_mStadium);
    FBallPtr mBall = std::dynamic_pointer_cast<FEntity>(_mBall);
    FTeamPtr mTeamA = std::dynamic_pointer_cast<FEntity>(_mTeamA);
  */
  FStadiumPtr mStadium;
  FBallPtr mBall;
  FTeamPtr mTeamA,mTeamB;
  FPlayerPtr playerPtr1,playerPtr2,playerPtr3;
  FPlayerPtr playerPtr1b,playerPtr2b,playerPtr3b;
  int totalGzObjects = 0;

  void TestSetParentPlayers()
  {std::cout << "\n TestSetParentPlayers ------- \n";        
    for (const std::weak_ptr<FEntity> weakPtr : this->mTeamA->children_) // GetEntities
    {
      if (const auto &ptr = weakPtr.lock())
      {
        FPlayerPtr ptrPLayer = std::dynamic_pointer_cast<FPlayer>(ptr); // downcast inheritance
        FTeamPtr ptrTeam= std::dynamic_pointer_cast<FTeam>(ptrPLayer->GetParent() );            
        ptrPLayer->mTeam = ptrTeam ;                    // SetTeam(this)

        std::cout << "\n test playerTTTTT------- : " << ptrPLayer->Name() << " level: " << typeid(*ptrPLayer).name()<<"\n";
        std::cout << "\n test ptrFTeamTTT------- : " << ptrTeam->Name() << " level: " << typeid(*ptrTeam).name()<<"\n";
        std::cout << "\n test ptrFTeam   ------- : " << ptrPLayer->mTeam->Name() << " level: " << typeid(*ptrPLayer->mTeam).name()<<"\n";
        
      }
    }

    for (const std::weak_ptr<FEntity> weakPtr : this->mTeamB->children_) // GetEntities
    {
      if (const auto &ptr = weakPtr.lock())
      {
        FPlayerPtr ptrPLayer = std::dynamic_pointer_cast<FPlayer>(ptr); // downcast inheritance
        FTeamPtr ptrTeam= std::dynamic_pointer_cast<FTeam>(ptrPLayer->GetParent() );            
        ptrPLayer->mTeam = ptrTeam ;                    // SetTeam(this)

        std::cout << "\n test playerTTTTT------- : " << ptrPLayer->Name() << " level: " << typeid(*ptrPLayer).name()<<"\n";
        std::cout << "\n test ptrFTeamTTT------- : " << ptrTeam->Name() << " level: " << typeid(*ptrTeam).name()<<"\n";
        std::cout << "\n test ptrFTeam   ------- : " << ptrPLayer->mTeam->Name() << " level: " << typeid(*ptrPLayer->mTeam).name()<<"\n";
        
      }
    }
  }
  void DebugSetParentPlayersShow()
  {std::cout << "\n DebugSetParentPlayersShow ------- \n";
    for (const std::weak_ptr<FEntity> weakPtr : this->mTeamA->children_) // GetEntities
    {
      if (const auto &ptr = weakPtr.lock())
      {
        FPlayerPtr ptrPLayer = std::dynamic_pointer_cast<FPlayer>(ptr); // downcast inheritance
        
        std::cout << "\n test playerTTTTT------- : " << ptrPLayer->Name() << " level: " << typeid(*ptrPLayer).name()<<"\n";
        std::cout << "\n test ptrFTeamTTT------- : " << ptrPLayer->mTeam->Name() << " level: " << typeid(*ptrPLayer->mTeam).name()<<"\n";
        ptrPLayer->mTeam->testLevelFEntity();  //ko because  ptrFEntity  is  in FScene* level     why  typeid  says   FScene*????
        ptrPLayer->mTeam->testLevelFScene();  //ko because  ptrFEntity  is  in FScene* level     why  typeid  says   FScene*????
        ptrPLayer->mTeam->testLevelFTeam();
      }
    }
  }

  FGame(const std::string &nameFGame) : mNameFGame{nameFGame}
  {
    pdu = std::make_shared<FPlayerCommon>();

    std::cout << "FGame: DEBUG &worldPtr : " << &_mWorldPtr << std::endl;
    // mTeamA = new CScene();
    // FabricaTeamA f;
    // mTeamA->initialize(f);  // insert player in the renderer gz   and  addFEntity in the scene

     


//-------- TREE CREATION: creation of the entities tree (architecture composite design pattern)
    _mMatch->Add(_mStadium);
    _mMatch->Add(_mBall);
    _mMatch->Add(_mTeamA);    
    _mMatch->Add(_mTeamB);
    _mTeamA->Add(_playerPtr1);_mTeamA->Add(_playerPtr2);_mTeamA->Add(_playerPtr3);
    _mTeamB->Add(_playerPtr1b);_mTeamB->Add(_playerPtr2b);_mTeamB->Add(_playerPtr3b);
    
    // advantage of tree composite  (without collisions)
    //totalGzObjects = _mMatch->Size(); //ex total entities without colssions  
    //Tmp: because box collision models are not entities
    totalGzObjects=_mStadium->Size()+_mBall->Size()+(_mTeamA->Size()+_mTeamB->Size())*2;   //player entities: x2: players+collision box model
    std::cout << _mMatch->Operation(); // for debug  for show the tree composite
    


//-------- RENDERER CREATION: creation of entities in the renderer gazebo
    // Tree downcast to use derivated level methods as createPlayerInRendererGz() ------------
    mStadium = std::dynamic_pointer_cast<FStadium>(_mStadium);
    mBall = std::dynamic_pointer_cast<FBall>(_mBall);
    mTeamA = std::dynamic_pointer_cast<FTeam>(_mTeamA);
    mTeamB = std::dynamic_pointer_cast<FTeam>(_mTeamB);
    playerPtr1 = std::dynamic_pointer_cast<FPlayer>(_playerPtr1);
    playerPtr2 = std::dynamic_pointer_cast<FPlayer>(_playerPtr2);
    playerPtr3 = std::dynamic_pointer_cast<FPlayer>(_playerPtr3);
    playerPtr1b = std::dynamic_pointer_cast<FPlayer>(_playerPtr1b);
    playerPtr2b = std::dynamic_pointer_cast<FPlayer>(_playerPtr2b);
    playerPtr3b = std::dynamic_pointer_cast<FPlayer>(_playerPtr3b);

    TestSetParentPlayers(); // Tmp assgin parent to players // mTeamA->SetParentPlayers(); //Similar to GetParent()  composite
    std::cout << "1DEBUGGGGGG  mTeamA size : " << mTeamA->children_.size() << std::endl;
    std::cout << "2DEBUGGGGGG  mTeamA size : " << mTeamA->Size() << std::endl;
    
    mStadium->createStadiumInRendererGz(); // ex init()
    mBall->createBallInRendererGz();       // ex init()
    playerPtr1->createPlayerInRendererGz();       
    playerPtr2->createPlayerInRendererGz();       
    playerPtr3->createPlayerInRendererGz();       
    playerPtr1b->createPlayerInRendererGz();       
    playerPtr2b->createPlayerInRendererGz();       
    playerPtr3b->createPlayerInRendererGz();  


//Test
    playerPtr1->showLevel();

  };
  ~FGame(){
      // delete Player;
      // delete mTeamA;
  };
  std::string mNameFGame;
  physics::WorldPtr mWorldPtr = nullptr;

  FPlayerCommonPtr pdu;

  void DebugShowVecPlayers_BallDistance() // For debug
  {
    std::cout << "DEBUG vecPlayers_BallDistance \n";
    for (auto &val : mTeamA->vecPlayers_BallDistance)
      std::cout << "1st: " << val.first->Name() << " 2nd: " << val.second << std::endl;
    // std::cout<< "1st: "<<&val.first<< " 2nd: "<<val.second<<std::endl;
  }
  void DebugShowVecPlayers_BallDistanceSort() // For debug
  {
    std::cout << " DebugShowVecPlayers_BallDistanceSort \n";
    for (auto &val : mTeamA->vecPlayers_BallDistanceSort)
      std::cout << "1st: " << val.first->Name() << " 2nd: " << val.second << std::endl;
    // std::cout<< "1st: "<<&val.first<< " 2nd: "<<val.second<<std::endl;
  }



 
  void UpdateGame() //-------------------(1 iteration)
  {
    // MsgCommand msgCmd = getMsgControlGz(mMovX, mMovY, mMovSprint, mActionBarrerse, mActionPase, mActionPatear);
    std::cout << "DISPLAY msg : "
              << msgCmd->movX << " " << msgCmd->movY << " " << msgCmd->sprint << " "
              << msgCmd->sweep << " " << msgCmd->sweep << " " << msgCmd->pass << " " << msgCmd->kick << "\n"; // DISABLE_KEYBOARD

    mTeamA->Update(); // ex void UpdateState(const MsgCommandPtr& msgCmd)  in Player.hpp
    mTeamB->Update(); // ex void UpdateState(const MsgCommandPtr& msgCmd)  in Player.hpp
    
    // mTeamA->Draw(); //ex UpdateRendering()

    mTeamA->SetVecPlayers_BallDistance();
    mTeamB->SetVecPlayers_BallDistance();
    
    DebugShowVecPlayers_BallDistance();     // For debug
    DebugShowVecPlayers_BallDistanceSort(); // For debug
    DebugSetParentPlayersShow();
    mTeamA->EnablePlayerMovement(); //habilita que se pueda controlar al jugador mas proximo a la bola    solo para el equipoA 
    //mTeamB->EnablePlayerMovement();

    //FBall receives a FTeam ptr
    //CRITERIUS: mBall->mPlayers is The team closest to the ball is the one with possession.
    mBall->mPlayers = mTeamA; //tmp mTeamA,mTeamB  asumo que el equipo que tiene la bola es el mTeamA
    if(mTeamA->vecPlayers_BallDistanceSort[0].second > mTeamB->vecPlayers_BallDistanceSort[0].second) // .second  is the distance to ball
      mBall->mPlayers = mTeamB; //tmp mTeamA,mTeamB  asumo que el equipo que tiene la bola es el mTeamA
    std::cout << "---------------- Tiene la bola el equipo : "<< mBall->mPlayers->Name()<<std::endl ;

    mBall->Update();
  }

  void LoadAnimationsRendererGz()
  {

    // Load animations  BE CARREFULL READ ONLY AT THE BEGGINIG
    auto animElems = mSdfPtr->GetElement("animations");
    auto animElem = animElems->GetElement("animation");
    std::uint8_t animationsTotal=4; //va de la mano con static_cast<uint8_t>()
    this->pdu->animations.reserve(animationsTotal); //to avoids weird behaviur of std::vector 
    while (animElem)
    {
      this->pdu->animations.push_back(animElem);
      animElem = animElem->GetNextElement("animation");
    }
  }

  // TODO: Usar composite FMatch to compare FMatch.size == GzModels.size
  bool wait1stDraw_assoRenderCompositePtr()
  {
    // std::cout<< "pppppppppppp this->_mMatch->Show() :"  << this->_mMatch->Show()  << std::endl;
    std::cout << "pppppppppppp this->_mMatch->Size() :" << this->_mMatch->Size() << std::endl;
    std::cout << "pppppppppppp this->totalGzObjects :" << totalGzObjects << std::endl;

    //---------Set container mTeam[] from the container of the current the gazebo models-------- ONCE!!!
    const auto &modelsPtrGzVec = _mWorldPtr->Models(); // container with the current the gazebo models
    // if(modelsPtrGzVec.size() == this->_mMatch->Size()) // TODO
    // if(modelsPtrGzVec.size() == (this->mTeamA->GetEntities().size()+2)) // ground+ball+2teams (+2 elements)
    if (modelsPtrGzVec.size() == totalGzObjects) // MANUAL TODO :
    {
      
      // Associtate redererePtr to entities
      for (auto &modelGzPtr : modelsPtrGzVec)
      {
        std::string nameModelGz = modelGzPtr->GetName();

        // other models no actors:   ball, stadium
        // Ptr for ball
        if (nameModelGz == mStadium->Name())
          mStadium->mModelGzPtr = modelGzPtr;

        if (nameModelGz == mBall->Name())
          mBall->mModelGzPtr = modelGzPtr;

        // Ptrs for players
        //  To acces to methods of child ActorGz
        //  const physics::ActorPtr &actorPtr = std::dynamic_pointer_cast<physics::Actor>(modelGzPtr);  //why does not work??  std::  and yes bost::     cause    gzPtr were created with bost::   and for converting need tha same library?

        // for (FEntityPtr &entityPtr : mTeamA->children_)
        for (const std::weak_ptr<FEntity> weakPtr : mTeamA->children_)
        {
          if (const auto &entityPtr = weakPtr.lock())
          {
            const FPlayerPtr &playerPtr = std::dynamic_pointer_cast<FPlayer>(entityPtr);
            if (nameModelGz == entityPtr->Name())
            {
              const physics::ActorPtr &actorPtr = boost::dynamic_pointer_cast<physics::Actor>(modelGzPtr);
              playerPtr->mActorGzPtr = actorPtr;
              //Get collission box adjunt to players 
              std::string playerNameC=playerPtr->Name()+"_collision";
              playerPtr->mActorCollisionGzPtr = _mWorldPtr->ModelByName(playerNameC); //        if (mActorCollisionGzPtr 

              playerPtr->EnableAnimationActorGz(); // take adavantage and to enable actor animation
            /*      
              // Init the map STATE-BEHAVIOUR   advantage clean(remove ifelse) and fast search ???
              playerPtr->mapStateBehaviour[FPlayer::ePlayerState::STOP] = std::function<void()>(&playerPtr->BehaviourPlayerStop);
              playerPtr->mapStateBehaviour[FPlayer::ePlayerState::JOG] = std::function<void()>(&playerPtr->BehaviourPlayerJog);
              playerPtr->mapStateBehaviour[FPlayer::ePlayerState::PASS] = std::function<void()>(&playerPtr->BehaviourPlayerPass);
              playerPtr->mapStateBehaviour[FPlayer::ePlayerState::KICK] = std::function<void()>(&playerPtr->BehaviourPlayerKick);
              playerPtr->mapStateBehaviour[FPlayer::ePlayerState::RUN] = std::function<void()>(&playerPtr->BehaviourPlayerRun);
            */
            }
          }
        }

        for (const std::weak_ptr<FEntity> weakPtr : mTeamB->children_)
        {
          if (const auto &entityPtr = weakPtr.lock())
          {
            const FPlayerPtr &playerPtr = std::dynamic_pointer_cast<FPlayer>(entityPtr);
            if (nameModelGz == entityPtr->Name())
            {
              const physics::ActorPtr &actorPtr = boost::dynamic_pointer_cast<physics::Actor>(modelGzPtr);
              playerPtr->mActorGzPtr = actorPtr;
              //Get collission box adjunt to players 
              std::string playerNameC=playerPtr->Name()+"_collision";
              playerPtr->mActorCollisionGzPtr = _mWorldPtr->ModelByName(playerNameC); //        if (mActorCollisionGzPtr 

              playerPtr->EnableAnimationActorGz(); // take adavantage and to enable actor animation
            }
          }
        }
      }
      LoadAnimationsRendererGz();
      InitPlayerPoses(); // take adavantage pass animations to players
      // mTeamA->SetParentPlayers() ; // take adavantage set Team for players
      //  InitBallPose();
      //  createMapDistancesBall; //once

      return true;

      /*
      https://github.com/freefloating-gazebo/freefloating_gazebo/blob/master/src/freefloating_gazebo_fluid.cpp
            if(std::find_if(parsed_models_.begin(), parsed_models_.end(),
                      [&](const model_st &parsed){return parsed.name == model->GetName();})
         == parsed_models_.end())    // not in parsed models
      */
    }
    return false;
  }

  // TODO: to become method of FTeam, problem depends of pdu
  void InitPlayerPoses()
  {
    // for (FEntityPtr &entityPtr : mTeamA->GetEntities())
    for (const std::weak_ptr<FEntity> weakPtr : mTeamA->children_)
    {
      if (const auto &entityPtr = weakPtr.lock())
      {
        const FPlayerPtr &playerPtr = std::dynamic_pointer_cast<FPlayer>(entityPtr);
        //mejorar Set rotacion inicial del jugador      
         playerPtr->yaw= ignition::math::Angle(atan2(0, 1)).Normalized();// orientation Y:0 ,X:1;  influye en la orientacion de los jugadores

        //----- TMP:-----------ini
        //  Debe ser el equipo que inicie la poses de los jugadores de acuerdo a sus posiciones
        // FTeam hereda de FTeam
        // m_teamA->InitPlayerPoses()
        double zPlayerDefault = 0.86;
        ignition::math::Vector3<double> posInit0;
        if (playerPtr->mPosition == FPlayer::Position::DEFENSE)
        {
          posInit0 = ignition::math::Vector3d(-10, -1, zPlayerDefault);
        
        }
        if (playerPtr->mPosition == FPlayer::Position::MIDFIELDER)
        {
          posInit0 = ignition::math::Vector3d(0, 0, zPlayerDefault);
        }
        if (playerPtr->mPosition == FPlayer::Position::FORWARD)
        {
          posInit0 = ignition::math::Vector3d(0, 5, zPlayerDefault);
        }
        ignition::math::Quaternion<double> rotInit0 = ignition::math::Quaterniond(1, 0, 0, 0);
        playerPtr->mActorGzPtr->SetWorldPose(ignition::math::Pose3d(posInit0, rotInit0));
        playerPtr->mActorGzPtr->SetLinearVel(ignition::math::Vector3d(0, 0, 0));

        // playerPtr->mPlayerCommon = this->pdu;  //ko clone
        playerPtr->mPlayerCommon = pdu; // all the players have access to the common pdu
        // std::shared_ptr<Event> o = std::make_shared<Event>(*e);

            //OK TRUCO TMP once: cargamos todas las animaciones para que esten al alcance rapido en la cache!
             std::cout<<"LOADING ONCE alll ANIMATIONS TO ACCELATE TRANSITIONS   TMP"<<std::endl;
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[3]);
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[2]); 
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[1]);
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[0]);   

      }
    }


    for (const std::weak_ptr<FEntity> weakPtr : mTeamB->children_)
    {
      if (const auto &entityPtr = weakPtr.lock())
      {
        const FPlayerPtr &playerPtr = std::dynamic_pointer_cast<FPlayer>(entityPtr);
        //mejorar Set rotacion inicial del jugador      
        playerPtr->yaw= ignition::math::Angle(atan2(0, -1)).Normalized();// orientation Y:0 ,X:-1;  influye en la orientacion de los jugadores
        
        //----- TMP:-----------ini
        //  Debe ser el equipo que inicie la poses de los jugadores de acuerdo a sus posiciones
        // FTeam hereda de FTeam
        // m_teamA->InitPlayerPoses()
        double zPlayerDefault = 0.86;
        ignition::math::Vector3<double> posInit0;
        if (playerPtr->mPosition == FPlayer::Position::DEFENSE)
        {
          posInit0 = ignition::math::Vector3d(10, -1, zPlayerDefault);
          }
        if (playerPtr->mPosition == FPlayer::Position::MIDFIELDER)
        {
          posInit0 = ignition::math::Vector3d(10, 0, zPlayerDefault);
        }
        if (playerPtr->mPosition == FPlayer::Position::FORWARD)
        {
          posInit0 = ignition::math::Vector3d(10, 5, zPlayerDefault);
        }
        //ignition::math::Quaternion<double> rotInit0 = ignition::math::Quaterniond(1.62, 0, -3.1415/2); //constructor con euler roll,pitch,yaw 
        ignition::math::Quaternion<double> rotInit0 = ignition::math::Quaterniond(0, 0, 0, 1); //constructor con quaternion qw,qx,qy,qz
        playerPtr->mActorGzPtr->SetWorldPose(ignition::math::Pose3d(posInit0, rotInit0));
        playerPtr->mActorGzPtr->SetLinearVel(ignition::math::Vector3d(0, 0, 0));

        // playerPtr->mPlayerCommon = this->pdu;  //ko clone
        playerPtr->mPlayerCommon = pdu; // all the players have access to the common pdu
        // std::shared_ptr<Event> o = std::make_shared<Event>(*e);

            //OK TRUCO TMP once: cargamos todas las animaciones para que esten al alcance rapido en la cache!
             std::cout<<"LOADING ONCE alll ANIMATIONS TO ACCELATE TRANSITIONS   TMP"<<std::endl;
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[3]);
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[2]); 
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[1]);
              playerPtr->mActorGzPtr->LoadAnimation(playerPtr->mPlayerCommon->animations[0]);   

      }
    }
  }

  /*
    // TODO: to become method of FTeam,
    void createMapDistancesBall()
    { //Info ball
      const auto &_ballGzPtr = _mWorldPtr->ModelByName("bola1");
      double posXball = _ballGzPtr->WorldPose().Pos().X(); // m
      double posYball = _ballGzPtr->WorldPose().Pos().Y(); // m

      std::cout << "UpdateMapDistancesBall: _ballGzPtr: " << &_ballGzPtr << "\n";
      for (auto &entityPtr : mTeamA->GetEntities())
      {
        const FPlayerPtr &_playerPtr = std::dynamic_pointer_cast<FPlayer>(entityPtr);
        double posXplayer1 = _playerPtr->mActorGzPtr->WorldPose().Pos().X(); // m
        double posYplayer1 = _playerPtr->mActorGzPtr->WorldPose().Pos().Y(); // m

        std::cout << "DEBUG UpdateMapDistancesBall: posXplayer1= " << posXplayer1 << std::endl;
        // double posXplayer1 = _playerPtr->actorPose.Pos().X(); // m
        // double posYplayer1 = _playerPtr->actorPose.Pos().Y(); // m

        double distBallPlayer = pow(((posXball - posXplayer1) * (posXball - posXplayer1) + (posYball - posYplayer1) * (posYball - posYplayer1)), 0.5);
        std::cout << "DEBUG UpdateMapDistancesBall: distBallPlayer= " << distBallPlayer << std::endl;

        mapPlayers_BallDistance.insert(std::pair<FPlayerPtr, double>(_playerPtr, distBallPlayer));
        std::cout << "DEBUG UpdateMapDistancesBall: size= " << mapPlayers_BallDistance.size() << std::endl;
      }

      // Print the map
      std::cout << "UpdateMapDistancesBall....\n";
      for (auto &it : mapPlayers_BallDistance)
      {
        std::cout << it.first << ' '
                  << it.second << std::endl;
      }
    }
  */

  bool readyToStartOnce = false;
  void update()
  // void OnUpdate() // main loop gz  1 iteration)
  {
    std::cout << "pppppppppppp this->totalGzObjects :" << totalGzObjects << std::endl;

    if (readyToStartOnce)
    {
      std::cout << "UpdateGame" << std::endl;
      UpdateGame();
    }
    else
    {
      // wait to check if compoiste tree == draw render gz list
      // Assssocite rendererGz ptr to entitie of composite tree
      // readyToStartOnce=wait1stDraw_assoRenderCompositePtr();//tmp: se inicia las poses de los players:
      readyToStartOnce = wait1stDraw_assoRenderCompositePtr(); // tmp: se inicia las poses de los players:
      // wait1stDraw_assoRenderCompositePtr(); // tmp: se inicia las poses de los players:

      // mTeamA->InitPlayerPoses(formationStratey442);  m_teamB->InitPlayerPoses(formationStratey433);
    }
  }

  // KO  is other thread
  /*
  void timeWait()   //unused
  {
     using clk = std::chrono::high_resolution_clock;
     using namespace std::chrono_literals;

     constexpr auto fps = 20.0f;
     constexpr auto spf = 1.0s / fps;
     static auto t = clk::now();

     auto passed = clk::now() - t;
     if (passed < spf)
        std::this_thread::sleep_for(spf - passed);

     t = clk::now();
  };

    void run(){
      this->InsertPlayer("player_1", Player::Position::DEFENSE);
      this->InsertPlayer("player_2", Player::Position::MIDFIELDER);
      this->InsertPlayer("player_3", Player::Position::FORWARD);

      //LOOP MAIN
        for(;;){
        OnUpdate();
        timeWait();
      }

    }
  */
};
