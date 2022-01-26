/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include "FEntity.h"
#include "FPlayerCommon.h"

#include <memory>

#include <unordered_map>
#include <functional>

//#include "FTeam.h"
class FTeam;

class FPlayer;
using FPlayerPtr = std::shared_ptr<FPlayer>;
class FPlayer : public FEntity
{

        // private:   std::string mPosition{"type_default"};

public:
        // TODO use simply a std:string
        // std:string mPosition{"MIDFIELDER"};
        //"GOALKEPPER","DEFENSE","MIDFIELDER","FORWARD"

        enum class Position
        {
                GOALKEPPER = 0,
                DEFENSE = 1,
                MIDFIELDER = 2,
                FORWARD = 3
        };
        Position mPosition = Position::MIDFIELDER;

        FPlayer(const std::string &_m_name, const Position &type);
        virtual ~FPlayer();
        virtual bool IsComposite() const override;

        virtual std::string Operation() const override;
        // new__________________

        virtual void Update() override; // based on show        update keyboard

        std::shared_ptr<FTeam> mTeam; // team   FTeamPtr or FEntityPtr ??

        virtual void showLevel() const override;

        // new methods__________________

        //--------- NEW player variables and methods------------ ini
        //=Position::FORWARD; //by default
        gazebo::physics::ActorPtr mActorGzPtr{nullptr}; // ex    gazebo::physics::ModelPtr
                                                        // void SetActorGzPtr(const gazebo::physics::ActorPtr &modelGzPtr) { mActorGzPtr = modelGzPtr; }
                                                        // gazebo::physics::ActorPtr GetActorGzPtr() const { return mActorGzPtr; }

        gazebo::physics::ModelPtr mActorCollisionGzPtr{nullptr};
        // TODO: TO ADD more collission box for each part of the squellet to make a detailed collision

        double animationFactor{5.1};
        double updateFreq{30};

        FPlayerCommonPtr mPlayerCommon{nullptr};

        //--------- NEW player variables and methods------------ end

        // Variable members
        // ignition::math::Vector3d playerDirection= ignition::math::Vector3d(1,0,0);  // ignition::math::Vector3d    ignition::math::Pose3d
        ignition::math::Vector3d playerDirectionNorm = ignition::math::Vector3d(1, 0, 0);
        ignition::math::Angle yaw = ignition::math::Angle(atan2(0, 1)).Normalized();     // orientation Y:0 ,X:1;  influye en la orientacion de los jugadores
        ignition::math::Angle yawLast = ignition::math::Angle(atan2(0, 1)).Normalized(); // orientation Y:0 ,X:1;  influye en la orientacion de los jugadores

        ignition::math::Pose3d actorPose = ignition::math::Pose3d(ignition::math::Vector3d(-2, -2, 2), ignition::math::Quaterniond(1, 0, 0, 0));
        // Need pointer a ball or gzWorld

public:
        double distBallPlayer{100000000};
        // Control Region   radius around of ball 10m

        // TODO: remove valueTest
public:
        double mPlayerVelocity{0.8};

        MsgCommandPtr msgCmdLast = std::make_shared<MsgCommand>();

        // bool mWithBall{false}; //with ball or without ball UNUSED!!!
        enum class ePlayerState // TODO: to make class states  for using StrategyPattern   to  change player behaviour
        {
                STOP = 0,
                JOG = 1,
                RUN = 2,
                PASS = 3,
                KICK = 4
        };
        ePlayerState mStatePlayer = ePlayerState::STOP;

        // to become operator of class   MsgCommand

        double mTimeInitPass{0}; // timepo desde que inicia el pase o patea
        bool executingPass{false}, executingKick{false};
        bool mModeManual{false};  // true:manual; false:auto
        bool onceModeAuto{false}; // to load once the ePlayerState::RUN animation
        double lastSimTime{0};

        // virtual bool isEqual(const MsgCommandPtr &m, const MsgCommandPtr &n);

        virtual void UpdateDistBallPlayer();
        virtual bool isPlayerInControlRegion();

        virtual void ModeManual();
        virtual void ModeAuto();

        virtual void EnableAnimationActorGz();
        virtual void UpdateState();
        virtual void UpdateRendering(); // draw animation
        virtual void createPlayerInRendererGz();

        // Player states
        virtual void BehaviourPlayerStop();
        virtual void BehaviourPlayerJog();
        virtual void BehaviourPlayerPass();
        virtual void BehaviourPlayerKick();
        virtual void BehaviourPlayerRun();
        // TODO: to use unordered_map  but why make complex? and use simply and swith
        virtual void BehaviourManager(ePlayerState &statePlayer);
        /*
        //std::unordered_map<ePlayerState , std::function<void()>> mapStateBehaviour;
        std::unordered_map<ePlayerState , std::function<void()>> mapStateBehaviour{
                {ePlayerState::STOP,std::function<void()>(&FPlayer::BehaviourPlayerStop)},
                {ePlayerState::JOG,std::function<void()>(&FPlayer::BehaviourPlayerJog)},
                {ePlayerState::RUN,std::function<void()>(&FPlayer::BehaviourPlayerRun)},
                {ePlayerState::PASS,std::function<void()>(&FPlayer::BehaviourPlayerPass)},
                {ePlayerState::KICK,std::function<void()>(&FPlayer::BehaviourPlayerKick)}
        };
*/
};