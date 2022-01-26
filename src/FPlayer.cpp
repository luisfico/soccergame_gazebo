/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FPlayer.h"
//#include "magic_enum.hpp"  //to convert enum class to string
//#include <magic_enum.hpp>  //to convert enum class to string

FPlayer::FPlayer(const std::string &_m_name, const Position &type) : FEntity(_m_name), mPosition(type)
{
        std::cout << "Building " << m_name << " level: " << typeid(*this).name() << " \n";
        /*
                // Init the map STATE-BEHAVIOUR   advantage clean(remove ifelse) and fast search ???
                this->mapStateBehaviour[ePlayerState::STOP] = std::function<void()>(&FPlayer::BehaviourPlayerStop);
                this->mapStateBehaviour[ePlayerState::JOG] = std::function<void()>(&FPlayer::BehaviourPlayerJog);
                this->mapStateBehaviour[ePlayerState::PASS] = std::function<void()>(&FPlayer::BehaviourPlayerPass);
                this->mapStateBehaviour[ePlayerState::KICK] = std::function<void()>(&FPlayer::BehaviourPlayerKick);
                this->mapStateBehaviour[ePlayerState::RUN] = std::function<void()>(&FPlayer::BehaviourPlayerRun);
                //  error: no matching function for call to ‘std::function<void()>::function(void (FPlayer::*)())’
        */
        // this->mapStateBehaviour[ePlayerState::STOP] = std::function<void()>(&(this->BehaviourPlayerStop));
        // this->mapStateBehaviour[ePlayerState::JOG] = std::function<void()>(&(this->BehaviourPlayerJog));
        // this->mapStateBehaviour[ePlayerState::PASS] = std::function<void()>(&(this->BehaviourPlayerPass));
        // this->mapStateBehaviour[ePlayerState::KICK] = std::function<void()>(&(this->BehaviourPlayerKick));
        // this->mapStateBehaviour[ePlayerState::RUN] = std::function<void()>(&(this->BehaviourPlayerRun));
}

FPlayer::~FPlayer()
{
        std::cout << "Destroying " << m_name << " level: " << typeid(*this).name() << " \n";
}

std::string FPlayer::Operation() const
{ // return "FPlayer";
        return m_name;
}

bool FPlayer::IsComposite() const
{
        return false;
}

void FPlayer::showLevel() const
{ // Problem to print enum enum class Position as std::string is complicated. It needs c++17  magic_enum.hpp
        // std::cout << "name: " << m_name << ", type: " << mPosition << " , level: " << typeid(*this).name() << "\n";
        std::cout << "player name: " << m_name << ", position: " << magic_enum::enum_name(mPosition) << " , level: " << typeid(*this).name() << "\n";
        // std::cout << "name: " << m_name << " , level: " << typeid(*this).name() << "\n";
}

// new__________________
void FPlayer::Update() // based on show        update keyboard
{
        UpdateState();     // become  update()
        UpdateRendering(); // become  draw()
}

// new methods__________________

void FPlayer::EnableAnimationActorGz()
{
        // Read in the animation name
        std::string animation{"animation"};
        // Set custom trajectory
        gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new gazebo::physics::TrajectoryInfo());
        trajectoryInfo->type = animation;
        trajectoryInfo->duration = 1.0;
        mActorGzPtr->SetCustomTrajectory(trajectoryInfo);
}

void FPlayer::UpdateDistBallPlayer()
{
        const auto &_ballPtr = _mWorldPtr->ModelByName("bola1");
        double posXball = _ballPtr->WorldPose().Pos().X(); // m
        double posYball = _ballPtr->WorldPose().Pos().Y(); // m

        // double posXball = this->mPlayerCommon.ptrGzBall->WorldPose().Pos().X(); // m
        // double posYball = this->mPlayerCommon.ptrGzBall->WorldPose().Pos().Y(); // m

        // const auto& _playerPtr= _mWorldPtr->ModelByName("player1");
        // dynamic cast  actor ptr gz
        double posXplayer1 = mActorGzPtr->WorldPose().Pos().X(); // m
        double posYplayer1 = mActorGzPtr->WorldPose().Pos().Y(); // m
        std::cout << "DEBUG FPLAYER: posXplayer1= " << posXplayer1 << std::endl;

        // Update distance ball!!!
        this->distBallPlayer = pow(((posXball - posXplayer1) * (posXball - posXplayer1) + (posYball - posYplayer1) * (posYball - posYplayer1)), 0.5);

        // Fill map mapPlayers_BallDistance
        // TODO: to acces to parent ptr of players : the teamPtr
        std::cout << "UpdateDistBallPlayer: test playerX------- : " << this->Name() << " level: " << typeid(*this).name() << "\n";
        std::cout << "UpdateDistBallPlayer: DEBUG FPLAYER: distBallPlayer= " << distBallPlayer << std::endl;

        // std::cout << "\n test ptrFTeamx------- : " << this->mTeam->Name() << " level: " << typeid(*mTeam).name()<<"\n";
        // this->mTeam->testLevelFEntity();  //ko because  ptrFEntity  is  in FScene* level     why  typeid  says   FScene*????

        /*
        mTeam=this->GetParent();  //unused  . Using   mTeamA->SetParentPlayers()
        FTeamPtr ptrFTeamx = std::dynamic_pointer_cast<FTeam>(mTeam); // downcast inheritance
        std::cout << "\n test playerX------- : " << this->Name() << " level: " << typeid(*this).name()<<"\n";
        std::cout << "\n test ptrFTeamx------- : " << ptrFTeamx->Name() << " level: " << typeid(*ptrFTeamx).name()<<"\n";
        ptrFTeamx->testLevelFEntity();  //ko because  ptrFEntity  is  in FScene* level     why  typeid  says   FScene*????
        ptrFTeamx->testLevelFScene();  //ko because  ptrFEntity  is  in FScene* level     why  typeid  says   FScene*????
        ptrFTeamx->testLevelFTeam();
        */

        // teamPtr->vecPlayers_BallDistance.push_back(std::make_pair(this,distBallPlayer));
        // mapPlayers_BallDistance.insert(std::pair<FPlayerPtr, double>(_playerPtr, distBallPlayer) );
        // std::cout<<"DEBUG UpdateMapDistancesBall: size= "<<mapPlayers_BallDistance.size() <<std::endl;
}

bool FPlayer::isPlayerInControlRegion()
{
        if (distBallPlayer < 2) // radio 2m
                return true;
        else
                return false;
}
/*
// to become operator of class   MsgCommand
bool FPlayer::isEqual(const MsgCommandPtr &m, const MsgCommandPtr &n)
{
        return (m->movX == n->movX &&
                m->movY == n->movY &&
                m->kick == n->kick &&
                m->pass == n->pass &&
                m->passLong == n->passLong &&
                m->sprint == n->sprint &&
                m->sweep == n->sweep);
}
*/

void FPlayer::BehaviourPlayerStop()
{
        mActorGzPtr->LoadAnimation(this->mPlayerCommon->animations[0]);
        mPlayerVelocity = 0;
}
void FPlayer::BehaviourPlayerJog()
{
        mActorGzPtr->LoadAnimation(this->mPlayerCommon->animations[1]);
        mPlayerVelocity = 0.8; // by default 0.8

        // Set direction player
        // playerDirection.Z() = 0;playerDirection.X() = msgCmd->movX;playerDirection.Y() = msgCmd->movY; // Enable to follow Jostick command as player direction
        // playerDirectionNorm = (playerDirection).Normalize();
        playerDirectionNorm = ignition::math::Vector3d(msgCmd->movX, msgCmd->movY, 0).Normalized();
        yaw = atan2(playerDirectionNorm.Y(), playerDirectionNorm.X()); // orientation
                                                                       // yaw.Normalize(); //-pi to pi
}
void FPlayer::BehaviourPlayerPass()
{
        mActorGzPtr->LoadAnimation(this->mPlayerCommon->animations[2]);
        mPlayerVelocity = 0; // just during a moment
        executingPass = true;
        mTimeInitPass = mActorGzPtr->GetWorld()->SimTime().Double();
}
void FPlayer::BehaviourPlayerKick()
{
        // mActorGzPtr->LoadAnimation(this->dataPtr->animations[3]);//
        mActorGzPtr->LoadAnimation(this->mPlayerCommon->animations[3]);
        mPlayerVelocity = 0; // just during a moment
        executingKick = true;
        mTimeInitPass = mActorGzPtr->GetWorld()->SimTime().Double();
}
void FPlayer::BehaviourPlayerRun()
{
        mPlayerVelocity = 4.0;
}

void FPlayer::BehaviourManager(ePlayerState &statePlayer)
{
        switch (statePlayer)
        { //                        STATE:BEHAVIOUR
        case ePlayerState::STOP:
                BehaviourPlayerStop();
                break;
        case ePlayerState::JOG:
                BehaviourPlayerJog();
                break;
        case ePlayerState::PASS:
                BehaviourPlayerPass();
                break;
        case ePlayerState::KICK:
                BehaviourPlayerKick();
                break;
        case ePlayerState::RUN:
                BehaviourPlayerRun();
                break;
        default:
                break;
        }
}

void FPlayer::ModeManual()
{

        if (executingPass || executingKick)
        {
                double current_time = _mWorldPtr->SimTime().Double();
                double dTimeInitPass = current_time - mTimeInitPass;
                executingPass = (dTimeInitPass < 0.8); // depens on pc power?
                executingKick = (dTimeInitPass < 2);   // depens on pc power?
                std::cout << "              uuuuuuuuuuuuuuuu dTimeInitPass is " << dTimeInitPass << std::endl;
        }
        else
        {
                // if (!isEqual(msgCmdLast, msgCmd)) // change animation only if there is a change in the command
                if (!(msgCmdLast == msgCmd)) // change animation only if there is a change in the command
                {
                        // MOVEMENT
                        if ((msgCmd->movX == 0 && msgCmd->movY == 0)) // no movement
                        {
                                mStatePlayer = ePlayerState::STOP; // BehaviourPlayerStop();
                        }
                        else //(msgCmd->movX != 0 || msgCmd->movY != 0) // movement
                        {
                                mStatePlayer = ePlayerState::JOG; // BehaviourPlayerJog();
                        }

                        // ACTION
                        if (msgCmd->pass)
                        {
                                mStatePlayer = ePlayerState::PASS; // BehaviourPlayerPass();
                        }
                        if (msgCmd->kick)
                        {
                                mStatePlayer = ePlayerState::KICK; // BehaviourPlayerKick();
                        }
                        if (msgCmd->sprint)
                        {
                                mStatePlayer = ePlayerState::RUN; // BehaviourPlayerRun();
                        }

                        // Future map: STATE-BEHAVIUR ------ to become a extern State machine class pattern??
                        BehaviourManager(mStatePlayer);

                        // if (mapStateBehaviour.find(mStatePlayer) != mapStateBehaviour.end())
                        // mapStateBehaviour.find(mStatePlayer)->second();
                        // mapStateBehaviour[mStatePlayer];

                        // auto playerDirection = targetPose.Pos();
                        // save last orientation
                        yawLast = yaw;
                }
                *msgCmdLast = *msgCmd;
                // msgCmdLast->setMsgControlGz(msgCmd->movX, msgCmd->movY, msgCmd->sprint, msgCmd->sweep, msgCmd->pass, msgCmd->kick);
        }
        onceModeAuto = false;
}

void FPlayer::ModeAuto()
{
        // once
        mStatePlayer = ePlayerState::JOG;
        mActorGzPtr->LoadAnimation(this->mPlayerCommon->animations[1]);
        mPlayerVelocity = 0.1; // by default 0.8

        // Set direction player    use init rotation
        // playerDirectionNorm     =ignition::math::Vector3d(1, 0, 0).Normalized(); //direccion x-axis
        // this->yaw = atan2(playerDirectionNorm.Y(), playerDirectionNorm.X()); // orientation
        // this->yaw.Normalize();
}

void FPlayer::UpdateState()
{
        this->UpdateDistBallPlayer();
        // if (this->isPlayerInControlRegion())
        //------------------mode MANUAL
        if (this->mModeManual)
                ModeManual();
        //------------------mode AUTO   it must be a state class   because it will be big
        if (this->mModeManual == false && !onceModeAuto)
        {
                ModeAuto();
                onceModeAuto = true; // to load once the ePlayerState::RUN animation
        }
}

void FPlayer::UpdateRendering() // draw animation
{

        // UpdateState----------------------------ini

        double dt = mActorGzPtr->GetWorld()->SimTime().Double() - this->lastSimTime;

        if (dt < 1 / this->updateFreq)
                return;

        lastSimTime = mActorGzPtr->GetWorld()->SimTime().Double();

        // Current pose - actor is oriented Y-up and Z-front
        actorPose = mActorGzPtr->WorldPose();
        actorPose.Pos() += playerDirectionNorm * mPlayerVelocity * dt; // DEBUG:  actorPose.Pos().X()
        // Influye en la orientacion inicial de todos los jugadores
        actorPose.Rot() = ignition::math::Quaterniond(1.24, 0, (M_PI_2 + this->yaw.Radian())); // euler to Quaterniond
        // Distance traveled is used to coordinate motion with the walking
        // animation
        // double distanceTraveled = (actorPose.Pos() - mActorGzPtr->WorldPose().Pos()).Length();
        // double distanceTraveled = mPlayerVelocity * dt;
        // std::cout<<"distanceTraveled: "<<distanceTraveled <<std::endl;

        // Update actor

        if (executingPass)
        {
                mActorGzPtr->SetWorldPose(actorPose, false, false);
                mActorGzPtr->SetScriptTime(0 + dt * this->animationFactor);
        }
        else
        {
                mActorGzPtr->SetWorldPose(actorPose, false, false);
                mActorGzPtr->SetScriptTime(mActorGzPtr->ScriptTime() + dt * this->animationFactor);
        }

        // mActorGzPtr->SetWorldPose(actorPose, false, false);
        // mActorGzPtr->SetScriptTime(mActorGzPtr->ScriptTime() + dt*this->dataPtr->animationFactor  );

        // UpdateState----------------------------end

        // this->mActorCollisionGzPtr->SetWorldPose(  mActorGzPtr->WorldPose()); //adjunt collisiion box to each player
        // NOTE: actor pose corresponds to lin  player_1::Hips   //example  player_1::Spine
        this->mActorCollisionGzPtr->SetWorldPose(mActorGzPtr->GetLink("Spine")->WorldPose()); // adjunt collisiion box to each player
        // this->mActorCollisionGzPtr->SetWorldPose( mActorGzPtr->GetLink("Head")->WorldPose()); //adjunt collisiion box to each player
        // TODO: TO ADD more collission box for each part of the squellet to make a detailed collision
}

void FPlayer::createPlayerInRendererGz()
{
        sdf::SDF obj1SDF;
        obj1SDF.SetFromString(models::modelStringFormat_player); // by default include "libmodel_push1" plugin Unused
        sdf::ElementPtr obj1model = obj1SDF.Root()->GetElement("actor");
        obj1model->GetAttribute("name")->SetFromString(this->Name());
        _mWorldPtr->InsertModelSDF(obj1SDF);

        // Add box collission
        sdf::SDF obj1SDFc;
        obj1SDFc.SetFromString(models::modelStringFormat_collisionBox); // by default include "libmodel_push1" plugin Unused
        sdf::ElementPtr obj1modelc = obj1SDFc.Root()->GetElement("model");
        std::string playerNameC = this->Name() + "_collision";
        obj1modelc->GetAttribute("name")->SetFromString(playerNameC);
        _mWorldPtr->InsertModelSDF(obj1SDFc);
}