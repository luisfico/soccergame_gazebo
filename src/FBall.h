/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include "FEntity.h"

#include <memory>

class FTeam;

class FBall;
using FBallPtr = std::shared_ptr<FBall>;
class FBall : public FEntity
{

private:
        std::string m_type;

public:
        FBall(const std::string &_m_name, const std::string &type);
        virtual ~FBall();
        virtual bool IsComposite() const override;

        virtual std::string Operation() const override;

        virtual void showLevel() const override;

        virtual void AssitedPass();

        // new__________________
        virtual void Update() override; // based on show        update keyboard

        enum class eBallState // TODO: to make class states  for using StrategyPattern   to  change player behaviour
        {
                FREE = 0,
                CONTROLED = 1,
                INIT_PASS = 2, // for pass 
                INIT_KICK = 3, // for kick with effect magnus 
        };
        eBallState mStateBall = eBallState::FREE;

        gazebo::physics::ModelPtr mModelGzPtr{nullptr}; // ex    gazebo::physics::ModelPtr
        // gazebo::physics::ModelPtr mModelGzPtr=boost::make_shared<gazebo::physics::Model>();

        virtual void createBallInRendererGz();

        std::shared_ptr<FTeam> mPlayers; 
        double mTimeInitPass=0;
        double angZball=0, angZplayer = 0; //yawBall

};