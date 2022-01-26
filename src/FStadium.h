/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include "FEntity.h"

#include <memory>

class FStadium;
using FStadiumPtr = std::shared_ptr<FStadium>;
class FStadium : public FEntity
{

private:
        std::string m_type;

public:
        FStadium(const std::string &_m_name, const std::string &type);
        virtual ~FStadium();
        virtual bool IsComposite() const override;

        virtual std::string Operation() const override;

        virtual void showLevel() const override;

        // new__________________
        gazebo::physics::ModelPtr mModelGzPtr{nullptr}; // ex    gazebo::physics::ModelPtr
                                                        // gazebo::physics::ModelPtr mModelGzPtr=boost::make_shared<gazebo::physics::Model>();

        virtual void Update() override; // based on show        update keyboard

        virtual void createStadiumInRendererGz();
};