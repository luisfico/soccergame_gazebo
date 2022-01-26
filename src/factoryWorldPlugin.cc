/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

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

#include "FGame.h"


MsgCommandPtr msgCmd;
gazebo::physics::WorldPtr _mWorldPtr;
sdf::ElementPtr mSdfPtr;


//======================== GAZEBO PLUGIN================================

class Factory : public gazebo::WorldPlugin
{

  /// \brief A node used for transport
  gazebo::transport::NodePtr node;

  /// \brief A subscriber to a named topic. (for receiving radius Factor of agvCmd API)
  gazebo::transport::SubscriberPtr sub;
  gazebo::transport::SubscriberPtr subAction;

  // std::vector<double> msg1,msg ;
  double mMovX = 0, mMovY = 0, mMovXsave = 0, mMovYsave = 0;
  double mMovSprint = 0;
  double mActionBarrerse = 0, mActionPase = 0, mActionPatear = 0; // is the logic jostick 0: push    1:no push

  // Callbacks
  void cmd2Callback(ConstVector3dPtr &_msg)
  {
    this->mActionBarrerse = 0, mActionPase = _msg->y(), mActionPatear = _msg->z();
    // std::cout << "cmd1Callback:: mActionBarrerse: " << mActionBarrerse<< " mActionPase: " << mActionPase << " mActionPatear: " << mActionPatear << std::endl;
    /*
       msg1.clear();
       msg1.push_back(mMovX);
       msg2.push_back(mMovY);
       msg1.push_back(mMovSprint);
    */
  }
  void cmd1Callback(ConstVector3dPtr &_msg)
  {
    mMovX = _msg->x();      // joystick push  axis xy
    mMovY = _msg->y();      // joystick push  axis xy
    mMovSprint = _msg->z(); // joystick push R1

#ifdef DEBUG1 //#endif
    std::cout << "TrajectoryActorPlugin::cmd2Callback Receiving movX : " << mMovX << std::endl;
    std::cout << "TrajectoryActorPlugin::cmd2Callback Receiving movY : " << mMovY << std::endl;
    std::cout << "TrajectoryActorPlugin::cmd2Callback Receiving mMovSprint : " << mMovSprint << std::endl;
#endif
    /*
        msg.clear();
        msg.push_back(mMovX);
        msg.push_back(mMovY);
        msg.push_back(mMovSprint);
    */

    //Fill cmd msg {cmd1+cmd2}
    //msgCmd->setMsgControlGz(mMovX, mMovY, mMovSprint, mActionBarrerse, mActionPase, mActionPatear);
    *msgCmd = MsgCommand{mMovX, mMovY, mMovSprint, mActionBarrerse, mActionPase, mActionPatear};
  }

  

public:
  
  std::shared_ptr<FGame> mMatchPtr1 = nullptr;
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    //TMP: For controllerManager: gz communication  
    this->node = transport::NodePtr(new gazebo::transport::Node());
    this->node->Init("NodeControlling");                                                         //
    this->sub = this->node->Subscribe("/cmdFifaGz", &Factory::cmd1Callback, this);              // the callbacks are spinned in OnUpdate()
    this->subAction = this->node->Subscribe("/cmdFifaGzAction", &Factory::cmd2Callback, this); // the callbacks are spinned in OnUpdate()



   
    std::cout << "1gz: DEBUG &_parent : " << &_parent << std::endl;
    std::cout << "1gz: DEBUG &_sdf : " << &_sdf << std::endl;
    
    //_mWorldPtr = std::make_shared<gazebo::physics::World>(""); //global ptr  ko
    //_mWorldPtr = new gazebo::physics::World(); //global ptr

    //mSdfPtr = std::make_shared<sdf::Element>(); //global ptr ok
    
    //gazebo::physics::WorldPtr _mWorldPtr = boost::make_shared<gazebo::physics::World>();
    //sdf::ElementPtr mSdfPtr = std::make_shared<sdf::Element>(); // global ptr
    _mWorldPtr = boost::make_shared<gazebo::physics::World>();
    mSdfPtr = std::make_shared<sdf::Element>(); // global ptr

    _mWorldPtr=_parent;  //ok
    mSdfPtr=_sdf;
    
    std::cout <<"2gz: DEBUG &worldPtr : " << &_mWorldPtr << std::endl;
    std::cout <<"2gz: DEBUG &mSdfPtr  : " << &mSdfPtr << std::endl;
     
    msgCmd = std::make_shared<MsgCommand>(); //global ptr
    //std::cout << "INICIO: msgCmd " << &msgCmd << std::endl;
    

    mMatchPtr1 = std::make_shared<FGame>("Match_1");
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Factory::OnUpdate, this));
    //mMatchPtr1->init();
  
  }

public:
  // void OnUpdate() // main loop gz  1 iteration)
  void OnUpdate()
  {
    //std::cout << "INICIO: msgCmd " << &msgCmd << std::endl;
    std::cout << "2gz: DEBUG &worldPtr : " << &_mWorldPtr << std::endl;
    std::cout << "2gz: DEBUG &mSdfPtr : " << &mSdfPtr << std::endl;
     
    mMatchPtr1->update();
  }

  // Pointer to the update event connection
private:
  gazebo::event::ConnectionPtr updateConnection;
  // std::unique_ptr<ControllerManager> controllerManagerPtr= nullptr; // //DISABLE_KEYBOARD

  
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)

