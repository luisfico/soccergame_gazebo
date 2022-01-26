/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

#include <memory>




// https://come-david.developpez.com/tutoriels/ndps/?page=Composite



// Dependences renderer gazebo_________________________
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "models.hh"
#include "magic_enum.hpp" 

//-----------------------TrajectoryActorPlugin.hh----------end
#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "MsgCommand.hpp"

// Global variable indirectly called by linking using EXTERN keyword
// extern gazebo::physics::WorldPtr& _mWorldPtr;

// Global varaible directly called by includes
// gazebo::physics::WorldPtr _mWorldPtr{nullptr};
// sdf::ElementPtr mSdfPtr{nullptr};
/*
MsgCommandPtr msgCmd{nullptr};
gazebo::physics::WorldPtr _mWorldPtr = boost::make_shared<gazebo::physics::World>();
sdf::ElementPtr mSdfPtr = std::make_shared<sdf::Element>(); // global ptr
*/

extern MsgCommandPtr msgCmd;
extern gazebo::physics::WorldPtr _mWorldPtr;
extern sdf::ElementPtr mSdfPtr;





class FEntity;
using FEntityPtr = std::shared_ptr<FEntity>;


/**
 * The base FEntity class declares common operations for both simple and
 * complex objects of a composition.
 */
class FEntity  : public std::enable_shared_from_this<FEntity>
{
    static const int m_size = 1;
  /**
   * @var FEntity
   */
protected:

  FEntityPtr  parent_;
  /**
   * Optionally, the base FEntity can declare an interface for setting and
   * accessing a parent of the FEntity in a tree structure. It can also
   * provide some default implementation for these methods.
   */
public:
  virtual ~FEntity();
  FEntity(const std::string &_m_name);

  void SetParent(FEntityPtr  parent);
  FEntityPtr  GetParent() const;
  virtual std::string Name();

  /**
   * In some cases, it would be beneficial to define the child-management
   * operations right in the base FEntity class. This way, you won't need to
   * expose any concrete FEntity classes to the client code, even during the
   * object tree assembly. The downside is that these methods will be empty for
   * the FPlayer-level FEntitys.
   */
  virtual void Add(FEntityPtr  FEntity);
  virtual void Remove(FEntityPtr  FEntity);
  /**
   * You can provide a method that lets the client code figure out whether a
   * FEntity can bear children.
   */
  virtual bool IsComposite() const;
  /**
   * The base FEntity may implement some default behavior or leave it to
   * concrete classes (by declaring the method containing the behavior as
   * "abstract").
   */
  virtual std::string Operation() const = 0; //original
  virtual int Size() const;
  virtual void showLevel() const; 
  virtual void testLevelFEntity() const; //for debug
  std::string m_name;

  virtual void Update() = 0; // Update keyboard

  //FEntityPtr playerPtr;//tmp playerPtr, children_ in same scope
    
};