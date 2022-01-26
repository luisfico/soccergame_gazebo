/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FEntity.h"

#include <memory>
class FScene;
using FScenePtr = std::shared_ptr<FScene>;


class FScene : public FEntity// , public std::enable_shared_from_this<FScene>
{
  /**
   * @var \SplObjectStorage
   */
//protected:
  //std::vector<std::weak_ptr<FEntity>> children_;
  //std::vector<FEntityPtr  > children_;

public:

  std::vector<std::weak_ptr<FEntity>> children_;
  
  FScene(const std::string &_m_name);
  virtual ~FScene();

  /**
   * A FScene object can add or remove other FEntitys (both simple or
   * complex) to or from its child vector.
   */
  virtual void Add(FEntityPtr  cmp) override;
  /**
   * Have in mind that this method removes the pointer to the vector but doesn't
   * frees the
   *     memory, you should do it manually or better use smart pointers.
   */
  virtual void Remove(FEntityPtr  cmp) override;
  virtual bool IsComposite() const override;
  /**
   * The FScene executes its primary logic in a particular way. It traverses
   * recursively through all its children, collecting and summing their results.
   * Since the FScene's children pass these calls to their children and so
   * forth, the whole object tree is traversed as a result.
   */
  virtual std::string Operation() const override;
  virtual void Update() override; // based on show
  virtual int Size() const override;

  virtual void testLevelFScene() const;
};