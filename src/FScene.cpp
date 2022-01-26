/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FScene.h"

FScene::FScene(const std::string &_m_name) : FEntity(_m_name)
{
  std::cout << "Building " << m_name << " level: " << typeid(*this).name() << " \n";
}

FScene::~FScene()
{
  std::cout << "Destroying " << m_name << " level: " << typeid(*this).name() << " \n";
}

void FScene::Add(FEntityPtr cmp)
{
  this->children_.push_back(cmp);
  cmp->SetParent(this->shared_from_this());
}
/**
 * Have in mind that this method removes the pointer to the vector but doesn't
 * frees the
 *     memory, you should do it manually or better use smart pointers.
 */
void FScene::Remove(FEntityPtr cmp)
{
  int index = 0;
  for (const std::weak_ptr<FEntity> weakPtr : children_)
  {
    // if (c == children_.back()){ result += c->Operation();}
    // else
    if (const auto &ptr = weakPtr.lock())
    {
      if (ptr->m_name == cmp->m_name)
      {
        children_.erase(children_.begin() + index); // children_.remove(ptr   o  weakPtr);
        cmp->SetParent(nullptr);
      }
    }
    index++;
  }
}
bool FScene::IsComposite() const
{
  return true;
}
/**
 * The FScene executes its primary logic in a particular way. It traverses
 * recursively through all its children, collecting and summing their results.
 * Since the FScene's children pass these calls to their children and so
 * forth, the whole object tree is traversed as a result.
 */
std::string FScene::Operation() const
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

int FScene::Size() const
{ // on fait la somme de la taille de chaque Ã©lÃ©ment
  int sumSize = 0;

  for (const std::weak_ptr<FEntity> weakPtr : children_)
  {
    if (const auto &ptr = weakPtr.lock())
    {
      sumSize = sumSize + ptr->Size();
    }
  }
  return sumSize;
}

void FScene::Update()  // based on show
{ for (const std::weak_ptr<FEntity> weakPtr : children_)
  {
    if (const auto &ptr = weakPtr.lock())
    {
      ptr->Update();
    }
  }
}

void FScene::testLevelFScene() const
{
  std::cout << "testLevelFScene \n";
}