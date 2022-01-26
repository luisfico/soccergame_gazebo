/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include "FEntity.h"

FEntity::FEntity(const std::string &_m_name) : m_name(_m_name) { std::cout << "Building " << m_name << " level: " << typeid(*this).name() << " \n"; }
FEntity::~FEntity() { std::cout << "Destroying " << m_name << " level: " << typeid(*this).name() << " \n"; }

void FEntity::SetParent(FEntityPtr  parent)
{
  this->FEntity::parent_ = parent;
}
FEntityPtr  FEntity::GetParent() const
{
  return this->FEntity::parent_;
}
std::string FEntity::Name() { return this->m_name; }

void FEntity::Add(FEntityPtr  cmp) {}
void FEntity::Remove(FEntityPtr  cmp) {}
/**
 * You can provide a method that lets the client code figure out whether a
 * FEntity can bear children.
 */
bool FEntity::IsComposite() const
{
  return false;
}

int FEntity::Size() const { return m_size; }
void FEntity::showLevel() const 
{ std::cout << "name: "<< m_name << " , level: " << typeid(*this).name()<<"\n";}
void FEntity::testLevelFEntity() const { std::cout << "testLevelFEntity \n"; }
