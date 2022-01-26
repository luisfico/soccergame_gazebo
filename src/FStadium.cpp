/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FStadium.h"

FStadium::FStadium(const std::string &_m_name, const std::string &type) : FEntity(_m_name), m_type(type)
{
  std::cout << "Building " << m_name << " level: " << typeid(*this).name() << " \n";
}

FStadium::~FStadium()
{
  std::cout << "Destroying " << m_name << " level: " << typeid(*this).name() << " \n";
}

std::string FStadium::Operation() const
{ // return "FStadium";
  return m_name;
}

bool FStadium::IsComposite() const
{
  return false;
}

void FStadium::showLevel() const
{
  std::cout << "name: " << m_name << ", type: " << m_type << " , level: " << typeid(*this).name() << "\n";
}

// new__________________
void FStadium::Update() // based on show        update keyboard
{
  std::cout << "FStadium debe actualizar su pose a partir de su ptr de renderer  \n";
  // UpdateState(); // become  update()
  // UpdateRendering();   // become  draw()
}

void FStadium::createStadiumInRendererGz()
{
  sdf::SDF obj1SDF;
  obj1SDF.SetFromString(models::modelStringFormat_stadiumA); // by default include "libmodel_push1" plugin Unused
  sdf::ElementPtr obj1model = obj1SDF.Root()->GetElement("model");
  obj1model->GetAttribute("name")->SetFromString(this->Name());
  _mWorldPtr->InsertModelSDF(obj1SDF);
}
