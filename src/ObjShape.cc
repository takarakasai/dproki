
//#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>
#include <list>
#include <string>

#include "dp_type.h"

//#include "PrimitiveObject.h"
#include "Link.h"
//#include "DrawableLink.h"
//#include "Mesh.h"
#include "Shape.h"

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <functional>

#include "ObjSensor.h"
#include "ObjFileReader.h"

namespace obj {

  //std::string dirpath_;

  const Eigen::Vector3d& parseRotaryAxis (std::string &axis_type) {
      static const std::unordered_map<std::string, Eigen::Vector3d> cases = {
        {"Base",        (Eigen::Vector3d){0.0,0.0,0.0}},
        {"RotaryLinkX", (Eigen::Vector3d){1.0,0.0,0.0}},
        {"RotaryLinkY", (Eigen::Vector3d){0.0,1.0,0.0}},
        {"RotaryLinkZ", (Eigen::Vector3d){0.0,0.0,1.0}}
      };
      auto result = cases.find(axis_type);
      return result != cases.end() ? result->second : cases.begin()->second ;
  }

  const std::string RotaryAxis2Str (JointType type) {
      switch (type) {
          case kBase:    return "Base";
          case kRigid:   return "RigidLink";
          case kRotaryX: return "RotaryLinkX";
          case kRotaryY: return "RotaryLinkY";
          case kRotaryZ: return "RotaryLinkZ";
          default:       return "RotaryLinkU";
      }
  }

  errno_t parseLinkInfo (std::ifstream &ifs, Link &link) {
      Dp::Math::real mass;
      ifs >> mass;
      link.SetMass(mass);

      Vector3d centroid;
      ifs >> centroid(0);
      ifs >> centroid(1);
      ifs >> centroid(2);
      //centroid(0) << ifs;
      //centroid(1) << ifs;
      //centroid(2) << ifs;
      link.SetCentroid(centroid);

      Matrix3d inertia;
      ifs >> inertia(0,0);
      ifs >> inertia(1,1);
      ifs >> inertia(2,2);
      ifs >> inertia(1,0);
      ifs >> inertia(2,1);
      ifs >> inertia(2,0);
      inertia(0,1) = inertia(1,0);
      inertia(1,2) = inertia(2,1);
      inertia(0,2) = inertia(2,0);
      //inertia(0, 0) << ifs;
      //inertia(1, 1) << ifs;
      //inertia(2, 2) << ifs;
      //inertia(0, 1) = inertia(1, 0) << ifs;
      //inertia(1, 2) = inertia(2, 1) << ifs;
      //inertia(0, 2) = inertia(2, 0) << ifs;
      link.SetInertia(inertia);

      std::cout << "mass\n";
      std::cout << mass << std::endl;
      std::cout << "centroid\n";
      std::cout << centroid << std::endl;
      std::cout << "inertia\n";
      std::cout << inertia << std::endl;
      std::cout << "---\n";

      return 0;
  }

  errno_t parseJointInertia (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real val;
      ifs >> idx;
      ifs >> val;

      joint.SetInertia(idx, val);
      return 0;
  }
  errno_t parseJointInertia (std::ifstream &ifs, Link &link) {
      return parseJointInertia(ifs, link.GetJoint());
  }

  errno_t parseJointViscosity (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real val;
      ifs >> idx;
      ifs >> val;

      joint.SetViscosity(idx, val);
      return 0;
  }

  errno_t parseJointViscosity (std::ifstream &ifs, Link &link) {
      return parseJointViscosity(ifs, link.GetJoint());
  }

  errno_t parseJointRange (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real minval, maxval;
      ifs >> idx;
      ifs >> minval;
      ifs >> maxval;

      joint.SetRange(idx, Dp::Math::deg2rad(minval), Dp::Math::deg2rad(maxval));
      return 0;
  }

  errno_t parseJointRange (std::ifstream &ifs, Link &link) {
      return parseJointRange(ifs, link.GetJoint());
  }

  errno_t parseShape (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      link.SetShapePath(file);

      auto shape = ObjFileReader::ReadShape(file);
      if (shape == nullptr) {
        return -1;
      }

      link.SetShape(shape);

      //std::cout << "Shape not implemented : " << file << "\n";

      return 0;
  }

  errno_t parseHull (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      link.SetHullPath(file);

      auto hull = ObjFileReader::ReadShape(file);
      if (hull == nullptr) {
        return -1;
      }

      link.SetShape(hull);

      //std::cout << "Hull not implemented : " << file << "\n";

      return 0;
  }

  errno_t parseChild (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      Eigen::Vector3d xyz, rpy;
      ifs >> xyz(0);
      ifs >> xyz(1);
      ifs >> xyz(2);
      ifs >> rpy(0);
      ifs >> rpy(1);
      ifs >> rpy(2);
      
      std::cout << "Child : " << file
                << " xyz = " << xyz(0)  << "," << xyz(1) << "," << xyz(2)
                << " rpy = " << rpy(0)  << "," << rpy(1) << "," << rpy(2) << "\n\n\n";

      /* TODO: LTipRot, TipPos */
      std::string path = dirpath_ + file;
      auto clink = ObjFileReader::ImportLinkFile(path);
      if (!clink) {
        return -1;
      }

      clink->SetFilePath(file);

      clink->LTipPos() = xyz;
      //clink->LTipRot() = Dp::Math::rpy2mat3(rpy);
      clink->SetTipRpy(rpy);
      link.AddChild(clink);

      return 0;
  }

  errno_t parseSensor (std::ifstream &ifs, Link &link) {
      std::string type;
      ifs >> type;
      auto sens = obj::CreateSensor(obj::Str2SensorType(type));
      if (sens == nullptr) {
        return -1;
      }

      std::string name;
      ifs >> name;
      sens->SetName(name);

      ECALL(sens->parse(ifs));

      link.AddSensor(sens);
     
      return 0;
  }

  errno_t parseLinkAttribute (std::string &attr_type, std::ifstream &ifs, Link &link) {
      static const std::unordered_map<std::string, std::function<errno_t(std::ifstream&, Link&)>> cases = {
        {"Shape"         , [](std::ifstream &ifs, Link &link){return parseShape(ifs, link);         }},
        {"Hull"          , [](std::ifstream &ifs, Link &link){return parseHull(ifs, link);          }},
        {"Child"         , [](std::ifstream &ifs, Link &link){return parseChild(ifs, link);         }},
        {"Inertia"       , [](std::ifstream &ifs, Link &link){return parseLinkInfo(ifs, link);      }},
        {"JointInertia"  , [](std::ifstream &ifs, Link &link){return parseJointInertia(ifs, link);  }},
        {"JointViscosity", [](std::ifstream &ifs, Link &link){return parseJointViscosity(ifs, link);}},
        {"JointRange"    , [](std::ifstream &ifs, Link &link){return parseJointRange(ifs, link);    }},
        {"Sensor"        , [](std::ifstream &ifs, Link &link){return parseSensor(ifs, link);        }}
      };
      //static const std::unordered_map<std::string, std::function<errno_t(std::ifstream&, Link&)>> cases = {
      //  {"JointInertia"  , parseJointInertia  },
      //  {"JointViscosity", parseJointViscosity},
      //  {"JointRange"    , parseJointRange    }
      //};
      auto result = cases.find(attr_type);
      return result != cases.end() ? result->second(ifs, link) : EINVAL ;
  }
}
 
errno_t obj::parseColor (Dp::Shape& shape, std::ifstream &ifs) {
  std::cout << "  : parse Color\n";

  Eigen::Vector4d color;
  ifs >> color(0);
  ifs >> color(1);
  ifs >> color(2);
  ifs >> color(3);

  std::cout << "  :::" << color << std::endl;

  //ECALL(shape.SetColor(color));
  (shape.SetColor(color));

  return 0;
}

errno_t obj::parseShapeAttribute (std::string &attr_type, Dp::Shape& shape, std::ifstream &ifs) {
    static const std::unordered_map<std::string, std::function<errno_t(Dp::Shape&, std::ifstream&)>> cases = {
      {"Color"         , [](Dp::Shape& shape, std::ifstream &ifs){return parseColor(shape, ifs);         }}
    };
    auto result = cases.find(attr_type);
    /* TODO */
    return result != cases.end() ? result->second(shape, ifs) : EINVAL ;
}

errno_t obj::parseShapeAttributes (Dp::Shape &shape, std::ifstream &ifs) {

  while(1)
  {
    std::string str;
    ifs >> str;
    std::cout << "===: " << str << std::endl;
    if (ifs.eof()) break;

    parseShapeAttribute(str, shape, ifs);
  }

  return 0;
}

errno_t obj::composeShapeAttributes (Dp::Shape &shape, std::ofstream &ofs) {

  const Eigen::Vector4d def(0.0,0.0,0.0,1.0);
  auto diff = shape.GetColor() - def;
  if (Dp::Math::NotZero(diff.dot(diff))) {
    ofs << "Color" << " ";
    ofs << shape.GetColor()(0) << " ";
    ofs << shape.GetColor()(1) << " ";
    ofs << shape.GetColor()(2) << " ";
    ofs << shape.GetColor()(3) << std::endl;
  }

  return 0;
}

/* STL */
errno_t obj::Stl::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape STL\n";

  //auto shape = std::make_shared<Stl>();
  ifs >> filename_;

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Stl::compose   (std::ofstream &ofs) {

  ofs << filename_ << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

/* Compound */
errno_t obj::Compound::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape Compound\n";

  std::string str;
  ifs >> str;

  /* statement guard */
  if (str != "{") {
    fprintf(stderr, "%s : statement error\n", __FUNCTION__);
    return -1;
  }

  while(1) {
    ifs >> str;
    /* statement guard */
    if (str == "}") {
      break;
    }

    std::string line;
    std::getline(ifs, line);
    std::stringstream ss(line);

    /* YRP or RPY */
    Dp::Math::real deg;
    Eigen::Vector3d xyz, rpy;
    ss >> xyz(0);
    ss >> xyz(1);
    ss >> xyz(2);
    ss >> deg; rpy(0) = Dp::Math::deg2rad(deg);
    ss >> deg; rpy(1) = Dp::Math::deg2rad(deg);
    ss >> deg; rpy(2) = Dp::Math::deg2rad(deg);

    Eigen::Vector3d scale(1.,1.,1.);
    ss >> scale(0);
    ss >> scale(1);
    ss >> scale(2);

    //auto rot = Dp::Math::rpy2mat3(rpy);

    // TODO: scale, rot, xyz
    auto shape = ObjFileReader::ReadShape(str/*filepath*/);
    if (nullptr == shape) return -2;

    shape->SetOffsetXyz(xyz);
    shape->SetOffsetRpy(rpy);
    shape->SetScale(scale);
    Add(str/*filepath*/, shape);
  }

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Compound::compose   (std::ofstream &ofs) {

  ofs << "{" << std::endl;

  for (auto compound : compounds_) {
    ofs << "  ";
    ofs << compound.filepath << " ";
    ofs << compound.shape->GetOffsetXyz()(0) << " "
        << compound.shape->GetOffsetXyz()(1) << " "
        << compound.shape->GetOffsetXyz()(2) << " ";
    ofs << Dp::Math::rad2deg(compound.shape->GetOffsetRpy()(0)) << " "
        << Dp::Math::rad2deg(compound.shape->GetOffsetRpy()(1)) << " "
        << Dp::Math::rad2deg(compound.shape->GetOffsetRpy()(2)) << std::endl;

    ECALL(ObjFileReader::ExportShapeFile(compound.filepath, *compound.shape));
  }

  ofs << "}" << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

/* Sphere */
errno_t obj::Sphere::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape Sphere\n";

  //auto shape = std::make_shared<Sphere>();
  ifs >> radius_;

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Sphere::compose   (std::ofstream &ofs) {

  ofs << radius_ << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

/* Cylinder */
errno_t obj::Cylinder::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape Cylinder\n";

  //auto shape = std::make_shared<Cylinder>();
  ifs >> radius_;
  ifs >> height_;

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Cylinder::compose   (std::ofstream &ofs) {

  ofs << radius_ << " ";
  ofs << height_ << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

/* Rectangular */
errno_t obj::Rectangular::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape Rectangular\n";

  //auto shape = std::make_shared<Rectangular>();
  ifs >> width_;
  ifs >> length_;
  ifs >> height_;

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Rectangular::compose   (std::ofstream &ofs) {

  ofs << width_  << " ";
  ofs << length_ << " ";
  ofs << height_ << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

/* Cone */
errno_t obj::Cone::parse (std::ifstream &ifs) {
  std::cout << ">>>: Shape Cone\n";

  //auto shape = std::make_shared<Cone>();
  ifs >> radius_;
  ifs >> height_;

  ECALL(parseShapeAttributes(*this, ifs));

  return 0;
}

errno_t obj::Cone::compose   (std::ofstream &ofs) {

  ofs << radius_ << " ";
  ofs << height_ << std::endl;

  ECALL(composeShapeAttributes(*this, ofs));

  return 0;
}

