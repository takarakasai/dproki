#ifndef OBJ_SHAPE_H
#define OBJ_SHAPE_H

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

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <functional>

#include "ObjFileReader.h"

/* common */
namespace obj {
  static std::string dirpath_;
}

/* link classes for obj format */
namespace obj {
  const Eigen::Vector3d& parseRotaryAxis (std::string &axis_type);
  const std::string RotaryAxis2Str (JointType type);

  errno_t parseLinkInfo (std::ifstream &ifs, Link &link);

  //errno_t parseJointInertia (std::ifstream &ifs, Joint &joint);
  errno_t parseJointInertia (std::ifstream &ifs, Link &link);

  //errno_t parseJointViscosity (std::ifstream &ifs, Joint &joint);
  errno_t parseJointViscosity (std::ifstream &ifs, Link &link);

  //errno_t parseJointRange (std::ifstream &ifs, Joint &joint);
  errno_t parseJointRange (std::ifstream &ifs, Link &link);

  errno_t parseShape (std::ifstream &ifs, Link &link);

  errno_t parseHull (std::ifstream &ifs, Link &link);

  errno_t parseChild (std::ifstream &ifs, Link &link);

  errno_t parseLinkAttribute (std::string &attr_type, std::ifstream &ifs, Link &link);
}

/* Shape classes for obj format */
namespace obj {
  errno_t parseColor (Dp::Shape& shape, std::ifstream &ifs);

  errno_t parseShapeAttribute (std::string &attr_type, Dp::Shape& shape, std::ifstream &ifs);

  errno_t parseShapeAttributes (Dp::Shape &shape, std::ifstream &ifs);

  errno_t composeShapeAttributes (Dp::Shape &shape, std::ofstream &ofs);

  class Stl : public Dp::Stl {
  public:
    Stl() :
      Dp::Stl() {};
    ~Stl() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

  class Compound : public Dp::Compound {
  public:
    Compound() :
      Dp::Compound() {};
    ~Compound() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

  class Sphere : public Dp::Sphere {
  public:
    Sphere() :
      Dp::Sphere() {};
    ~Sphere() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

  class Cylinder : public Dp::Cylinder {
  public:
    Cylinder() :
      Dp::Cylinder() {};
    ~Cylinder() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

  class Rectangular : public Dp::Rectangular {
  public:
    Rectangular() :
      Dp::Rectangular() {};
    ~Rectangular() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

  class Cone : public Dp::Cone {
  public:
    Cone() :
      Dp::Cone() {};
    ~Cone() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose   (std::ofstream &ofs);
  };

}

#endif

