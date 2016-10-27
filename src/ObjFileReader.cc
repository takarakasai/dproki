
#include "ObjFileReader.h"

#include <iostream>

#include "dprint.h"

//#include <sys/types.h>
#include <sys/stat.h>

#include "Common.h"
#include "ObjSensor.h"

static errno_t my_mkdir (std::string path) {
  errno_t eno = mkdir(path.c_str(), S_IREAD|S_IWRITE|0755);
  //PRINTF("====== %s %d %d\n", path.c_str(), eno, errno);
  return (eno == 0 || errno == EEXIST) ? 0 : errno;
}

static errno_t prepare_directory (std::string filepath) {
  for (size_t i = 1; i < filepath.size(); i++) {
    if (filepath.c_str()[i] == '/') {
      //PRINTF("====== %s %d\n", filepath.c_str(), i);
      ECALL(my_mkdir(filepath.substr(0, i)));
    }
  }
  return 0;  
}

const char* ObjFileReader::Type2str (Dp::SHAPE_TYPE type) {
  switch (type) {
    case Dp::kShapeStl:         return "Stl";
    case Dp::kShapeCompound:    return "Compound";
    case Dp::kShapeSphere:      return "Sphere";
    case Dp::kShapeCylinder:    return "Cylinder";
    case Dp::kShapeRectangular: return "Box";
    case Dp::kShapeCone:        return "Cone";
    default:                return "Invalid";
  }
}

const Dp::SHAPE_TYPE ObjFileReader::Str2type (std::string str) {
  if (str == "Stl") {
    return Dp::kShapeStl;
  } else if (str == "Compound") {
    return Dp::kShapeCompound;
  } else if (str == "Sphere") {
    return Dp::kShapeSphere;
  } else if (str == "Cylinder") {
    return Dp::kShapeCylinder;
  } else if (str == "Box") {
    return Dp::kShapeRectangular;
  } else if (str == "Cone") {
    return Dp::kShapeCone;
  } else {
    return Dp::kShapeInvalid;
  }
}

std::shared_ptr<Dp::Shape> ObjFileReader::CreateShape (std::ifstream &ifs, Dp::SHAPE_TYPE type) {
  std::shared_ptr<Dp::Shape> shape;
  switch (type) {
      case Dp::kShapeStl:         shape = std::make_shared<obj::Stl>();         shape->parse(ifs); break;
      case Dp::kShapeCompound:    shape = std::make_shared<obj::Compound>();    shape->parse(ifs); break;
      case Dp::kShapeSphere:      shape = std::make_shared<obj::Sphere>();      shape->parse(ifs); break;
      case Dp::kShapeCylinder:    shape = std::make_shared<obj::Cylinder>();    shape->parse(ifs); break;
      case Dp::kShapeRectangular: shape = std::make_shared<obj::Rectangular>(); shape->parse(ifs); break;
      case Dp::kShapeCone:        shape = std::make_shared<obj::Cone>();        shape->parse(ifs); break;
      default:                shape = nullptr;
  }
  return shape;
}

errno_t ObjFileReader::composeShape (std::ofstream &ofs, Dp::Shape &shape) {
  ofs << Type2str(shape.Type()) << " ";
  shape.compose(ofs);
  return 0;
}

std::shared_ptr<Dp::Shape> ObjFileReader::ReadShape (std::string filepath) {
  std::ifstream ifs(Common::DirPath() + filepath);
  if (!ifs.good()) {
    std::cout << "error read shape file :" << Common::DirPath() << "|" << filepath << "\n";
    return NULL;
  }

  std::string str;
  ifs >> str;

  return CreateShape(ifs, Str2type(str));
}

errno_t ObjFileReader::ExportShapeFile (std::string &filepath, Dp::Shape &shape) {
  std::string path(Common::DirPath() + Common::OutPathHead() + filepath);
  ECALL(prepare_directory(path));
  std::ofstream ofs(path);
  std::cout << "export shape :" << path << "\n";
  if (!ofs.good()) {
    std::cout << "error export shape:" << Common::DirPath() << "|" << Common::OutPathHead() << "|" << filepath << "\n";
    return EINVAL;
  }

  return composeShape(ofs, shape);
}

errno_t ObjFileReader::composeLinkAttribute (std::ofstream &ofs, Link &link) {
  ofs << obj::RotaryAxis2Str(link.GetJoint().GetType()) << " " << link.GetName() << std::endl;
  if (link.GetShapePath().size() > 0) {
    ofs << "Shape" << " " << Common::OutPathHead() + link.GetShapePath() << std::endl;
  }
  if (link.GetHullPath().size() > 0) {
    ofs << "Hull"  << " " << Common::OutPathHead() + link.GetHullPath()  << std::endl;
  }
  ofs << "Inertia" << std::endl;
  ofs << "     "   << std::showpoint << link.GetMass()                 << std::endl;
  ofs << "     "   << std::showpoint << link.GetCentroid().transpose() << std::endl;
  ofs << "     "   << std::showpoint << link.GetInertia()(0,0) << " "
                   << std::showpoint << link.GetInertia()(1,1) << " "
                   << std::showpoint << link.GetInertia()(2,2) << std::endl;
  ofs << "     "   << std::showpoint << link.GetInertia()(1,0) << " "
                   << std::showpoint << link.GetInertia()(2,1) << " "
                   << std::showpoint << link.GetInertia()(2,0) << std::endl;
  // TODO: multiple attribute
  ofs << "JointRange" << " " << "0" << " "
                             << std::showpoint << Dp::Math::rad2deg(link.GetJoint().GetMinRange(0)) << " "
                             << std::showpoint << Dp::Math::rad2deg(link.GetJoint().GetMaxRange(0)) << std::endl;
  ofs << "JointInertia"   << " " << "0" << " " << std::showpoint << link.GetJoint().GetInertia(0)   << std::endl;
  ofs << "JointViscosity" << " " << "0" << " " << std::showpoint << link.GetJoint().GetViscosity(0) << std::endl;

  auto clinks = link.GetChilds();
  for (auto &clink : clinks) {
      ofs << "Child" << " " << Common::OutPathHead() + clink->GetFilePath() << " "
                            << std::showpoint << clink->LTipPos()(0)  << " " 
                            << std::showpoint << clink->LTipPos()(1)  << " " 
                            << std::showpoint << clink->LTipPos()(2)  << " " 
                            << std::showpoint << clink->LTipRpy()(0)  << " " 
                            << std::showpoint << clink->LTipRpy()(1)  << " " 
                            << std::showpoint << clink->LTipRpy()(2)  << std::endl;
  }

  auto sensors = link.GetSensors();
  for (auto &sens : sensors) {
      ofs << "Sensor" << " " << obj::SensorType2Str(sens->Type()) << " " << sens->GetName() << std::endl;
      sens->compose(ofs);
  }

  return 0;
}

errno_t ObjFileReader::composeObjectAttribute (std::ofstream &ofs, Object &obj) {
  ofs << obj.GetName() << std::endl;
  ofs << Common::OutPathHead() + obj.RootLink()->GetFilePath() << std::endl;
  ofs << std::endl;

  PRINTF("%zd\n", obj.NumOfLinks());
  for (size_t i = 0; i < obj.NumOfLinks(); i++) {
    Dp::Math::real ovalue = obj.GetLink(i).GetJoint().GetOffsetValue()(0);
    if (Dp::Math::NotZero(ovalue)) {
      ofs << "InitJointValue" << " " << obj.GetLink(i).GetName() << " " << ovalue << std::endl;
    }
  }
  for (auto &apair: obj.GetAssocPair()) {
    ofs << "AssocPair";
    ofs << " " << apair->link1_->GetName() << " " << apair->idx1_ << " " << apair->scale1_;
    ofs << " " << apair->link2_->GetName() << " " << apair->idx2_ << " " << apair->scale2_;
    ofs << std::endl;
  }

  return 0;
}

std::shared_ptr<Link> ObjFileReader::ImportLinkFile (std::string &dirpath, std::string &filepath, ssize_t max_childs) {
  Common::DirPath() = dirpath;
  return ObjFileReader::ImportLinkFile(filepath, max_childs);
}
 
std::shared_ptr<Link> ObjFileReader::ImportLinkFile (std::string &filepath, ssize_t max_childs) {
  std::ifstream ifs(Common::DirPath() + filepath);
  if (!ifs.good()) {
    std::cout << "error import link:" << Common::DirPath() << "|" << filepath << "\n";
    return NULL;
  }

  std::string type;
  std::string name;
  //std::getline(ifs, type, ' ');
  //std::getline(ifs, name, ' ');
  ifs >> type;
  ifs >> name;
  std::cout << "type: " << type << ", name: " << name << "|" << std::endl;

  // TODO:
  std::shared_ptr<Joint> joint;
  if (type == "RigidLink") {
    joint = RigidJoint::Create(name.c_str());
  } else {
    joint = RotaryJoint::Create(name.c_str(), obj::parseRotaryAxis(type));
  }

  //auto link = ssg::DrawableLink::Create(name.c_str(), joint, (Vector3d){0.0, 0.0, 0.0}, 0, (Vector3d){0.0,0.0,0.0}, Matrix3d::Zero());
  auto link = Link::Create(name.c_str(), joint, (Vector3d){0.0, 0.0, 0.0}, 0, (Vector3d){0.0,0.0,0.0}, Matrix3d::Zero());
  link->SetFilePath(filepath);

  while(1)
  {
    std::string str;
    ifs >> str;
    if (ifs.eof()) break;

#if defined(DP_DEBUG)
    std::cout << "--: " << Common::DirPath() << "|" << filepath << "  ---:" << str << ":" << ifs.eof() << ":" << std::endl;
#endif
    if (obj::parseLinkAttribute(str, ifs, *link, max_childs) != 0) {
      return nullptr;
    }
  }

  return link;
}

errno_t ObjFileReader::ExportLinkFile (Link &link) {
  std::string path(Common::DirPath() + Common::OutPathHead() + link.GetFilePath());
  ECALL(prepare_directory(path));
  std::ofstream ofs(path);
  std::cout << "export Link : " << path << "\n";
  if (!ofs.good()) {
    std::cout << "error export Link:" << Common::DirPath() << "|" << Common::OutPathHead() << "|" << link.GetFilePath() << "\n";
    return EINVAL;
  }

  return composeLinkAttribute(ofs, link);
}

errno_t ObjFileReader::ExportLinkFile (std::string &filepath, Link &link) {
    link.SetFilePath(filepath);
    return ExportLinkFile(link);
}

/* TODO: Link --> Object */
//std::shared_ptr<Link> ImportObjFile(std::string &dirpath, std::string &filepath) {
std::shared_ptr<Object> ObjFileReader::ImportObjFile(std::string &dirpath, std::string &filepath) {
  Common::DirPath() = dirpath;
  std::ifstream ifs(Common::DirPath() + filepath);
  if (!ifs.good()) {
    std::cout << "error import obj :" << Common::DirPath() << "|" << filepath << "\n";
    return NULL;
  }

  /* object name */
  std::string obj_name;
  std::string root_file;
  std::getline(ifs, obj_name);
  std::getline(ifs, root_file);
  PRINTF("OBJ NAME: %s\n", obj_name.c_str());
  PRINTF("ROOT FILE NAME: %s\n", root_file.c_str());
  /*               file.name --> "" */
  /*              /file.name --> "/" */
  /* hoge/fuga/aho/file.name --> "hoge/fuga/aho/" */
  //std::string data_dir = root_file.substr(0, root_file.find_last_of("/") + 1);
  auto link = ImportLinkFile(root_file, -1);
  if (link == NULL) {
    return NULL;
  }

  auto obj = Object::Create(obj_name.c_str());
  obj->SetFilePath(filepath);
  obj->SetRootLink(link);

  std::string str;
  while(std::getline(ifs, str))
  {
    std::string tmp;
    std::istringstream stream(str);
    while(std::getline(stream, tmp, ' '))
    {
      if (tmp == "InitJointValue") {
        std::string link_name;
        std::getline(stream, link_name, ' ');
        Dp::Math::real link_val;
        stream >> link_val;
        auto joint = obj->FindJoint(link_name);
        if (joint) {
          //joint->SetValue(link_val);
          joint->SetOffsetValue(link_val);
        }
      } else if (tmp == "AssocPair") {
        std::string link_name;
        size_t idx1, idx2;
        Dp::Math::real scale1, scale2;

        stream >> link_name;
        auto link1 = obj->FindLink(link_name);
        stream >> idx1;
        stream >> scale1;

        stream >> link_name;
        auto link2 = obj->FindLink(link_name);
        stream >> idx2;
        stream >> scale2;

        if (link1 && link2) {
          obj->AddAssocPair(link1, idx1, scale1, link2, idx2, scale2);
        }
      }
    }
  }

  return obj;
}

errno_t ObjFileReader::ExportObjectFile (Object &obj) {
  std::string path(Common::DirPath() + Common::OutPathHead() + obj.GetFilePath());
  ECALL(prepare_directory(path));
  std::ofstream ofs(path);
  std::cout << "export obj : " << path << "\n";
  if (!ofs.good()) {
    std::cout << "error export obj:" << Common::DirPath() << "|" << Common::OutPathHead() << "|" << obj.GetFilePath() << "\n";
    return EINVAL;
  }

  return composeObjectAttribute(ofs, obj);
}

errno_t ObjFileReader::ExportObjectFile (std::string &filepath, Object &obj) {
  obj.SetFilePath(filepath);
  return ExportObjectFile(obj);
}

errno_t ObjFileReader::Export (Object &obj) {
  ECALL(ExportObjectFile(obj));
  for (size_t i = 0; i < obj.NumOfLinks(); i++) {
    ECALL(ExportLinkFile(obj.GetLink(i)));

    if (obj.GetLink(i).GetShapePath().size() > 0) {
      ECALL(ExportShapeFile(obj.GetLink(i).GetShapePath(), *(obj.GetLink(i).GetShape())));
    }

    if (obj.GetLink(i).GetHullPath().size() > 0) {
      // TODO: compare pointer and path
      if (obj.GetLink(i).GetShapePath() != obj.GetLink(i).GetHullPath()) {
        ECALL(ExportShapeFile(obj.GetLink(i).GetHullPath(), *(obj.GetLink(i).GetHull())));
      }
    }
  }
  return 0;
}

errno_t ObjFileReader::Export(std::string &dirpath, Object &obj) {
  Common::DirPath() = dirpath;
  Common::OutPathHead() = "";
  ECALL(Export(obj));
  return 0;
}

errno_t ObjFileReader::Export(std::string &dirpath, std::string &out_path_head, Object &obj) {
  Common::DirPath() = dirpath;
  Common::OutPathHead() = out_path_head;
  ECALL(Export(obj));
  return 0;
}


