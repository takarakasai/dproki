#ifndef OBJ_FILE_READER_H
#define OBJ_FILE_READER_H

//#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>
#include <list>
#include <string>
#include <fstream>

#include "dp_type.h"

#include "Shape.h"
#include "ObjShape.h"
#include "ObjSensor.h"

class Link;
class Object;

class ObjFileReader {
private:
  static std::string dirpath_;

public:
  static const char* Type2str (Dp::SHAPE_TYPE type);
  static const Dp::SHAPE_TYPE Str2type (std::string str);

private:
  /* for shape */
  static std::shared_ptr<Dp::Shape> CreateShape (std::ifstream &ifs, Dp::SHAPE_TYPE type);

  /* for shape  -- Composer */
  static errno_t composeShape (std::ofstream &ofs, Dp::Shape &shape);

  /* for link */
  /* N/A */

  /* for link  -- Composer */
  static errno_t composeLinkAttribute (std::ofstream &ofs, Link &link);
  static errno_t composeObjectAttribute (std::ofstream &ofs, Object &obj);

public:
  /* for shape -- Parser */
  static std::shared_ptr<Dp::Shape> ReadShape (std::string filepath);

  /* for shape -- Composer */
  //static errno_t ExportShapeFile (Link &link);
  static errno_t ExportShapeFile (std::string &filepath, Dp::Shape &shape);

  /* for link  -- Parser */
  static std::shared_ptr<Link> ImportLinkFile (std::string &filepath);
  static std::shared_ptr<Object> ImportObjFile(std::string &dirpath, std::string &filepath);

  /* for link  -- Composer */
  static errno_t ExportLinkFile (Link &link);
  static errno_t ExportLinkFile (std::string &filepath, Link &link);

  static errno_t ExportObjectFile (Object &obj);
  static errno_t ExportObjectFile (std::string &filepath, Object &obj);

  static errno_t Export (Object &obj);
  static errno_t Export (std::string &dirpath, Object &obj);
};

#endif


