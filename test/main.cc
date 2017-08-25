
#include <string>

//#include "dproki/Link.h"
#include "dproki/ObjFileReader.h"

int main(int argc, char *argv[]) {

  if (argc < 3) {
    fprintf(stderr, "%s <dir path> <obj path>\n", argv[0]);
    return 1;
  }

  auto joint = Dp::RotaryJoint::Create("test_joint", (Dp::Vector3d){0.0, 0.0, 1.0});
  auto link = Dp::Link::Create("test_link", joint, (Dp::Vector3d){0.0, 0.0, 0.0}, 0, (Dp::Vector3d){0.0,0.0,0.0}, Dp::Matrix3d::Zero());

  //std::string objpath = "obj/khr3-hv/khr3-hv.obj";
  //std::string objpath = "obj/eV/eV.obj";
  std::string dirpath(argv[1]);
  std::string objpath(argv[2]);

  auto robot = Dp::ObjFileReader::ImportObjFile(dirpath, objpath);
  if (robot == NULL) {
    fprintf(stderr, "fail to load %s %s.\n", dirpath.c_str(), objpath.c_str());
    return 2;
  }

  robot->Setup();
  robot->UpdateCasCoords();

  auto lnks = robot->GetLinks();
  std::cout << "size : " << lnks.size() << std::endl;
  for (auto lnk : lnks) {
    std::cout << " NAME : " << lnk->GetName() << std::endl;
    Dp::Vector3d ltippos = lnk->LTipPos();
    std::cout << "   ltippos: " << ltippos[0] << ", " << ltippos[1] << ", " << ltippos[2] << std::endl;
    Dp::Vector3d wtippos = lnk->GetWTipPos();
    std::cout << "   wtippos: " << wtippos[0] << ", " << wtippos[1] << ", " << wtippos[2] << std::endl;

    Dp::Joint& jnt = lnk->GetJoint();
    std::cout << "     joint: " << jnt.GetAngle() << " ( offset : " << jnt.GetOffsetAngle() << " )" << std::endl;
  }
 
  fprintf(stderr, "success to load %s %s.\n", dirpath.c_str(), objpath.c_str());


  return 0;
}
