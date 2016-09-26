
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mf_errcheck.h"

#include "p4model.h"

//typedef struct {
//  size_t w;
//  size_t h;
//  double pitch;
//  double yaw;
//  double near;
//  double far;
//
//  double xyz[3];
//  double rpy[3]; /* [deg] */
//} camera_sensor;

int main(int argc, char *argv[]) {

    if (argc < 16) {
      goto ERROR;
    }

    //std::string dirpath = "../../../resources";
    //std::string filename = "/obj/p4arm/p4arm.obj";
    const char* dirpath = argv[1];
    const char* filename = argv[2];
    const char* savepath = argv[3];
    const char* linkname = argv[4];
    const char* sensname = argv[5];
    const char* name = argv[6];

    //if (strcmp(dirpath, savepath) == 0) {
    //  fprintf(stderr, " -- res_dirpath and save_dirpath must not be same : save_dirpath:%s.\n", savepath);
    //  goto ERROR;
    //}

    if (strcmp(sensname, "Camera") != 0) {
      goto ERROR;
    }

    size_t w = strtoul(argv[7], NULL, 10);
    size_t h = strtoul(argv[8], NULL, 10);
    double vang = strtod(argv[9], NULL);
    double hang = strtod(argv[10], NULL);
    double near = strtod(argv[11], NULL);
    double far  = strtod(argv[12], NULL);

    double Px = strtod(argv[13], NULL);
    double Py = strtod(argv[14], NULL);
    double Pz = strtod(argv[15], NULL);

    double r  = strtod(argv[16], NULL);
    double p  = strtod(argv[17], NULL);
    double y  = strtod(argv[18], NULL);

    camera_sensor sens = {name, w, h, vang, hang, near, far, {Px,Py,Pz}, {r,p,y}};

    void* hdl;
    EFUNC(p4model_open(&hdl, dirpath, filename));
    EFUNC(p4model_add_camera(hdl, linkname, &sens));
    EFUNC(p4model_saveas(hdl, savepath));
    EFUNC(p4model_close(hdl));

    return 0;
 
ERROR:
    fprintf(stdout, "%s <res_dirpath> <filepath> <save_dirpath> linkname Camera name width height vang hang near far x y z r p y\n", argv[0]);
    fprintf(stdout, "   (rpy-->[deg])\n");

    return 1;
}


