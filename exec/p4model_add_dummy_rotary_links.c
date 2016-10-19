
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

    if (argc < 4) {
      goto ERROR;
    }

    //std::string dirpath = "../../../resources";
    //std::string filename = "/obj/p4arm/p4arm.obj";
    const char* dirpath = argv[1];
    const char* filename = argv[2];
    const char* savepath = argv[3];

    void* hdl;
    EFUNC(p4model_open(&hdl, dirpath, filename));
    EFUNC(p4model_add_dummy_rotary_links(hdl));
    EFUNC(p4model_saveas(hdl, savepath));
    EFUNC(p4model_close(hdl));

    return 0;
 
ERROR:
    fprintf(stdout, "%s <res_dirpath> <filepath> <save_dirpath>\n", argv[0]);

    return 1;
}


