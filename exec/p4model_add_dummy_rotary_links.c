
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
    const char* respath = argv[3];
    const char* save_filerootpath = NULL;

    if (argc >= 5) {
      save_filerootpath = argv[4];
      if (strcmp(save_filerootpath, "/") == 0) {
        fprintf(stderr, " -- res_dirpath and (save_respath + save_filerootpath) must not be same : save_respath:%s.\n", respath);
        goto ERROR;
      }
    } else {
      if (strcmp(dirpath, respath) == 0) {
        fprintf(stderr, " -- res_dirpath and save_respath must not be same : save_respath:%s.\n", respath);
        goto ERROR;
      }
    }

    void* hdl;
    EFUNC(p4model_open(&hdl, dirpath, filename));
    EFUNC(p4model_add_dummy_rotary_links(hdl));

    if (save_filerootpath) {
      EFUNC(p4model_saveas2(hdl, respath, save_filerootpath));
    } else {
      EFUNC(p4model_saveas(hdl, respath));
    }

    EFUNC(p4model_close(hdl));

    return 0;
 
ERROR:
    fprintf(stdout, "%s <res_dirpath> <filepath> <save_res_dirpath> [save_filerootpath]\n", argv[0]);

    return 1;
}


