/*
 * File:
 *    faV3GStatus.c
 *
 * Description:
 *    Show the status of all fadcs found in the crate
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-HallD.h"

int
main(int argc, char *argv[])
{

    int status;

    status = vmeOpenDefaultWindows();
    if(status != OK)
      goto CLOSE;

    vmeBusLock();
    /* Set the FADC structure pointer */
    faV3HallDInit( 3 << 19 , 1 << 19, 18, FAV3_INIT_SKIP | FAV3_INIT_SKIP_FIRMWARE_CHECK);

    faV3HallDGStatus(0);
    vmeBusUnlock();


 CLOSE:
    vmeCloseDefaultWindows();

    exit(0);
}
