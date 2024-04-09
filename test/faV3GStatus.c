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

int
main(int argc, char *argv[])
{

    int status;

    status = vmeOpenDefaultWindows();
    if(status != OK)
      goto CLOSE;

    vmeBusLock();
    /* Set the FADC structure pointer */
    faV3Init( 3 << 19 , 1 << 19, 18, FAV3_INIT_SKIP);

    faV3GStatus(0);
    vmeBusUnlock();


 CLOSE:
    vmeCloseDefaultWindows();

    exit(0);
}
