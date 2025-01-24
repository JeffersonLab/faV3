/*
 * File:
 *    faV3GDACTest.c
 *
 * Description:
 *    Test Writing and Reading to Channel DACs
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

    faV3Init(3 << 19, 1<<19, 2,
		  FAV3_INIT_VXS_CLKSRC | FAV3_INIT_EXT_SYNCRESET | FAV3_INIT_VXS_TRIG);

    int ifa = 0;
    char serial_number[1024];
    for(ifa = 0; ifa < faV3GetN(); ifa++)
      {
	faV3GetSerialNumber(faV3Slot(ifa), (char**)&serial_number);
	printf(">%s<\n", serial_number);
      }
    vmeBusUnlock();


 CLOSE:
    vmeCloseDefaultWindows();

    exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faV3DACTest "
  End:
*/
