/*
 * File:
 *    faWriteConfigFile.c
 *
 * Description:
 *    Program to write a config file from a specified module
 *
 *
 */


#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <libgen.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3Config.h"

int
main(int argc, char *argv[])
{
  int32_t slot = 3;

  /* grab filename using arguments */
  if(argc == 2)
    {
      slot = atoi(argv[1]);
    }

  vmeOpen();
  faV3Init(slot << 19, 0, 1, 0);

  faV3InitGlobals();
  faV3DownloadAll();
  faV3UploadAllPrint();
  vmeClose();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faWriteConfigFile "
  End:
*/
