/*
 * File:
 *    faCheckConfigfile.c
 *
 * Description:
 *    Program check for error return from the config file format
 *
 *
 */


#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <libgen.h>

#include "fadc250Config.h"

int
main(int argc, char *argv[])
{
  char config_filename[256] = "";

  // taken from getcwd(3p)
  long size;
  char *buf;
  char *ptr;

  size = pathconf(".", _PC_PATH_MAX);

  if ((buf = (char *)malloc((size_t)size)) != NULL)
    ptr = getcwd(buf, (size_t)size);

  /* grab filename using arguments */
  if(argc == 2)
    {
      sprintf(config_filename, "%s/%s/nps-vme.cfg",
	      buf,
	      argv[1]);
    }
  else
    {
      printf("%s: Usage\n", argv[0]);
      printf("  %s   <configtype>\n", argv[0]);
      exit(-1);
    }

  fadc250InitGlobals();
  int err = fadc250ReadConfigFile(config_filename);

  printf("\n\n");
  if(err < 0)
    printf("ERROR!\n");
  else
    printf("SUCCESS!\n");

  printf(" file: %s\n", config_filename);
  printf(" fadc250ReadConfigFile returned %d\n\n", err);

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faCheckConfigfile "
  End:
*/
