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

#include "faV3Config.h"

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
      sprintf(config_filename, "%s/%s",
	      buf,
	      argv[1]);
    }
  else
    {
      printf("%s: Usage\n", argv[0]);
      printf("  %s   <configtype>\n", argv[0]);
      exit(-1);
    }

  faV3InitGlobals();
  int err = faV3ReadConfigFile(config_filename);

  printf("\n\n");
  if(err < 0)
    printf("ERROR!\n");
  else
    printf("SUCCESS!\n");

  printf(" file: %s\n", config_filename);
  printf(" faV3ReadConfigFile returned %d\n\n", err);

  extern int faV3ID[21];
  extern int nfaV3;

  nfaV3 = 1;
  faV3ID[0] = 3;

  char bigstring[12000];

  faV3ConfigToString(bigstring, 12000);
  printf("%s", bigstring);

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faCheckConfigfile "
  End:
*/
