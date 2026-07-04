/*
 * Host-build stand-in for FatFS (ff.h), backed by C stdio.
 *
 * Only the surface used by config.c is provided. Paths are passed through
 * to the host filesystem unchanged, so FS_Config_Read() can be pointed at
 * any config.txt on disk.
 */

#ifndef FAKE_FF_H_
#define FAKE_FF_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int UINT;

typedef enum
{
	FR_OK = 0,
	FR_DISK_ERR,
	FR_NO_FILE
} FRESULT;

#define FA_READ          0x01
#define FA_WRITE         0x02
#define FA_CREATE_ALWAYS 0x08

typedef struct
{
	FILE *fp;
} FIL;

FRESULT f_open(FIL *fp, const char *path, unsigned char mode);
FRESULT f_close(FIL *fp);
char   *f_gets(char *buff, int len, FIL *fp);
int     f_puts(const char *str, FIL *fp);
int     f_eof(FIL *fp);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_FF_H_ */
