/* stdio-backed FatFS fake (see ff.h) */

#include <stdio.h>

#include "ff.h"

FRESULT f_open(FIL *fp, const char *path, unsigned char mode)
{
	const char *m = (mode & FA_WRITE) ? "wb" : "rb";

	fp->fp = fopen(path, m);
	return fp->fp ? FR_OK : FR_NO_FILE;
}

FRESULT f_close(FIL *fp)
{
	if (fp->fp)
	{
		fclose(fp->fp);
		fp->fp = NULL;
	}
	return FR_OK;
}

char *f_gets(char *buff, int len, FIL *fp)
{
	return fgets(buff, len, fp->fp);
}

int f_puts(const char *str, FIL *fp)
{
	return fputs(str, fp->fp);
}

int f_eof(FIL *fp)
{
	int c = fgetc(fp->fp);

	if (c == EOF) return 1;
	ungetc(c, fp->fp);
	return 0;
}
