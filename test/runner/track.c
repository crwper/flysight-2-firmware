#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "gnss.h"
#include "sim.h"
#include "track.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* days since 1970-01-01 (Howard Hinnant's days_from_civil) */
static int64_t days_from_civil(int y, int m, int d)
{
	y -= m <= 2;
	{
		const int64_t era = (y >= 0 ? y : y - 399) / 400;
		const unsigned yoe = (unsigned) (y - era * 400);
		const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
		const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
		return era * 146097 + (int64_t) doe - 719468;
	}
}

static void fill_derived(FS_GNSS_Data_t *g, double velN, double velE, double velD)
{
	double gs = sqrt(velN * velN + velE * velE);        /* m/s */
	double sp = sqrt(gs * gs + velD * velD);            /* m/s */
	double hd = atan2(velE, velN) * 180.0 / M_PI;       /* deg */

	if (hd < 0) hd += 360.0;

	g->gSpeed  = (int32_t) llround(gs * 100.0);         /* cm/s */
	g->speed   = (int32_t) llround(sp * 100.0);         /* cm/s */
	g->heading = (int32_t) llround(hd * 100000.0);      /* deg * 1e5 */
}

static int parse_gnss_row(const char *line, FS_GNSS_Data_t *g, int64_t *abs_ms)
{
	int year, month, day, hour, min, sec, ms;
	double lat, lon, hMSL, velN, velE, velD, hAcc, vAcc, sAcc;
	int numSV;

	if (sscanf(line,
			"$GNSS,%d-%d-%dT%d:%d:%d.%dZ,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
			&year, &month, &day, &hour, &min, &sec, &ms,
			&lat, &lon, &hMSL, &velN, &velE, &velD,
			&hAcc, &vAcc, &sAcc, &numSV) != 17)
	{
		return -1;
	}

	memset(g, 0, sizeof(*g));

	g->year  = (uint16_t) year;
	g->month = (uint8_t) month;
	g->day   = (uint8_t) day;
	g->hour  = (uint8_t) hour;
	g->min   = (uint8_t) min;
	g->sec   = (uint8_t) sec;
	g->nano  = ms * 1000000;

	g->lat   = (int32_t) llround(lat * 1e7);
	g->lon   = (int32_t) llround(lon * 1e7);
	g->hMSL  = (int32_t) llround(hMSL * 1000.0);   /* mm */

	g->velN  = (int32_t) llround(velN * 1000.0);   /* mm/s */
	g->velE  = (int32_t) llround(velE * 1000.0);
	g->velD  = (int32_t) llround(velD * 1000.0);

	g->hAcc  = (uint32_t) llround(hAcc * 1000.0);  /* mm */
	g->vAcc  = (uint32_t) llround(vAcc * 1000.0);
	g->sAcc  = (uint32_t) llround(sAcc * 1000.0);  /* mm/s */

	g->numSV  = (uint8_t) numSV;
	g->gpsFix = 3;

	fill_derived(g, velN, velE, velD);

	*abs_ms = (days_from_civil(year, month, day) * 86400LL
			+ hour * 3600LL + min * 60LL + sec) * 1000LL + ms;

	return 0;
}

static int parse_simple_row(const char *line, FS_GNSS_Data_t *g, double *t_s)
{
	double lat, lon, hMSL, velN, velE, velD, hAcc, vAcc, sAcc;
	int numSV, gpsFix;

	if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d",
			t_s, &lat, &lon, &hMSL, &velN, &velE, &velD,
			&hAcc, &vAcc, &sAcc, &numSV, &gpsFix) != 12)
	{
		return -1;
	}

	memset(g, 0, sizeof(*g));

	g->iTOW  = (uint32_t) llround(*t_s * 1000.0);

	g->lat   = (int32_t) llround(lat * 1e7);
	g->lon   = (int32_t) llround(lon * 1e7);
	g->hMSL  = (int32_t) llround(hMSL * 1000.0);

	g->velN  = (int32_t) llround(velN * 1000.0);
	g->velE  = (int32_t) llround(velE * 1000.0);
	g->velD  = (int32_t) llround(velD * 1000.0);

	g->hAcc  = (uint32_t) llround(hAcc * 1000.0);
	g->vAcc  = (uint32_t) llround(vAcc * 1000.0);
	g->sAcc  = (uint32_t) llround(sAcc * 1000.0);

	g->numSV  = (uint8_t) numSV;
	g->gpsFix = (uint8_t) gpsFix;

	fill_derived(g, velN, velE, velD);

	return 0;
}

int track_load(const char *path, Sim_TrackPoint_t **points, size_t *count)
{
	FILE *fp;
	char line[512];
	size_t cap = 1024, n = 0;
	Sim_TrackPoint_t *pts;
	int64_t first_abs_ms = 0;
	int have_first_abs = 0;
	int lineno = 0;

	fp = fopen(path, "rb");
	if (!fp)
	{
		fprintf(stderr, "error: cannot open track file '%s'\n", path);
		return -1;
	}

	pts = malloc(cap * sizeof(*pts));

	while (fgets(line, sizeof(line), fp))
	{
		FS_GNSS_Data_t g;
		uint64_t t_ns;

		++lineno;

		if (line[0] == '\0' || line[0] == '\n' || line[0] == '\r' || line[0] == '#')
			continue;

		if (line[0] == '$')
		{
			int64_t abs_ms;

			if (strncmp(line, "$GNSS,", 6) != 0)
				continue; /* $FLYS, $VAR, $COL, $UNIT, ... */

			if (parse_gnss_row(line, &g, &abs_ms) != 0)
			{
				fprintf(stderr, "error: %s:%d: bad $GNSS row\n", path, lineno);
				free(pts);
				fclose(fp);
				return -1;
			}

			if (!have_first_abs)
			{
				first_abs_ms = abs_ms;
				have_first_abs = 1;
			}

			t_ns = (uint64_t) (abs_ms - first_abs_ms) * 1000000ULL;
		}
		else if (isdigit((unsigned char) line[0]) || line[0] == '-' || line[0] == '+' || line[0] == '.')
		{
			double t_s;

			if (parse_simple_row(line, &g, &t_s) != 0)
			{
				fprintf(stderr, "error: %s:%d: bad simple row (need 12 columns)\n", path, lineno);
				free(pts);
				fclose(fp);
				return -1;
			}

			t_ns = (uint64_t) llround(t_s * 1e9);
		}
		else
		{
			continue; /* column-header or other text line */
		}

		if (n == cap)
		{
			cap *= 2;
			pts = realloc(pts, cap * sizeof(*pts));
		}

		pts[n].t_ns = t_ns;
		pts[n].data = g;
		++n;
	}

	fclose(fp);

	if (n == 0)
	{
		fprintf(stderr, "error: %s: no track points found\n", path);
		free(pts);
		return -1;
	}

	*points = pts;
	*count = n;
	return 0;
}
