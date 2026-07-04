/*
 * Track file loader. Two row formats are accepted (they can be mixed):
 *
 * 1. Device format -- rows exactly as written by the FlySight 2 logger
 *    (FS_Log_UpdateGNSS), so a real TRACK.CSV can be replayed unmodified:
 *
 *      $GNSS,2024-05-11T16:31:12.400Z,51.0521366,-114.0525577,1109.336,
 *            0.010,-0.011,0.017,1.541,2.008,0.223,15
 *
 *    (time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV; SI units.)
 *    Timestamps are made relative to the first row. gpsFix is assumed 3.
 *
 * 2. Simple format -- for synthetic tracks (see scripts/gen_jump.py);
 *    relative time in seconds and an explicit fix type:
 *
 *      t,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV,gpsFix
 *
 * Lines that are empty, start with '#', or start with an unrecognized '$'
 * sentence (e.g. $FLYS/$VAR/$COL/$UNIT headers) are skipped, as is a
 * column-header line.
 *
 * Derived fields (speed, gSpeed, heading) are computed from velN/velE/velD
 * the same way the GNSS receiver reports them: gSpeed = 2D speed, speed =
 * 3D speed (both cm/s), heading = atan2(velE, velN) in deg * 1e5, in
 * [0, 360).
 */

#ifndef TRACK_H_
#define TRACK_H_

#include "sim.h"

/* Returns 0 on success; *points is malloc'd (caller frees) */
int track_load(const char *path, Sim_TrackPoint_t **points, size_t *count);

#endif /* TRACK_H_ */
