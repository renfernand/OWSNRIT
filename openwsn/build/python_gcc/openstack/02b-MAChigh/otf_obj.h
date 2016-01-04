/**
DO NOT EDIT DIRECTLY!!

This file was 'objectified' by SCons as a pre-processing
step for the building a Python extension module.

This was done on 2015-10-30 10:49:00.666000.
*/
#ifndef __OTF_H
#define __OTF_H

/**
\addtogroup MAChigh
\{
\addtogroup otf
\{
*/

#include "opendefs_obj.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

#include "openwsnmodule_obj.h"
typedef struct OpenMote OpenMote;

//=========================== prototypes ======================================

// admin
void otf_init(OpenMote* self);
// notification from sixtop
void otf_notif_addedCell(OpenMote* self);
void otf_notif_removedCell(OpenMote* self);

/**
\}
\}
*/

#endif
