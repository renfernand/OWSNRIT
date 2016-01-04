/**
DO NOT EDIT DIRECTLY!!

This file was 'objectified' by SCons as a pre-processing
step for the building a Python extension module.

This was done on 2015-10-30 10:48:19.964000.
*/
#ifndef __CEXAMPLE_H
#define __CEXAMPLE_H

/**
\addtogroup AppUdp
\{
\addtogroup cexample
\{
*/
#include "opencoap_obj.h"
//=========================== define ==========================================

//=========================== typedef =========================================

typedef struct {
   coap_resource_desc_t desc;
   opentimer_id_t       timerId;
} cexample_vars_t;

//=========================== variables =======================================

#include "openwsnmodule_obj.h"
typedef struct OpenMote OpenMote;

//=========================== prototypes ======================================

void cexample_init(OpenMote* self);

/**
\}
\}
*/

#endif
