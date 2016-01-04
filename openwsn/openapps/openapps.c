/**
\brief Applications running on top of the OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, September 2014.
*/

#include "board.h"
#include "opendefs.h"
#include "idmanager.h"

// CoAP
#include "c6t.h"
#include "cinfo.h"
#include "cleds.h"
#include "cwellknown.h"
// TCP
#include "techo.h"
// UDP
#include "uecho.h"
// OSENS
#include "osens_app.h"
#include "board.h"


//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

//=========================== private =========================================

void openapps_init(void) {
   // CoAP
   c6t_init();
   cinfo_init();
   cleds__init();
   cstorm_init();
   cwellknown_init();
   //rrt_init();
   // TCP
   techo_init();
   // UDP
   uecho_init();

   //sensor board
#if (MYLINKXS_SENSORS == 1)
   osens_app_init();
#endif

#if DAGROOT_ENABLE_ONSTARTUP
   idmanager_setIsDAGroot(TRUE);

#if (NEW_DAG_BRIDGE == 0)
   idmanager_setIsBridge(TRUE);
#endif
#endif



}
