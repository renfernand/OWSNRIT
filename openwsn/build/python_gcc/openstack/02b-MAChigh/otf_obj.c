/**
DO NOT EDIT DIRECTLY!!

This file was 'objectified' by SCons as a pre-processing
step for the building a Python extension module.

This was done on 2015-10-30 10:49:00.338000.
*/
#include "opendefs_obj.h"
#include "otf_obj.h"
#include "neighbors_obj.h"
#include "sixtop_obj.h"
#include "scheduler_obj.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

void otf_addCell_task(OpenMote* self);
void otf_removeCell_task(OpenMote* self);

//=========================== public ==========================================

void otf_init(OpenMote* self) {
}

void otf_notif_addedCell(OpenMote* self) {
 scheduler_push_task(self, otf_addCell_task,TASKPRIO_OTF);
}

void otf_notif_removedCell(OpenMote* self) {
 scheduler_push_task(self, otf_removeCell_task,TASKPRIO_OTF);
}

//=========================== private =========================================

void otf_addCell_task(OpenMote* self) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   
   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(self, &neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }
   
   // call sixtop
 sixtop_addCells(self, 
      &neighbor,
      1
   );
}

void otf_removeCell_task(OpenMote* self) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   
   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(self, &neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }
   
   // call sixtop
 sixtop_removeCell(self, 
      &neighbor
   );
}