#ifndef __IDMANAGER_H
#define __IDMANAGER_H

/**
\addtogroup cross-layers
\{
\addtogroup IDManager
\{
*/

#include "board.h"
#include "opendefs.h"

//=========================== define ==========================================

#define ACTION_YES      'Y'
#define ACTION_NO       'N'
#define ACTION_TOGGLE   'T'

//=========================== typedef =========================================

#if (NEW_DAG_BRIDGE == 1)
BEGIN_PACK

typedef struct {
   bool          isDAGroot;
   uint8_t       myPANID[2];
   uint8_t       my16bID[2];
   uint8_t       my64bID[8];
   uint8_t       myPrefix[8];
} debugIDManagerEntry_t;

END_PACK

typedef struct {
   bool          isDAGroot;
   open_addr_t   myPANID;
   open_addr_t   my16bID;
   open_addr_t   my64bID;
   open_addr_t   myPrefix;
} idmanager_vars_t;

//=========================== prototypes ======================================

void         idmanager_init(void);
bool         idmanager_getIsDAGroot(void);
void         idmanager_setIsDAGroot(bool newRole);
open_addr_t* idmanager_getMyID(uint8_t type);
owerror_t      idmanager_setMyID(open_addr_t* newID);
bool         idmanager_isMyAddress(open_addr_t* addr);
bool         idmanager_isMyAddressMasked(open_addr_t* addr);
void         idmanager_triggerAboutRoot(void);
uint16_t idmanager_getMyID16bits(void);
bool         debugPrint_id(void);

#else
typedef struct {
   bool          isDAGroot;
   bool          isBridge;
   open_addr_t   my16bID;
   open_addr_t   my64bID;
   open_addr_t   myPANID;
   open_addr_t   myPrefix;
} debugIDManagerEntry_t;

typedef struct {
   bool          isDAGroot;
   bool          isBridge;
   open_addr_t   my16bID;
   open_addr_t   my64bID;
   open_addr_t   myPANID;
   open_addr_t   myPrefix;
} idmanager_vars_t;

//=========================== prototypes ======================================

void         idmanager_init(void);
bool         idmanager_getIsDAGroot(void);
void         idmanager_setIsDAGroot(bool newRole);
bool         idmanager_getIsBridge(void);
void         idmanager_setIsBridge(bool newRole);
open_addr_t* idmanager_getMyID(uint8_t type);
owerror_t      idmanager_setMyID(open_addr_t* newID);
bool         idmanager_isMyAddress(open_addr_t* addr);
void         idmanager_triggerAboutRoot(void);
void         idmanager_triggerAboutBridge(void);

bool         debugPrint_id(void);

#endif

/**
\}
\}
*/

#endif
