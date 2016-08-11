#include "opendefs.h"
#include "board.h"
#include "neighbors.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "idmanager.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "IEEE802154RIT.h"

//=========================== variables =======================================

neighbors_vars_t neighbors_vars;
extern const forceNeighbor_t tableNeighbor[];
extern const uint8_t MoteBestChanTable[];
extern RIT_stats_t ritstat;


#if (IEEE802154E_AMCA == 1)
//esta tabela de vizinhos eh diferentes do RPL pois o do rpl somente tem os vizinhos que entraram no RPL...
//esta outra tem a ver com todos os vizinhos que mandaram ola para o mote
macneighbors_vars_t macneighbors_vars;
#endif
//=========================== prototypes ======================================
bool macisThisRowMatching(open_addr_t* address, uint8_t rowNumber);
void forceNeighborhood(uint8_t myaddr);
uint8_t getMotePos(uint8_t addr,uint8_t len);

void registerNewNeighbor(
        open_addr_t* neighborID,
        int8_t       rssi,
        asn_t*       asnTimestamp,
        bool         joinPrioPresent,
        uint8_t      joinPrio
     );
bool isNeighbor(open_addr_t* neighbor);
void removeNeighbor(uint8_t neighborIndex);
bool isThisRowMatching(
        open_addr_t* address,
        uint8_t      rowNumber
     );

//=========================== public ==========================================

/**
\brief Initializes this module.
*/
void neighbors_init() {
#if (IEEE802154E_AMCA == 1)
   open_addr_t *myaddr = idmanager_getMyID(ADDR_16B);
#endif
   
   // clear module variables
   memset(&neighbors_vars,0,sizeof(neighbors_vars_t));
   
   // set myDAGrank
   if (idmanager_getIsDAGroot()==TRUE) {
      neighbors_vars.myDAGrank=0;
   } else {
      neighbors_vars.myDAGrank=DEFAULTDAGRANK;
   }

#if (IEEE802154E_AMCA == 1)
   // clear module variables
   memset(&macneighbors_vars,0,sizeof(macneighbors_vars_t));

   macneighbors_vars.mybestchan = macneighbors_getFixedBestChan(myaddr->addr_16b[1]);
   forceNeighborhood(myaddr->addr_16b[1]);
#endif

}

/**
\brief Retrieve this mote's current DAG rank.

\returns This mote's current DAG rank.
*/
dagrank_t neighbors_getMyDAGrank() {
   return neighbors_vars.myDAGrank;
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t neighbors_getNumNeighbors() {
   uint8_t i;
   uint8_t returnVal;
   
   returnVal=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         returnVal++;
      }
   }
   return returnVal;
}

/**
\brief Retrieve my preferred parent's EUI64 address.

\param[out] addressToWrite Where to write the preferred parent's address to.
*/
bool neighbors_getPreferredParentEui64(open_addr_t* addressToWrite) {
   uint8_t   i;
   bool      foundPreferred;
   uint8_t   numNeighbors;
   dagrank_t minRankVal;
   uint8_t   minRankIdx;
   
   addressToWrite->type = ADDR_NONE;
   
   foundPreferred       = FALSE;
   numNeighbors         = 0;
   minRankVal           = MAXDAGRANK;
   minRankIdx           = MAXNUMNEIGHBORS+1;
   
   //===== step 1. Try to find preferred parent
   for (i=0; i<MAXNUMNEIGHBORS; i++) {
      if (neighbors_vars.neighbors[i].used==TRUE){
         if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
            memcpy(addressToWrite,&(neighbors_vars.neighbors[i].addr_64b),sizeof(open_addr_t));
            addressToWrite->type=ADDR_64B;
            foundPreferred=TRUE;
         }
         // identify neighbor with lowest rank
         if (neighbors_vars.neighbors[i].DAGrank < minRankVal) {
            minRankVal=neighbors_vars.neighbors[i].DAGrank;
            minRankIdx=i;
         }
         numNeighbors++;
      }
   }
   
   //===== step 2. (backup) Promote neighbor with min rank to preferred parent
   if (foundPreferred==FALSE && numNeighbors > 0){
      // promote neighbor
      neighbors_vars.neighbors[minRankIdx].parentPreference       = MAXPREFERENCE;
      neighbors_vars.neighbors[minRankIdx].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[minRankIdx].switchStabilityCounter = 0;
      // return its address
      memcpy(addressToWrite,&(neighbors_vars.neighbors[minRankIdx].addr_64b),sizeof(open_addr_t));
      addressToWrite->type=ADDR_64B;
      foundPreferred=TRUE;         
   }
   
   return foundPreferred;
}

/**
\brief Find neighbor to which to send KA.

This function iterates through the neighbor table and identifies the neighbor
we need to send a KA to, if any. This neighbor satisfies the following
conditions:
- it is one of our preferred parents
- we haven't heard it for over kaPeriod

\param[in] kaPeriod The maximum number of slots I'm allowed not to have heard
   it.

\returns A pointer to the neighbor's address, or NULL if no KA is needed.
*/
open_addr_t* neighbors_getKANeighbor(uint16_t kaPeriod) {
   uint8_t         i;
   uint16_t        timeSinceHeard;
   open_addr_t*    addrPreferred;
   open_addr_t*    addrOther;
   
   // initialize
   addrPreferred = NULL;
   addrOther     = NULL;
   
   // scan through the neighbor table, and populate addrPreferred and addrOther
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==1) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>kaPeriod) {
            // this neighbor needs to be KA'ed to
            if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
               // its a preferred parent
               addrPreferred = &(neighbors_vars.neighbors[i].addr_64b);
            } else {
               // its not a preferred parent
               // Note: commented out since policy is not to KA to non-preferred parents
               // addrOther =     &(neighbors_vars.neighbors[i].addr_64b);
            }
         }
      }
   }
   
   // return the EUI64 of the most urgent KA to send:
   // - if available, preferred parent
   // - if not, non-preferred parent
   if        (addrPreferred!=NULL) {
      return addrPreferred;
   } else if (addrOther!=NULL) {
      return addrOther;
   } else {
      return NULL;
   }
}

//===== interrogators

/**
\brief Indicate whether some neighbor is a stable neighbor

\param[in] address The address of the neighbor, a full 128-bit IPv6 addres.

\returns TRUE if that neighbor is stable, FALSE otherwise.
*/
bool neighbors_isStableNeighbor(open_addr_t* address) {
   uint8_t     i;
   open_addr_t temp_addr_64b;
   open_addr_t temp_prefix;
   bool        returnVal;
   
   // by default, not stable
   returnVal  = FALSE;
   
   // but neighbor's IPv6 address in prefix and EUI64
   switch (address->type) {
      case ADDR_128B:
         packetfunctions_ip128bToMac64b(address,&temp_prefix,&temp_addr_64b);
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)0);
         return returnVal;
   }
   
   // iterate through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(&temp_addr_64b,i) && neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
         returnVal  = TRUE;
         break;
      }
   }
   
   return returnVal;
}

/**
\brief Indicate whether message pending is for this probe.

\param[in] address The EUI64 address of the Pending message and the Frame Received.

\returns TRUE if the pending msg is for this probe, FALSE otherwise.
*/
bool neighbors_ChkMsgPendIsforThisProbe(open_addr_t*  destinationAddress , open_addr_t*  probeaddress)
{
   uint8_t i;
   bool    returnVal;

   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();

   returnVal = packetfunctions_sameAddress(destinationAddress,probeaddress);

   ENABLE_INTERRUPTS();
   return returnVal;
}

/**
\brief Indicate whether some neighbor is a preferred neighbor.

\param[in] address The EUI64 address of the neighbor.

\returns TRUE if that neighbor is preferred, FALSE otherwise.
*/
bool neighbors_isPreferredParent(open_addr_t* address) {
   uint8_t i;
   bool    returnVal;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // by default, not preferred
   returnVal = FALSE;
   
   // iterate through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(address,i) && neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
         returnVal  = TRUE;
         break;
      }
   }
   
   ENABLE_INTERRUPTS();
   return returnVal;
}

/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithLowerDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank < neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}


/* RFF - Retorno da tabela de vizinhos os vizinhos validos que estao estáveis.
 * Tabela tem 10 elementos.
 */
bool neighbors_isValidNeighbor(uint8_t index) {
   bool    returnVal;

   if ((neighbors_vars.neighbors[index].used==TRUE) && (neighbors_vars.neighbors[index].stableNeighbor == TRUE)) {
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }

   return returnVal;
}

uint8_t neighbors_howmanyIhave(void){

	uint8_t numTargetParents=0;
	uint8_t nbrIdx;

	for (nbrIdx=0;nbrIdx<MAXNUMNEIGHBORS;nbrIdx++) {
	  if ((neighbors_isValidNeighbor(nbrIdx))==TRUE) {
	     //neighbors_getNeighbor(&address,ADDR_64B,nbrIdx);
	     numTargetParents++;
	  }
	}
    return numTargetParents;
}
/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithHigherDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank >= neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}

//===== updating neighbor information

/**
\brief Indicate some (non-ACK) packet was received from a neighbor.

This function should be called for each received (non-ACK) packet so neighbor
statistics in the neighbor table can be updated.

The fields which are updated are:
- numRx
- rssi
- asn
- stableNeighbor
- switchStabilityCounter

\param[in] l2_src MAC source address of the packet, i.e. the neighbor who sent
   the packet just received.
\param[in] rssi   RSSI with which this packet was received.
\param[in] asnTs  ASN at which this packet was received.
\param[in] joinPrioPresent Whether a join priority was present in the received
   packet.
\param[in] joinPrio The join priority present in the packet, if any.
*/
void neighbors_indicateRx(open_addr_t* l2_src,
                          int8_t       rssi,
                          asn_t*       asnTs,
                          bool         joinPrioPresent,
                          uint8_t      joinPrio) {
   uint8_t i;
   bool    newNeighbor;
   
   // update existing neighbor
   newNeighbor = TRUE;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(l2_src,i)) {
         
         // this is not a new neighbor
         newNeighbor = FALSE;
         
         // update numRx, rssi, asn
         neighbors_vars.neighbors[i].numRx++;
         neighbors_vars.neighbors[i].rssi=rssi;
         memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
         //update jp
         if (joinPrioPresent==TRUE){
            neighbors_vars.neighbors[i].joinPrio=joinPrio;
         }
         
         // update stableNeighbor, switchStabilityCounter
         if (neighbors_vars.neighbors[i].stableNeighbor==FALSE) {
            if (neighbors_vars.neighbors[i].rssi>BADNEIGHBORMAXRSSI) {
               neighbors_vars.neighbors[i].switchStabilityCounter++;
               if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                  neighbors_vars.neighbors[i].switchStabilityCounter=0;
                  neighbors_vars.neighbors[i].stableNeighbor=TRUE;
               }
            } else {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
            }
         } else if (neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
            if (neighbors_vars.neighbors[i].rssi<GOODNEIGHBORMINRSSI) {
               neighbors_vars.neighbors[i].switchStabilityCounter++;
               if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                  neighbors_vars.neighbors[i].switchStabilityCounter=0;
                   neighbors_vars.neighbors[i].stableNeighbor=FALSE;
               }
            } else {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
            }
         }
         
         // stop looping
         break;
      }
   }
   
   // register new neighbor
   if (newNeighbor==TRUE) {
      registerNewNeighbor(l2_src, rssi, asnTs, joinPrioPresent,joinPrio);
   }
}

/**
\brief Indicate some packet was sent to some neighbor.

This function should be called for each transmitted (non-ACK) packet so
neighbor statistics in the neighbor table can be updated.

The fields which are updated are:
- numTx
- numTxACK
- asn

\param[in] l2_dest MAC destination address of the packet, i.e. the neighbor
   who I just sent the packet to.
\param[in] numTxAttempts Number of transmission attempts to this neighbor.
\param[in] was_finally_acked TRUE iff the packet was ACK'ed by the neighbor
   on final transmission attempt.
\param[in] asnTs ASN of the last transmission attempt.
*/
void neighbors_indicateTx(open_addr_t* l2_dest,
                          uint8_t      numTxAttempts,
                          bool         was_finally_acked,
                          asn_t*       asnTs) {
   uint8_t i;
   // don't run through this function if packet was sent to broadcast address
   if (packetfunctions_isBroadcastMulticast(l2_dest)==TRUE) {
      return;
   }
   
   // loop through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(l2_dest,i)) {
         // handle roll-over case
        
          if (neighbors_vars.neighbors[i].numTx>(0xff-numTxAttempts)) {
        	  neighbors_vars.neighbors[i].numWraps++; //counting the number of times that tx wraps.
        	  neighbors_vars.neighbors[i].numTx/=2;
              neighbors_vars.neighbors[i].numTxACK/=2;
           }
         // update statistics
        neighbors_vars.neighbors[i].numTx += numTxAttempts; 
        
        if (was_finally_acked==TRUE) {
            neighbors_vars.neighbors[i].numTxACK++;
            memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
        }
        break;
      }
   }

#if 1 //(DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
	   uint8_t pos=0;

		rffbuf[pos++]= RFF_ICMPv6RPL_TX;
		rffbuf[pos++]= 0x10;
		 pos = printvar((uint8_t *)&neighbors_vars.myDAGrank,sizeof(uint16_t),rffbuf,pos);
		 //pos = printvar((uint8_t *)&neighbors_vars.dio->rank,sizeof(uint16_t),rffbuf,pos);

		for (i=0;i<MAXNUMNEIGHBORS;i++) {
			 if (neighbors_vars.neighbors[i].used) {
				 pos = printaddress(neighbors_vars.neighbors[i].addr_64b,&rffbuf[0],pos);
				 rffbuf[pos++]= neighbors_vars.neighbors[i].parentPreference;
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].DAGrank,sizeof(uint16_t),rffbuf,pos);
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].numTx,sizeof(uint8_t),rffbuf,pos);
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].numRx,sizeof(uint8_t),rffbuf,pos);
			}
		}

	   openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif
}

/**
\brief Indicate I just received a RPL DIO from a neighbor.

This function should be called for each received a DIO is received so neighbor
routing information in the neighbor table can be updated.

The fields which are updated are:
- DAGrank

\param[in] msg The received message with msg->payload pointing to the DIO
   header.
*/
void neighbors_indicateRxDIO(OpenQueueEntry_t* msg) {
   uint8_t          i;
  
   // take ownership over the packet
   msg->owner = COMPONENT_NEIGHBORS;
   
   // update rank of that neighbor in table
   neighbors_vars.dio = (icmpv6rpl_dio_ht*)(msg->payload);
   if (isNeighbor(&(msg->l2_nextORpreviousHop))==TRUE) {
      for (i=0;i<MAXNUMNEIGHBORS;i++) {
         if (isThisRowMatching(&(msg->l2_nextORpreviousHop),i)) {
            if (
                  neighbors_vars.dio->rank > neighbors_vars.neighbors[i].DAGrank &&
                  neighbors_vars.dio->rank - neighbors_vars.neighbors[i].DAGrank >(DEFAULTLINKCOST*2*MINHOPRANKINCREASE)
               ) {
                // the new DAGrank looks suspiciously high, only increment a bit
                neighbors_vars.neighbors[i].DAGrank += (DEFAULTLINKCOST*2*MINHOPRANKINCREASE);
                openserial_printError(COMPONENT_NEIGHBORS,ERR_LARGE_DAGRANK,
                               (errorparameter_t)neighbors_vars.dio->rank,
                               (errorparameter_t)neighbors_vars.neighbors[i].DAGrank);
            } else {
               neighbors_vars.neighbors[i].DAGrank = neighbors_vars.dio->rank;
            }
            break;
         }
      }
   } 
   // update my routing information
   neighbors_updateMyDAGrankAndNeighborPreference(); 

#if 1
   {
	   uint8_t pos=0;

		rffbuf[pos++]= RFF_ICMPv6RPL_RX;
		rffbuf[pos++]= 0x20;
		 pos = printvar((uint8_t *)&neighbors_vars.myDAGrank,sizeof(uint16_t),rffbuf,pos);
		 //pos = printvar((uint8_t *)&neighbors_vars.dio->rank,sizeof(uint16_t),rffbuf,pos);

		for (i=0;i<MAXNUMNEIGHBORS;i++) {
			 if (neighbors_vars.neighbors[i].used) {
				 pos = printaddress(neighbors_vars.neighbors[i].addr_64b,&rffbuf[0],pos);
				 rffbuf[pos++]= neighbors_vars.neighbors[i].parentPreference;
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].DAGrank,sizeof(uint16_t),rffbuf,pos);
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].numTx,sizeof(uint8_t),rffbuf,pos);
				 pos = printvar((uint8_t *)&neighbors_vars.neighbors[i].numRx,sizeof(uint8_t),rffbuf,pos);
			}
		}

		 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

}

//===== write addresses

/**
\brief Write the 64-bit address of some neighbor to some location.
*/
void  neighbors_getNeighbor(open_addr_t* address, uint8_t addr_type, uint8_t index){
   switch(addr_type) {
      case ADDR_64B:
         memcpy(&(address->addr_64b),&(neighbors_vars.neighbors[index].addr_64b.addr_64b),LENGTH_ADDR64b);
         address->type=ADDR_64B;
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)addr_type,
                               (errorparameter_t)1);
         break; 
   }
}

//===== managing routing info

/**
\brief Update my DAG rank and neighbor preference.

Call this function whenever some data is changed that could cause this mote's
routing decisions to change. Examples are:
- I received a DIO which updated by neighbor table. If this DIO indicated a
  very low DAGrank, I may want to change by routing parent.
- I became a DAGroot, so my DAGrank should be 0.
*/
void neighbors_updateMyDAGrankAndNeighborPreference() {
   uint8_t   i;
   uint16_t  rankIncrease;
   uint32_t  tentativeDAGrank; // 32-bit since is used to sum
   uint8_t   prefParentIdx;
   bool      prefParentFound;
   
   // if I'm a DAGroot, my DAGrank is always 0
   if ((idmanager_getIsDAGroot())==TRUE) {
      neighbors_vars.myDAGrank=0;
      return;
   }
   
   // reset my DAG rank to max value. May be lowered below.
   neighbors_vars.myDAGrank  = MAXDAGRANK;
   
   // by default, I haven't found a preferred parent
   prefParentFound           = FALSE;
   prefParentIdx             = 0;
   
   // loop through neighbor table, update myDAGrank
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         
         // reset parent preference
         neighbors_vars.neighbors[i].parentPreference=0;
         
         // calculate link cost to this neighbor
         if (neighbors_vars.neighbors[i].numTxACK==0) {
            rankIncrease = DEFAULTLINKCOST*2*MINHOPRANKINCREASE;
         } else {
            //6TiSCH minimal draft using OF0 for rank computation
            rankIncrease = (uint16_t)((((float)neighbors_vars.neighbors[i].numTx)/((float)neighbors_vars.neighbors[i].numTxACK))*2*MINHOPRANKINCREASE);
         }
         
         tentativeDAGrank = neighbors_vars.neighbors[i].DAGrank+rankIncrease;
         if ( tentativeDAGrank<neighbors_vars.myDAGrank &&
              tentativeDAGrank<MAXDAGRANK) {
            // found better parent, lower my DAGrank
            neighbors_vars.myDAGrank   = tentativeDAGrank;
            prefParentFound            = TRUE;
            prefParentIdx              = i;
         }
      }
   } 
   
   // update preferred parent
   if (prefParentFound) {
      neighbors_vars.neighbors[prefParentIdx].parentPreference       = MAXPREFERENCE;
      neighbors_vars.neighbors[prefParentIdx].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[prefParentIdx].switchStabilityCounter = 0;
   }
}

//===== maintenance

void  neighbors_removeOld() {
   uint8_t    i;
   uint16_t   timeSinceHeard;
   
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==1) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>DESYNCTIMEOUT) {
            removeNeighbor(i);
         }
      }
   } 
}

//===== debug

/**
\brief Triggers this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_neighbors() {
   debugNeighborEntry_t temp;
   neighbors_vars.debugRow=(neighbors_vars.debugRow+1)%MAXNUMNEIGHBORS;
   temp.row=neighbors_vars.debugRow;
   temp.neighborEntry=neighbors_vars.neighbors[neighbors_vars.debugRow];
   openserial_printStatus(STATUS_NEIGHBORS,(uint8_t*)&temp,sizeof(debugNeighborEntry_t));
   return TRUE;
}

//=========================== private =========================================

void registerNewNeighbor(open_addr_t* address,
                         int8_t       rssi,
                         asn_t*       asnTimestamp,
                         bool         joinPrioPresent,
                         uint8_t      joinPrio) {
   uint8_t  i,j;
   bool     iHaveAPreferedParent;
   // filter errors
   if (address->type!=ADDR_64B) {
      openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)address->type,
                            (errorparameter_t)2);
      return;
   }
   // add this neighbor
   if (isNeighbor(address)==FALSE) {
      i=0;
      while(i<MAXNUMNEIGHBORS) {
         if (neighbors_vars.neighbors[i].used==FALSE) {
            // add this neighbor
            neighbors_vars.neighbors[i].used                   = TRUE;
            neighbors_vars.neighbors[i].parentPreference       = 0;
            // neighbors_vars.neighbors[i].stableNeighbor         = FALSE;
            // Note: all new neighbors are consider stable
            neighbors_vars.neighbors[i].stableNeighbor         = TRUE;
            neighbors_vars.neighbors[i].switchStabilityCounter = 0;
            memcpy(&neighbors_vars.neighbors[i].addr_64b,address,sizeof(open_addr_t));
            neighbors_vars.neighbors[i].DAGrank                = DEFAULTDAGRANK;
            neighbors_vars.neighbors[i].rssi                   = rssi;
            neighbors_vars.neighbors[i].numRx                  = 1;
            neighbors_vars.neighbors[i].numTx                  = 0;
            neighbors_vars.neighbors[i].numTxACK               = 0;
            memcpy(&neighbors_vars.neighbors[i].asn,asnTimestamp,sizeof(asn_t));
            //update jp
            if (joinPrioPresent==TRUE){
               neighbors_vars.neighbors[i].joinPrio=joinPrio;
            }
            
            // do I already have a preferred parent ? -- TODO change to use JP
            iHaveAPreferedParent = FALSE;
            for (j=0;j<MAXNUMNEIGHBORS;j++) {
               if (neighbors_vars.neighbors[j].parentPreference==MAXPREFERENCE) {
                  iHaveAPreferedParent = TRUE;
               }
            }
            // if I have none, and I'm not DAGroot, the new neighbor is my preferred
            if (iHaveAPreferedParent==FALSE && idmanager_getIsDAGroot()==FALSE) {      
               neighbors_vars.neighbors[i].parentPreference     = MAXPREFERENCE;
            }
            break;
         }
         i++;
      }
      if (i==MAXNUMNEIGHBORS) {
         openserial_printError(COMPONENT_NEIGHBORS,ERR_NEIGHBORS_FULL,
                               (errorparameter_t)MAXNUMNEIGHBORS,
                               (errorparameter_t)0);
         return;
      }
   }
}

bool isNeighbor(open_addr_t* neighbor) {
   uint8_t i=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(neighbor,i)) {
         return TRUE;
      }
   }
   return FALSE;
}

void removeNeighbor(uint8_t neighborIndex) {
   neighbors_vars.neighbors[neighborIndex].used                      = FALSE;
   neighbors_vars.neighbors[neighborIndex].parentPreference          = 0;
   neighbors_vars.neighbors[neighborIndex].stableNeighbor            = FALSE;
   neighbors_vars.neighbors[neighborIndex].switchStabilityCounter    = 0;
   //neighbors_vars.neighbors[neighborIndex].addr_16b.type           = ADDR_NONE; // to save RAM
   neighbors_vars.neighbors[neighborIndex].addr_64b.type             = ADDR_NONE;
   //neighbors_vars.neighbors[neighborIndex].addr_128b.type          = ADDR_NONE; // to save RAM
   neighbors_vars.neighbors[neighborIndex].DAGrank                   = DEFAULTDAGRANK;
   neighbors_vars.neighbors[neighborIndex].rssi                      = 0;
   neighbors_vars.neighbors[neighborIndex].numRx                     = 0;
   neighbors_vars.neighbors[neighborIndex].numTx                     = 0;
   neighbors_vars.neighbors[neighborIndex].numTxACK                  = 0;
   neighbors_vars.neighbors[neighborIndex].asn.bytes0and1            = 0;
   neighbors_vars.neighbors[neighborIndex].asn.bytes2and3            = 0;
   neighbors_vars.neighbors[neighborIndex].asn.byte4                 = 0;
}

//=========================== helpers =========================================

bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
   switch (address->type) {
      case ADDR_64B:
         return neighbors_vars.neighbors[rowNumber].used &&
                packetfunctions_sameAddress(address,&neighbors_vars.neighbors[rowNumber].addr_64b);
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)3);
         return FALSE;
   }
}


#if (IEEE802154E_AMCA == 1)
void forceNeighborhood(uint8_t myaddr){
	uint8_t i,pos;
	uint16_t addr16b,bc;

     i=0;

     pos = getMotePos(myaddr,1);

     if (pos < MAX_NUM_MOTES){
		for (i=0;i<tableNeighbor[pos].numNeighbor;i++) {
			addr16b = tableNeighbor[pos].ele[i].addr16b;
			bc = tableNeighbor[pos].ele[i].bc;
			macneighbors_vars.neighbors[i].used                   = TRUE;
			macneighbors_vars.neighbors[i].parentPreference       = 0;
			macneighbors_vars.neighbors[i].stableNeighbor         = 0;
			macneighbors_vars.neighbors[i].switchStabilityCounter = 0;
			macneighbors_vars.neighbors[i].addr_16b.type = 1;
			macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] = *((uint8_t *)&addr16b);
			macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] = *(((uint8_t *)&addr16b)+1);
			//memcpy(&neighbors_vars.neighbors[i].addr_64b,address,sizeof(open_addr_t));
			//macneighbors_vars.neighbors[i].numRx                  = 1;
			macneighbors_vars.neighbors[i].numTxErr               = 0;
			macneighbors_vars.neighbors[i].numTxDIO               = 0;
			macneighbors_vars.neighbors[i].numTxDIOErr            = 0;
			macneighbors_vars.neighbors[i].numTxDAO               = 0;
			macneighbors_vars.neighbors[i].numTxDAOErr            = 0;
			macneighbors_vars.neighbors[i].numTxCOAP              = 0;
			macneighbors_vars.neighbors[i].numTxCOAPErr           = 0;
			macneighbors_vars.neighbors[i].numTxHello             = 0;
			macneighbors_vars.neighbors[i].bestchan               = bc;
			macneighbors_vars.neighbors[i].broadcastPending       = 0;
		}
     }
}

uint8_t macneighbors_getFixedBestChan(uint8_t addr16b){
	uint8_t bestchan=0;
	uint8_t pos=0;

	pos= getMotePos(addr16b,1);

	if (pos < MAX_NUM_MOTES){
		bestchan = MoteBestChanTable[pos];
	}

	return bestchan;

}


uint8_t macneighbors_getMyBestChan(void){
	return (macneighbors_vars.mybestchan);
}

/*
 * apartir do address retorna o bestchannel da tabela de vizinhos
 * quando o address eh broadcast, retorna o primeiro pendente da lista e o respectivo address deste vizinho em targetaddr
 */
uint8_t macneigh_getBChan(open_addr_t* address){
   uint8_t     i;
   open_addr_t temp_addr_16b;
   uint8_t     returnVal=0;

   // but neighbor's IPv6 address in prefix and EUI64
   switch (address->type) {
	  case ADDR_16B:
		 memcpy((void *)&temp_addr_16b,(void *) address,sizeof(open_addr_t));
		 break;
	  case ADDR_64B:
		  temp_addr_16b.type = 1;
		  temp_addr_16b.addr_16b[0] = address->addr_64b[6];
		  temp_addr_16b.addr_16b[1] = address->addr_64b[7];
		  break;
	  case ADDR_128B:
		  temp_addr_16b.type = 1;
		  //Aqui pode ser o dagroot...neste caso o endereco eh 0xFF02...0002
		  //PROVISORIO>...TODO!!!! SOMENTE PARA TESTE...DESCOBRIR COMO LER O ENDERECO DO DESTINO...
		  if (((address->addr_128b[0] == 0xFF) && (address->addr_128b[1] == 0x02)) ||
			  ((address->addr_128b[0] == 0xFE) && (address->addr_128b[1] == 0x80)))
		  {
			  uint16_t addr16b=ADDR16b_MOTE0;
			  temp_addr_16b.addr_16b[1] = *((uint8_t *)&addr16b);
			  temp_addr_16b.addr_16b[0] = *(((uint8_t *)&addr16b)+1);
		  }
		  else{
			  temp_addr_16b.addr_16b[0] = address->addr_128b[14];
			  temp_addr_16b.addr_16b[1] = address->addr_128b[15];
		  }

		  break;
	  default:
		 return returnVal;
   }

   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	  if (macisThisRowMatching(&temp_addr_16b,i)) {
		 returnVal  = macneighbors_vars.neighbors[i].bestchan;
		 break;
	  }
   }

   return (returnVal);
}

uint8_t macneigh_getBChanMultiCast(uint8_t *bestchannel,open_addr_t* targetaddr,uint8_t useLiveList){
   uint8_t     i;
   open_addr_t temp_addr_16b;
   uint8_t     returnVal=0;
   uint8_t     ret = FALSE;

   if (useLiveList == TRUE) {
	   //Descubro um vizinho que ainda esta pendente o broadcast...entao seleciono o canal dele...
	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
		  if ((macneighbors_vars.neighbors[i].used == TRUE) &&
			  (macneighbors_vars.neighbors[i].stableNeighbor == TRUE) &&
			  (macneighbors_vars.neighbors[i].broadcastPending)) {
		     *bestchannel  = macneighbors_vars.neighbors[i].bestchan;
			 targetaddr->type = macneighbors_vars.neighbors[i].addr_16b.type;
			 targetaddr->addr_16b[0] = macneighbors_vars.neighbors[i].addr_16b.addr_16b[0];
			 targetaddr->addr_16b[1] = macneighbors_vars.neighbors[i].addr_16b.addr_16b[1];
			 ret = TRUE;
			 break;
		  }
	   }
   }
   else{
	   //Descubro um vizinho que ainda esta pendente o broadcast...entao seleciono o canal dele...
	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
		  if ((macneighbors_vars.neighbors[i].used == TRUE) &&
			  (macneighbors_vars.neighbors[i].broadcastPending)) {
		     *bestchannel  = macneighbors_vars.neighbors[i].bestchan;
			 targetaddr->type = macneighbors_vars.neighbors[i].addr_16b.type;
			 targetaddr->addr_16b[0] = macneighbors_vars.neighbors[i].addr_16b.addr_16b[0];
			 targetaddr->addr_16b[1] = macneighbors_vars.neighbors[i].addr_16b.addr_16b[1];
			 ret = TRUE;
			 break;
		  }
	   }
   }

   return ret;
}

bool macisThisAddressPendingBroadcast(open_addr_t* address) {

	uint8_t i;
	bool ret=0;

   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	  if ((macneighbors_vars.neighbors[i].used == TRUE) &&
		  (macneighbors_vars.neighbors[i].broadcastPending) &&
		  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == address->addr_16b[0]) &&
		  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == address->addr_16b[1])) {
		 ret  = TRUE;
		 break;
	  }
   }

   return ret;
}

bool macisThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
   switch (address->type) {
      case ADDR_16B:
         return macneighbors_vars.neighbors[rowNumber].used &&
                packetfunctions_sameAddress(address,&macneighbors_vars.neighbors[rowNumber].addr_16b);
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)3);
         return FALSE;
   }
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t macneighbors_getNumNeighbors() {
   uint8_t i;
   uint8_t returnVal;

   returnVal=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (macneighbors_vars.neighbors[i].used==TRUE) {
         returnVal++;
      }
   }
   return returnVal;
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.
       Esta funcao é utilizada no inicio de um slot de Tx.
       Entao ele considera o numero de vizinhos baseado no criterio da livelist do slot anterior.

 inputs useStableNeighbor indica se vou considerar a livelist ou nao...

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t macneighbors_setBroadCastPending(uint8_t useStableNeighbor) {
   uint8_t i;
   uint8_t returnVal;

   returnVal=0;
   if (useStableNeighbor == TRUE) {
	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
	    		  (macneighbors_vars.neighbors[i].stableNeighbor == TRUE))  {
	    	  macneighbors_vars.neighbors[i].broadcastPending = TRUE;
	         returnVal++;
	      }
	   }
   }
   else {
	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	      if (macneighbors_vars.neighbors[i].used==TRUE)  {
	    	  macneighbors_vars.neighbors[i].broadcastPending = TRUE;
	         returnVal++;
	      }
	   }

   }

   return returnVal;
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t macneighbors_clearBroadcastPending(open_addr_t actualsrcaddr) {
   uint8_t i;

   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
       	  (macneighbors_vars.neighbors[i].addr_16b.type == actualsrcaddr.type) &&
    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == actualsrcaddr.addr_16b[0]) &&
    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == actualsrcaddr.addr_16b[1])){
    	  macneighbors_vars.neighbors[i].broadcastPending = 0;
    	  return TRUE;
      }
   }
   return FALSE;
}

/*
 * Calcula o numero de frameDIO (broadcast) que foi enviado para cada nó
 */
uint8_t macneighbors_calcbroadcastsent(void) {
#if 0
	uint8_t i;
	uint8_t count=0;

   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
    	  (macneighbors_vars.neighbors[i].stableNeighbor == TRUE) &&
       	  (macneighbors_vars.neighbors[i].broadcastPending == 0)) {
            count++;
   	   }
   }

   return count;

#else
   return 0;
#endif
}

uint8_t macneighbors_getaddrbroadcastsent(open_addr_t *address) {
   uint8_t i;

   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
    	  (macneighbors_vars.neighbors[i].stableNeighbor == TRUE) &&
       	  (macneighbors_vars.neighbors[i].broadcastPending == 0)) {
    	  //TODO!!!! Este campo pode ser modificado incluindo isso diretamente na tabela da topologia...
    	  convertaddress16to64(address,&macneighbors_vars.neighbors[i].addr_16b);
    	  return TRUE;
   	   }
   }

   return FALSE;
}
void macneighbors_updtlivelist(open_addr_t actualsrcaddr) {
	   uint8_t i;

	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
	       	  (macneighbors_vars.neighbors[i].addr_16b.type == actualsrcaddr.type) &&
	    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == actualsrcaddr.addr_16b[0]) &&
	    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == actualsrcaddr.addr_16b[1])){
#if (MULTI_CHANNEL_HELLO_ENABLE == 0)
				  macneighbors_vars.neighbors[i].numTx++;

				  if ( macneighbors_vars.neighbors[i].numTx > 10) {
					  macneighbors_vars.neighbors[i].stableNeighbor = TRUE;
		    	  }
#else
	    		  macneighbors_vars.neighbors[i].switchStabilityCounter = 0;
		    	  macneighbors_vars.neighbors[i].stableNeighbor = TRUE;
#endif

	    	  break;

	      }
	   }
}

void macneighbors_updtstatistics(open_addr_t address, uint8_t frameType,uint8_t status) {
	   uint8_t i;
	   open_addr_t addr16b;

	   switch (address.type) {
		  case ADDR_16B:
			 memcpy((void *)&addr16b,(void *) &address,sizeof(open_addr_t));
			 break;
		  case ADDR_64B:
			  addr16b.type = ADDR_16B;
			  addr16b.addr_16b[0] = address.addr_64b[6];
			  addr16b.addr_16b[1] = address.addr_64b[7];
			  break;
		  case ADDR_128B:
			  addr16b.type = ADDR_16B;
			  addr16b.addr_16b[0] = address.addr_128b[14];
			  addr16b.addr_16b[1] = address.addr_128b[15];

			  break;
		  default:
			  addr16b.type = 0;
			  break;
	   }

	   if ((status == E_SUCCESS) && (addr16b.type == ADDR_16B)) {
		   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
		      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
		    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == addr16b.addr_16b[0]) &&
		    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == addr16b.addr_16b[1])){

		    	  switch (frameType) {
					  case IANA_ICMPv6_RPL_DIO:
						 macneighbors_vars.neighbors[i].numTxDIO++;
						 break;
					  case IANA_ICMPv6_RPL_DAO:
						 macneighbors_vars.neighbors[i].numTxDAO++;
						 break;
					  case IANA_UDP:
						 macneighbors_vars.neighbors[i].numTxCOAP++;
						 break;
					  case IANA_ICMPv6_RA_PREFIX_INFORMATION:
						 macneighbors_vars.neighbors[i].numTxHello++;
						 break;
					  default:
						 macneighbors_vars.neighbors[i].numTxErr++;
						 break;
		    	  }
		    	  break;
		      }
		   }
	   }
	   else{  //E_FAIL
		   if (addr16b.type == ADDR_16B) {
			   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
				  if ((macneighbors_vars.neighbors[i].used==TRUE) &&
					  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == addr16b.addr_16b[0]) &&
					  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == addr16b.addr_16b[1])){

					  switch (frameType){
						  case IANA_ICMPv6_RPL_DIO:
							 macneighbors_vars.neighbors[i].numTxDIOErr++;
							 break;
						  case IANA_ICMPv6_RPL_DAO:
							 macneighbors_vars.neighbors[i].numTxDAOErr++;
							 break;
						  case IANA_UDP:
							 macneighbors_vars.neighbors[i].numTxCOAPErr++;
							 break;
						  default:
							 macneighbors_vars.neighbors[i].numTxErr++;
							 break;
					  }

					  break;
				  }
			   }
		   }
	   }
 }

void macneighbors_clearlivelist(open_addr_t actualsrcaddr) {
	   uint8_t i;

	   for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
	      if ((macneighbors_vars.neighbors[i].used==TRUE) &&
	       	  (macneighbors_vars.neighbors[i].addr_16b.type == actualsrcaddr.type) &&
	    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[0] == actualsrcaddr.addr_16b[0]) &&
	    	  (macneighbors_vars.neighbors[i].addr_16b.addr_16b[1] == actualsrcaddr.addr_16b[1])){

	    	  macneighbors_vars.neighbors[i].switchStabilityCounter++;

	    	  if (macneighbors_vars.neighbors[i].switchStabilityCounter >= 2) {
	    		  macneighbors_vars.neighbors[i].switchStabilityCounter  = 0;
	    	      macneighbors_vars.neighbors[i].stableNeighbor          = 0;
	    	      break;
              }
	      }
	   }
}

#endif


