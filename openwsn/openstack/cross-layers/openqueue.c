#include "opendefs.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "IEEE802154E.h"
#include "IEEE802154RIT.h"
#include "radio.h"
#include "neighbors.h"
//=========================== variables =======================================

openqueue_vars_t openqueue_vars;

#if (IEEE802154E_RIT == 1) || (IEEE802154E_AMAC == 1) || (IEEE802154E_AMCA == 1) || (IEEE802154E_RITMC == 1)
OpenQueueEntry_t advRIT;
owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);
sRITqueue pvObjList[MAX_RIT_LIST_ELEM];
#endif


//support variable for RIT procedure
extern uint8_t     coappending;


uint8_t u8NrMsgQueue;          // numero de posicoes utilizadas na queue de RIT Pending Msg to TX
uint8_t numAvailableElements;  // numero de lugares ocupados no pool de mensagem
uint8_t maxElements;
uint8_t macRITActualPos;           //tem a informacao da posicao do atual Tx Msg Pending (antes do ola)
//uint8_t RITQueue_ElementPending;   //salvo o elemento que acabou de enviar uma msg - para retornar apos o envio (apos o ola)
uint32_t numOfQueueInsertionErrors;

//=========================== prototypes ======================================

void openqueue_reset_entry(OpenQueueEntry_t* entry);

//=========================== public ==========================================

//======= admin

/**
\brief Initialize this module.
*/
void openqueue_init() {
   uint8_t i;
   for (i=0;i<QUEUELENGTH;i++){
      openqueue_reset_entry(&(openqueue_vars.queue[i]));
   }


#if (IEEE802154E_RIT == 1)
   // Inicializa a area de memoria do frame do RIT
   //TODO!!! Foi criado como global mas nao sei se ocupa muito espaco.
   openqueue_freePacketRITBuffer(&(advRIT));
#endif
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_queue() {
   debugOpenQueueEntry_t output[QUEUELENGTH];
   uint8_t i;
   for (i=0;i<QUEUELENGTH;i++) {
      output[i].creator = openqueue_vars.queue[i].creator;
      output[i].owner   = openqueue_vars.queue[i].owner;
   }
   openserial_printStatus(STATUS_QUEUE,(uint8_t*)&output,QUEUELENGTH*sizeof(debugOpenQueueEntry_t));
   return TRUE;
}

//======= called by any component

/**
\brief Request a new (free) packet buffer.

Component throughout the protocol stack can call this function is they want to
get a new packet buffer to start creating a new packet.

\note Once a packet has been allocated, it is up to the creator of the packet
      to free it using the openqueue_freePacketBuffer() function.

\returns A pointer to the queue entry when it could be allocated, or NULL when
         it could not be allocated (buffer full or not synchronized).
*/
OpenQueueEntry_t* openqueue_getFreePacketBuffer(uint8_t creator) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // refuse to allocate if we're not in sync
#if (IEEE802154E_TSCH == 1)
   if (ieee154e_isSynch()==FALSE && creator > COMPONENT_IEEE802154E){
     ENABLE_INTERRUPTS();
     return NULL;
   }
#endif
   // if you get here, I will try to allocate a buffer for you
   
   // walk through queue and find free entry
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_NULL) {
         openqueue_vars.queue[i].creator=creator;
         openqueue_vars.queue[i].owner=COMPONENT_OPENQUEUE;
         ENABLE_INTERRUPTS(); 

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_OPENQUEUE == 1))
	{
		uint8_t *pucAux = (uint8_t *) &openqueue_vars.queue[i];
		uint8_t pos=0;

		rffbuf[pos++]= RFF_OPENQUEUE_ALLOC;
		rffbuf[pos++]= 0x01;
		rffbuf[pos++]= creator;
		rffbuf[pos++]= i;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();


   return NULL;
}


/**
\brief Free a previously-allocated packet buffer.

\param pkt A pointer to the previsouly-allocated packet buffer.

\returns E_SUCCESS when the freeing was succeful.
\returns E_FAIL when the module could not find the specified packet buffer.
*/
owerror_t openqueue_freePacketBuffer(OpenQueueEntry_t* pkt) {
   uint8_t i;
   uint8_t *pucAux = (uint8_t *) pkt;
   uint8_t  creator;

   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (&openqueue_vars.queue[i]==pkt) {
         if (openqueue_vars.queue[i].owner==COMPONENT_NULL) {
            // log the error
            openserial_printCritical(COMPONENT_OPENQUEUE,ERR_FREEING_UNUSED,
                                  (errorparameter_t)0,
                                  (errorparameter_t)0);
         }
         creator = openqueue_vars.queue[i].creator;

         openqueue_reset_entry(&(openqueue_vars.queue[i]));
         ENABLE_INTERRUPTS();

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_OPENQUEUE == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_OPENQUEUE_FREE;
		rffbuf[pos++]= 0x01;
		rffbuf[pos++]= creator;
		rffbuf[pos++]= i;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux++;
		rffbuf[pos++]= pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
         return E_SUCCESS;
      }
   }
   // log the error
   openserial_printCritical(COMPONENT_OPENQUEUE,ERR_FREEING_ERROR,
                         (errorparameter_t)0,
                         (errorparameter_t)0);
   ENABLE_INTERRUPTS();
   return E_FAIL;
}

#if (IEEE802154E_RIT == 1) || (IEEE802154E_AMAC == 1) || (IEEE802154E_AMCA == 1) || (IEEE802154E_RITMC == 1)
// Inicializa a area de memoria do frame do RIT
owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt) {

	INTERRUPT_DECLARATION();

	DISABLE_INTERRUPTS();

	openqueue_reset_entry(pkt);

	ENABLE_INTERRUPTS();

	return E_SUCCESS;
}


#endif

/**
\brief Free all the packet buffers created by a specific module.

\param creator The identifier of the component, taken in COMPONENT_*.
*/
void openqueue_removeAllCreatedBy(uint8_t creator) {
   uint8_t i;
   uint8_t count=0;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++){
      if (openqueue_vars.queue[i].creator==creator) {
    	  count++;
    	  openqueue_reset_entry(&(openqueue_vars.queue[i]));
      }
   }
   ENABLE_INTERRUPTS();

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_OPENQUEUE == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_OPENQUEUE_FREE;
		rffbuf[pos++]= 0x02;
		rffbuf[pos++]= creator;
		rffbuf[pos++]= count;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}

/**
\brief Free all the packet buffers owned by a specific module.

\param owner The identifier of the component, taken in COMPONENT_*.
*/
void openqueue_removeAllOwnedBy(uint8_t owner) {
   uint8_t i;
   uint8_t count=0;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++){
      if (openqueue_vars.queue[i].owner==owner) {
    	  count++;
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
      }
   }
   ENABLE_INTERRUPTS();

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_OPENQUEUE == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_OPENQUEUE_FREE;
		rffbuf[pos++]= 0x03;
		rffbuf[pos++]= owner;
		rffbuf[pos++]= count;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}

//======= called by RES

OpenQueueEntry_t* openqueue_sixtopGetSentPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_IEEE802154E_TO_SIXTOP &&
          openqueue_vars.queue[i].creator!=COMPONENT_IEEE802154E) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

OpenQueueEntry_t* openqueue_sixtopGetReceivedPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_IEEE802154E_TO_SIXTOP &&
          openqueue_vars.queue[i].creator==COMPONENT_IEEE802154E) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

//======= called by IEEE80215E

OpenQueueEntry_t* openqueue_macGetDataPacket(open_addr_t* toNeighbor) {
   uint8_t i;
   //INTERRUPT_DECLARATION();
   // DISABLE_INTERRUPTS();
   if (toNeighbor->type==ADDR_64B) {
      // a neighbor is specified, look for a packet unicast to that neigbhbor
      for (i=0;i<QUEUELENGTH;i++) {
         if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
            packetfunctions_sameAddress(toNeighbor,&openqueue_vars.queue[i].l2_nextORpreviousHop)) {
            // ENABLE_INTERRUPTS();
            return &openqueue_vars.queue[i];
         }
      }
   } else if (toNeighbor->type==ADDR_ANYCAST) {
      // anycast case: look for a packet which is either not created by RES
      // or an KA (created by RES, but not broadcast)
      for (i=0;i<QUEUELENGTH;i++) {
         if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
             ( openqueue_vars.queue[i].creator!=COMPONENT_SIXTOP ||
                (
                   openqueue_vars.queue[i].creator==COMPONENT_SIXTOP &&
                   packetfunctions_isBroadcastMulticast(&(openqueue_vars.queue[i].l2_nextORpreviousHop))==FALSE
                )
             )
            ) {
            // ENABLE_INTERRUPTS();
            return &openqueue_vars.queue[i];
         }
      }
   }
   // ENABLE_INTERRUPTS();
   return NULL;
}

OpenQueueEntry_t* openqueue_macGetAdvPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
          openqueue_vars.queue[i].creator==COMPONENT_SIXTOP              &&
          packetfunctions_isBroadcastMulticast(&(openqueue_vars.queue[i].l2_nextORpreviousHop))) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}


#if (IEEE802154E_TSCH == 0)

inline bool RITQueue_IsEmpty(void) {	return (numAvailableElements == 0) ? true : false; }

inline bool RITQueue_IsFull(void) { 	return (numAvailableElements == maxElements) ? true : false; }

/**
\brief Write the 64-bit address of some neighbor to some location.
   address_1 - Endereco da mensagem que chegou
   adrress_2 - endereco da mensagem que esta pendente
    se endereco da mensagem pendente eh do SINK entao envio mesmo assim.
    TODO!!! MELHORAR
*/

bool  RITQueue_AddrIsTheSame(open_addr_t* addr1, open_addr_t* addr2){

	uint8_t i, nroctets = 0;

    //if ((addr1->type == 0x02) && (addr1->addr_64b[6] == 0xFF) && (addr1->addr_64b[7] == 0xFF))
    //{ //if address is broadcast then check the neighbor list
    //	return true;
    //}
#if (USE_RITREQ_SHORT == 1)
    if ((addr1->type == 0x02) && (addr2->type == 0x01))
    { // o addr2 é um olá request...neste caso somente comparo o final do frame
    	if ((addr1->addr_64b[6] == addr2->addr_16b[0]) && (addr1->addr_64b[7] == addr2->addr_16b[1]))
            return TRUE;
    }
#endif
    else if (addr1->type == addr2->type)
	{
		switch (addr1->type)
		{
		case 0x01:
			nroctets = 2;
			break;
		case 0x02:
			nroctets = 8;
			break;
		case 0x03:
			nroctets = 16;
			break;
		default:
			nroctets = 0;
			break;
		}

		if (nroctets > 0) {
			for (i = 0; i < nroctets; i++) {
				if (addr1->addr_16b[i] != addr2->addr_16b[i])
				{
					return false;
				}
			}
			return TRUE;
		}
		else
			return false;

	}
#if (USE_RITREQ_SHORT == 1)
    else if ((addr1->type == 0x03) &&  (addr2->type == 0x01))
    {
		//AQUI TEM DOIS CASOS
		// SE O ADDRESS 1  (TRANSMISSOR) eh o SINK... (FF 02 ...00 02) entao eu envio...sem pensar...(DIO)
		// pode ser que seja DAO FORWARDING...ai somente comparo o final do endereco do Transmissor com o frame do ola

		if ((addr1->addr_128b[0] == 0xFF) &&
			(addr1->addr_128b[1] == 0x02) &&
			(addr1->addr_128b[14] == 0x00) &&
			(addr1->addr_128b[15] == 0x02)) {
			return TRUE;
		}
		else if ((addr1->addr_128b[14] == addr2->addr_16b[0]) &&
				 (addr1->addr_128b[15] == addr2->addr_64b[1])) {
			return TRUE;
		}


#else
	    else if ((addr1->type == 0x03) &&  (addr2->type == 0x01))
	    {
			//AQUI TEM DOIS CASOS
			// SE O ADDRESS 1  (TRANSMISSOR) eh o SINK... (FF 02 ...00 02) entao eu envio...sem pensar...(DIO)
			// pode ser que seja DAO FORWARDING...ai somente comparo o final do endereco do Transmissor com o frame do ola

			if ((addr1->addr_64b[0] == 0xFF) &&
				(addr1->addr_64b[1] == 0x02) &&
				(addr1->addr_64b[14] == 0x00) &&
				(addr1->addr_64b[15] == 0x02)) {
				return TRUE;
			}
			else if ((addr1->addr_128b[8] == addr2->addr_64b[0]) &&
					 (addr1->addr_128b[9] == addr2->addr_64b[1]) &&
					 (addr1->addr_128b[10] == addr2->addr_64b[2]) &&
					 (addr1->addr_128b[11] == addr2->addr_64b[3]) &&
					 (addr1->addr_128b[12] == addr2->addr_64b[4]) &&
					 (addr1->addr_128b[13] == addr2->addr_64b[5]) &&
					 (addr1->addr_128b[14] == addr2->addr_64b[6]) &&
					 (addr1->addr_128b[15] == addr2->addr_64b[7])) {
				return TRUE;
			}
#endif
	}

	return false;
}

#if 0
bool  RITQueue_isMyAddress(open_addr_t* addr){

	uint8_t i, nroctets = 0;

    if ((addr1->type == 0x02) && (addr1->addr_64b[6] == 0xFF) && (addr1->addr_64b[7] == 0xFF))
    { //if address is broadcast then send for anyone.
        return true;
    }
    else if (addr1->type == addr2->type)
	{
		switch (addr1->type)
		{
		case 0x01:
			nroctets = 2;
			break;
		case 0x02:
			nroctets = 8;
			break;
		case 0x03:
			nroctets = 16;
			break;
		default:
			nroctets = 0;
			break;
		}

		if (nroctets > 0) {
			for (i = 0; i < nroctets; i++) {
				if (addr1->addr_16b[i] != addr2->addr_16b[i])
				{
					return false;
				}
			}
			return true;
		}
		else
			return false;

	}
    else if ((addr1->type == 0x03) &&  (addr2->type == 0x02))
    {
		//AQUI TEM DOIS CASOS
		// SE O ADDRESS 1  (TRANSMISSOR) eh o SINK... (FF 02 ...00 02) entao eu envio...sem pensar...(DIO)
		// pode ser que seja DAO FORWARDING...ai somente comparo o final do endereco do Transmissor com o frame do ola

		if ((addr1->addr_64b[0] == 0xFF) &&
			(addr1->addr_64b[1] == 0x02) &&
			(addr1->addr_64b[14] == 0x00) &&
			(addr1->addr_64b[15] == 0x02)) {
			return true;
		}
		else if ((addr1->addr_128b[8] == addr2->addr_64b[0]) &&
				 (addr1->addr_128b[9] == addr2->addr_64b[1]) &&
				 (addr1->addr_128b[10] == addr2->addr_64b[2]) &&
				 (addr1->addr_128b[11] == addr2->addr_64b[3]) &&
				 (addr1->addr_128b[12] == addr2->addr_64b[4]) &&
				 (addr1->addr_128b[13] == addr2->addr_64b[5]) &&
				 (addr1->addr_128b[14] == addr2->addr_64b[6]) &&
				 (addr1->addr_128b[15] == addr2->addr_64b[7])) {
			return true;
		}
	}

	return false;
}
#endif

bool  RITQueue_copyaddress(open_addr_t* addr1, open_addr_t* addr2){

	uint8_t i, nroctets = 0;

	addr1->type = addr2->type;

	switch (addr2->type)
	{
		case 0x01:
			nroctets = 2;
			break;
		case 0x02:
			nroctets = 8;
			break;
		case 0x03:
			nroctets = 16;
			break;
		default:
			nroctets = 0;
			break;
	}

	if (nroctets > 0) {
		for (i = 0; i < nroctets; i++) {
			addr1->addr_128b[i] = addr2->addr_128b[i];
		}
		return true;
	}

	return false;
}

uint8_t RITQueue_Put(sRITelement *psEle,uint8_t pending, uint8_t numTargetParents)
{
	uint8_t  i;

	if (psEle == NULL)
		return maxElements; //msg invalida

	if (RITQueue_IsFull() == false)
	{
#if 0 // AQUI EU NAO ESTOU USANDO MAIS A FILA..COLOCO SEMPRE NO PRIMEIRO ELEMENTO SOMENTE
		//procuro o primeiro elemento da fila disponivel
		for (i = 0; i < maxElements; i++)
		{
			if ((pvObjList[i].frameType == 0) && (pvObjList[i].msglength == 0))
			{
				/* Insert msg into the queue. */
				pvObjList[i].destaddr = psEle->destaddr;
				pvObjList[i].timestamp = psEle->timestamp;
				pvObjList[i].msglength = psEle->msglength;
				pvObjList[i].frameType = psEle->frameType;
				pvObjList[i].isBroadcastMulticast = psEle->isBroadcastMulticast;
				memcpy(pvObjList[i].msg, psEle->msg, psEle->msglength);
				pvObjList[i].pending = pending;
				pvObjList[i].numTargetParents = numTargetParents;

				if (psEle->frameType == IANA_UDP)
					coappending = true;

				if (psEle->frameType != IANA_ICMPv6_RPL_DIO) {
					pvObjList[i].countretry++;
				}

				numAvailableElements++;
				break;
			}
		}
#else
		i =0;
		if ((pvObjList[i].frameType == 0) && (pvObjList[i].msglength == 0))
		{
			/* Insert msg into the queue. */
			pvObjList[i].destaddr = psEle->destaddr;
			pvObjList[i].timestamp = psEle->timestamp;
			pvObjList[i].msglength = psEle->msglength;
			pvObjList[i].frameType = psEle->frameType;
			pvObjList[i].isBroadcastMulticast = psEle->isBroadcastMulticast;
			memcpy(pvObjList[i].msg, psEle->msg, psEle->msglength);
			pvObjList[i].pending = pending;
			pvObjList[i].numTargetParents = numTargetParents;

			if (psEle->frameType == IANA_UDP)
				coappending = true;

			if (psEle->frameType != IANA_ICMPv6_RPL_DIO) {
				pvObjList[i].countretry++;
			}

			numAvailableElements++;
		}
#endif
	}

	return i;
}

/* RITQueue_updateactualtxtime
 * salvo a ultima programacao da janela do RIT tx window para se
 * precisar reprogramar a janela de timer quando uma outra msg chegar
 *
 */

void RITQueue_updateactualtxtime(uint32_t actualtime)
{
	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

    if (macRITActualPos < maxElements)
	  pvObjList[macRITActualPos].lasttxduration = actualtime ;

	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

}
void RITQueue_update_element (uint8_t pos)
{
	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

    if (macRITActualPos < maxElements)
	  pvObjList[pos].numTargetParents--;

	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

}


uint8_t RITQueue_getNrPendingParents (uint8_t pos)
{
    if (pos < maxElements)
	  return (pvObjList[pos].numTargetParents);

    return 0;
}

uint8_t RITQueue_Clear_Pending(uint8_t pos,open_addr_t actualsrcaddr)
{
	uint8_t  elem=0;

	if (RITQueue_IsEmpty() == false)
	{
		if ( pos < maxElements)
		{
			if (macneighbors_clearBroadcastPending(actualsrcaddr) == TRUE) {
				if (pvObjList[pos].numTargetParents > 0)
					pvObjList[pos].numTargetParents--;

				elem = TRUE;
			}
		}
	}

	return elem;
}



sRITqueue RITQueue_Get_Element(uint8_t pos)
{
	sRITqueue  elem;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		if ( pos < maxElements)
		{
			elem = pvObjList[pos];
		}
		else
		{
			elem.frameType = 0;
			RITQueue_ClearAddress(&elem.destaddr);
 		}
	}
	else
	{
		elem.frameType = 0;
		RITQueue_ClearAddress(&elem.destaddr);
	}

	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return elem;
}

uint8_t RITQueue_Get_Pos(open_addr_t *paddr)
{
	uint8_t i,ret=maxElements;
	uint8_t broadcast=0;

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if (RITQueue_AddrIsTheSame(&pvObjList[i].destaddr, paddr) == TRUE)
			{
				ret = i;
				break;
			}
		}
	}

	return ret;
}


bool  RITQueue_ClearAddress(open_addr_t* addr){

	uint8_t i;

	addr->type = 0;
	for (i=0;i<16;i++)
	{
	  addr->addr_128b[i] = 0;
	}

	return (true);
}

bool RITQueue_PutPending(uint8_t elementpos)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		if (elementpos < maxElements)
		{
			pvObjList[elementpos].pending = 1;
			pvObjList[elementpos].countretry++;
			ret = true;
		}
	}
	else
	{
		//printf("LISTA VAZIA!!!!! \n");
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}

/*
 * Verifica se existe algum frame pendente
 */
bool RITQueue_ExistFramePending(void)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	for (i = 0; i < maxElements; i++)
	{
		if (pvObjList[i].pending)
		{
			if (pvObjList[i].countretry < 3)
			{
				ret = true;
				break;
			}
			else  //REMOVO O ELEMENTO QUE JA ESTA A TEMPOS AQUI
			{
				RITQueue_Free(i);
			}
		}
	}

   if (coappending)
	   ret = true;


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}
/*
 * Retorna a posicao do 1o frame pendente
 * se ja existir um frame pendente e ja ultrapassou o nro de retry entao
 * eh removido ele da fila...
 */
uint8_t RITQueue_GetPending(void)
{
	bool pos = maxElements;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if (pvObjList[i].pending)
			{
				if (pvObjList[i].countretry < 3)
				{
					pos = i;
					break;
				}
				else  //REMOVO O ELEMENTO QUE JA ESTA A TEMPOS AQUI
				{
					RITQueue_Free(i);
				}
			}
		}
	}
	else
	{
		//printf("LISTA VAZIA!!!!! \n");
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return pos;
}

bool RITQueue_Free(uint8_t elementpos)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (elementpos < maxElements)
	{
		RITQueue_ClearAddress(&pvObjList[elementpos].destaddr);
		pvObjList[elementpos].timestamp   = 0;
		pvObjList[elementpos].msglength   = 0;
		pvObjList[elementpos].frameType   = 0;
		pvObjList[elementpos].pending     = 0;
		pvObjList[elementpos].numTargetParents     = 0;
		pvObjList[elementpos].countretry  = 0;
		pvObjList[elementpos].lasttxduration  = 0;
		pvObjList[elementpos].isBroadcastMulticast = 0;

		if (numAvailableElements > 0)
			numAvailableElements--;

		ret = true;
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}


/*
  Deleto todas as mensagens que estao na fila que nao tem prioridades
  criterios:
      - msg com a mais de 2000 segundos
      - msg DIO que nao foram enviadas
  retorno o numero de elementos deletados
*/
#define TIMEOUT_2SEC 65530
uint8_t RITQueue_cleanupoldmsg(void){
	uint32_t actualtime;
	uint32_t timeout;
	uint8_t i;
	uint8_t count = 0;
	uint8_t pos=0;
	bool imprimir = false;

	//get actualtime
	actualtime = radio_getTimerValue();

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if ((pvObjList[i].msglength > 0))
			{
				if ((pvObjList[i].frameType == 1) || (pvObjList[i].frameType == 2) ||
				    ((pvObjList[i].timestamp + TIMEOUT_2SEC) < actualtime))
				{
					if (RITQueue_Free(i)) {
						count++;
					}
				}
			}
		}

	}




	return count;
}

void RITQueue_Init(void){

	uint32_t i;

	numOfQueueInsertionErrors = 0;
	numAvailableElements = 0;
	maxElements = MAX_RIT_LIST_ELEM;

	for (i = 0; i < maxElements; i++)
	{
		RITQueue_Free(i);
	}
}
#endif // if (IEEE802154E_TSCH == 0)
//=========================== private =========================================

void openqueue_reset_entry(OpenQueueEntry_t* entry) {
   //admin
   entry->creator                      = COMPONENT_NULL;
   entry->owner                        = COMPONENT_NULL;
   entry->payload                      = &(entry->packet[127]);
   entry->length                       = 0;
   //l4
   entry->l4_protocol                  = IANA_UNDEFINED;
   //l3
   entry->l3_destinationAdd.type       = ADDR_NONE;
   entry->l3_sourceAdd.type            = ADDR_NONE;
   //l2
   entry->l2_nextORpreviousHop.type    = ADDR_NONE;
   entry->l2_frameType                 = IEEE154_TYPE_UNDEFINED;
   entry->l2_retriesLeft               = 0;
   entry->l2_IEListPresent             = 0;
}


