#ifndef __TOPOLOGY_H
#define __TOPOLOGY_H

/**
\addtogroup MAClow
\{
\addtogroup topology
\{
*/

#include "opendefs.h"
#include "IEEE802154.h"

//=========================== define ==========================================
#define TOPOLOGY_MOTE1    0x01
#define TOPOLOGY_MOTE2    0x02
#define TOPOLOGY_MOTE3    0x03
#define TOPOLOGY_MOTE4    0x04
#define TOPOLOGY_MOTE5    0x05
#define TOPOLOGY_MOTE6    0x06
#define TOPOLOGY_MOTE7    0x07
#define TOPOLOGY_MOTE8    0x08
#define TOPOLOGY_MOTE9    0x09
#define TOPOLOGY_MOTE10   0x0A
#define TOPOLOGY_MOTE11   0x0B
#define TOPOLOGY_MOTE12   0x0C
#define TOPOLOGY_MOTE13   0x0D
#define TOPOLOGY_MOTE14   0x0E
#define TOPOLOGY_MOTE15   0x0F

#define TOPOLOGY_MOTE16   0x10
#define TOPOLOGY_MOTE17   0x11
#define TOPOLOGY_MOTE18   0x12
#define TOPOLOGY_MOTE19   0x13
#define TOPOLOGY_MOTE20   0x14
#define TOPOLOGY_MOTE21   0x15
#define TOPOLOGY_MOTE22   0x16
#define TOPOLOGY_MOTE23   0x17
#define TOPOLOGY_MOTE24   0x18
#define TOPOLOGY_MOTE25   0x19
#define TOPOLOGY_MOTE26   0x1A
#define TOPOLOGY_MOTE27   0x1B
#define TOPOLOGY_MOTE28   0x1C
#define TOPOLOGY_MOTE29   0x1D
#define TOPOLOGY_MOTE30   0x1E
#define TOPOLOGY_MOTE31   0x1F

#define TOPOLOGY_MOTE32   0x20
#define TOPOLOGY_MOTE33   0x21
#define TOPOLOGY_MOTE34   0x22
#define TOPOLOGY_MOTE35   0x23
#define TOPOLOGY_MOTE36   0x24
#define TOPOLOGY_MOTE37   0x25
#define TOPOLOGY_MOTE38   0x26
#define TOPOLOGY_MOTE39   0x27
#define TOPOLOGY_MOTE40   0x28
#define TOPOLOGY_MOTE41   0x29
#define TOPOLOGY_MOTE42   0x2A
#define TOPOLOGY_MOTE43   0x2B
#define TOPOLOGY_MOTE44   0x2C
#define TOPOLOGY_MOTE45   0x2D
#define TOPOLOGY_MOTE46   0x2E
#define TOPOLOGY_MOTE47   0x2F

#define TOPOLOGY_MOTE48   0x30
#define TOPOLOGY_MOTE49   0x31
#define TOPOLOGY_MOTE50   0x32
#define TOPOLOGY_MOTE51   0x33
#define TOPOLOGY_MOTE52   0x34
#define TOPOLOGY_MOTE53   0x35
#define TOPOLOGY_MOTE54   0x36
#define TOPOLOGY_MOTE55   0x37
#define TOPOLOGY_MOTE56   0x38
#define TOPOLOGY_MOTE57   0x39
#define TOPOLOGY_MOTE58   0x3A
#define TOPOLOGY_MOTE59   0x3B
#define TOPOLOGY_MOTE60   0x3C
#define TOPOLOGY_MOTE61   0x3D
#define TOPOLOGY_MOTE62   0x3E
#define TOPOLOGY_MOTE63   0x3F

#define TOPOLOGY_MOTE64   0x40
#define TOPOLOGY_MOTE65   0x41
#define TOPOLOGY_MOTE66   0x42
#define TOPOLOGY_MOTE67   0x43
#define TOPOLOGY_MOTE68   0x44
#define TOPOLOGY_MOTE69   0x45
#define TOPOLOGY_MOTE70   0x46
#define TOPOLOGY_MOTE71   0x47
#define TOPOLOGY_MOTE72   0x48
#define TOPOLOGY_MOTE73   0x49
#define TOPOLOGY_MOTE74   0x4A
#define TOPOLOGY_MOTE75   0x4B
#define TOPOLOGY_MOTE76   0x4C
#define TOPOLOGY_MOTE77   0x4D
#define TOPOLOGY_MOTE78   0x4E
#define TOPOLOGY_MOTE79   0x4F


//=========================== typedef =========================================
 typedef struct {
	unsigned char idx;
	unsigned char nridx;
	unsigned char neighboors[10];
}SNeibhboorstable;

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== prototypes ======================================

bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header);

/**
\}
\}
*/

#endif
