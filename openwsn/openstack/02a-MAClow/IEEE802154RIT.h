#ifndef __IEEE802154RIT_H
#define __IEEE802154RIT_H

//----------------RIT stage
enum RITstate{
	S_RIT_sleep_state     = 0x00,
	S_RIT_RX_window_state = 0x01,
	S_RIT_TX_state        = 0x02
};

#endif
