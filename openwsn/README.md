OpenWSN firmware: stuff that runs on a mote

Part of UC Berkeley's OpenWSN project, http://www.openwsn.org/.

Build status
------------

|              builder                                                           |      build               | outcome
| ------------------------------------------------------------------------------ | ------------------------ | -------
| [Travis](https://travis-ci.org/openwsn-berkeley/openwsn-fw)                    | compile                  | [![Build Status](https://travis-ci.org/openwsn-berkeley/openwsn-fw.png?branch=develop)](https://travis-ci.org/openwsn-berkeley/openwsn-fw)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20TelosB/)           | compile (TelosB)         | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20TelosB)](http://builder.openwsn.org/job/Firmware%20TelosB/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20GINA/)             | compile (GINA)           | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20GINA)](http://builder.openwsn.org/job/Firmware%20GINA/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20wsn430v13b/)       | compile (wsn430v13b)     | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20wsn430v13b)](http://builder.openwsn.org/job/Firmware%20wsn430v13b/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20wsn430v14/)        | compile (wsn430v14)      | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20wsn430v14)](http://builder.openwsn.org/job/Firmware%20wsn430v14/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20Z1/)               | compile (Z1)             | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20Z1)](http://builder.openwsn.org/job/Firmware%20Z1/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20OpenMote-CC2538/)  | compile (OpenMote-CC2538) | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20OpenMote-CC2538)](http://builder.openwsn.org/job/Firmware%20OpenMote-CC2538/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20OpenMoteSTM/)      | compile (OpenMoteSTM)    | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20OpenMoteSTM)](http://builder.openwsn.org/job/Firmware%20OpenMoteSTM/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20IoT-LAB_M3/)       | compile (IoT-LAB_M3)     | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20IoT-LAB_M3)](http://builder.openwsn.org/job/Firmware%20IoT-LAB_M3/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20Agilefox/)         | compile (Agilefox)     | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20Agilefox)](http://builder.openwsn.org/job/Firmware%20Agilefox/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20Python%20(simulation)/) | compile (Python, simulation) | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware%20Python%20(simulation))](http://builder.openwsn.org/job/Firmware%20Python%20(simulation)/)
| [OpenWSN builder](http://builder.openwsn.org/job/Firmware%20Docs%20Publisher/) | publish documentation    | [![Build Status](http://builder.openwsn.org/buildStatus/icon?job=Firmware Docs Publisher)](http://builder.openwsn.org/job/Firmware%20Docs%20Publisher/)

Documentation
-------------

- overview: https://openwsn.atlassian.net/wiki/
- source code: http://openwsn-berkeley.github.io/firmware/


---------------------------------------------------------------
V8_0_0

data: 03/01/16
Remarks:
Esta versao ja funciona sem problemas com o COAP somente para 1 sink e 1 mote. (nao testei para mais)
Foi implementado um mecanismo de tentar otimizar o COAP onde nao publica RPL.DIO enquanto tem um 
comando coap pendente (request-->response) (flag coappending)

Esta versao tem integrado a placa de medicao de consumo configurado no arquivo board.h
funcionando no stack sem RIT.
Defines IEEE802154E_RIT 0 --> usando o OpenWSN padrao.
MYLINKXS_SENSORS 1 -> habilita os sensores implementados ate agora.
SENSOR_ACCEL = 1 --> foi testado nesta versao...leitura do acelerometro da placa RF06 (Texas)
SONOMA14 = 1 --> foi testado nesta versao...habilita a leitura da placa de medicao de consumo
MYLINKXS_REMOTE_CONTROL - NAO TESTADO NESTA VERSAO. Versao do controle remoto com arduino
MYLINKXS_LIGHT_CONTROL - NAO TESTADO NESTA VERSAO. Controle da lampada em um GPIO direto na CC2538.
MYLINKXS_SENSORS 0 seria a validacao da placa vermelha do marcelo barros. Mas nao foi testada nesta versao.



