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


------------------------------------------------------------------
Update history
-----------------------------------------------------------------
Version: V8_0_4
Date: 10_02_16
Remarks:
 Nesta versao foi feito testes somente com o RIT para suportar até 2 saltos. 3 saltos nao conseguiu enviar o comando de DAO...
 tambem foi feito a tentativa de enviar o DIO para mais de um vizinho mas nao tive sucesso pois esta ocorrendo muitas colisoes...
 Sera necessario primeiro resolver os problemas das colisoes para depois prosseguir...

Bugs Resolvidos:
      
Bugs Atuais:
#B04 - muitas colisoes quando se coloca 3 ou 4 nós da rede. Parece que o CSMA-CA nao esta corretamente implementado.

-----------------------------------------------------------------
Version: V8_0_3
Date: 20_01_16
Remarks:
AMAC.C - Foi corrigido alguns problemas de erro para o plocotolo AMAC
         movido as rotinas de RITQUEUE para o arquivo OpenQueue pois AMAC esta com mais de 5000 linhas e estava 
         mostrando msg de erros.
         Foi testado para 2 nos com o protocolo AMAC e esta com valores mais estaveis de Comandos Aciclicos.
		 Foi incluido o arquivo RITMC mas ainda eh somente para testes (nao esta funcionando).

Bugs Resolvidos:
#R01 - AMAC SINK - O RITPeriod as vezes eh de 1000 as vezes eh de 2000. Porem para o MOTE nao eh assim.
       R.: Isto esta relacionado com a rotina inicial IT0 onde quando eh sink ele tem de abrir a janela de inputserial 
	   para o openvisualizer. Nesta hora o RIT nao executa...ou seja, perde um ciclo sem comunicacao. 
#R02 - AMAC - com valores altos de RITwindow comeca a apresentar problemas.
       R.: estou me baseando nos mesmos timer do TSCH. Mas o Schedule_timer eh um timer de 15 bits que vai de 0 a 16500.
	   Entao para valores de RITWindows acima de 400ms ele vai apresentar problemas...

#R03 - Hoje nao esta ocorrendo o travamento da comunicacao quando tem TX pendente. por que ?
      e como resolver o caso onde transmissor tem um Tx.dio pendente...porem o receptor esta enviando um Tx.Dao..
      Neste caso o transmissor nao vai aceitar nada alem do Ola...entao ele fica aguardando o ola...
      porem o receptor esta aguardando um ack...ou um ola tambem...
      R.: Quando ocorre o timer principal entao comeca um novo slot...e entao ele fica ciclando entre Tx e Rx ou seja,
	  se no ultimo ciclo foi Tx entao no proximo eh RX... Nao tem o inverso..
      
Bugs Atuais:
 
-----------------------------------------------------------------
Version: V8_0_2
Date: 16_01_16
Remarks:
Versao com a medicao de consumo tanto para RIT quanto para o AMAC.
Esta versao eh que foi feito o artigo do I2MTC.

Bugs Resolvidos:

Bugs Atuais:
#01 - AMAC SINK - O RITPeriod as vezes eh de 1000 as vezes eh de 2000. 
      Porem para o MOTE nao eh assim.
#02 - AMAC - com valores altos de RITwindow comeca a apresentar problemas

#03 - Hoje nao esta ocorrendo o travamento da comunicacao quando tem TX pendente. por que ?
      e como resolver o caso onde transmissor tem um Tx.dio pendente...porem o receptor esta enviando um Tx.Dao..
      Neste caso o transmissor nao vai aceitar nada alem do Ola...entao ele fica aguardando o ola...
      porem o receptor esta aguardando um ack...ou um ola tambem...
      Eho o caso abaixo..veja que o transmissor perdeu o dio...e acabou recebendo o DAO com sucesso...
      
      
 

-----------------------------------------------------------------
Version: V8_0_1
Date: 11_01_15
Remarks:
Esta versao esta com o rit mais estavel...com log de consumo...
ja esta com o codigo do AMAC porem ainda nao testado

#01 - AMAC SINK - O RITPeriod as vezes eh de 1000 as vezes eh de 2000. 
      Porem para o MOTE nao eh assim.
#02 - AMAC - com valores altos de RITwindow comeca a apresentar problemas

-----------------------------------------------------------------
Version: V8_0_1
Date: 11_01_15
Remarks:
Esta versao esta com o rit mais estavel...com log de consumo...
ja esta com o codigo do AMAC porem ainda nao testado

#01 - AMAC SINK - O RITPeriod as vezes eh de 1000 as vezes eh de 2000. 
      Porem para o MOTE nao eh assim.


-----------------------------------------------------------------
V8_0_0
Remarks:
Esta versao tem integrado a placa de medicao de consumo configurado no arquivo board.h
funcionando no stack sem RIT.
Defines IEEE802154E_RIT 0 --> usando o OpenWSN padrao.
MYLINKXS_SENSORS 1 -> habilita os sensores implementados ate agora.
SENSOR_ACCEL = 1 --> foi testado nesta versao...leitura do acelerometro da placa RF06 (Texas)
SONOMA14 = 1 --> foi testado nesta versao...habilita a leitura da placa de medicao de consumo
MYLINKXS_REMOTE_CONTROL - NAO TESTADO NESTA VERSAO. Versao do controle remoto com arduino
MYLINKXS_LIGHT_CONTROL - NAO TESTADO NESTA VERSAO. Controle da lampada em um GPIO direto na CC2538.
MYLINKXS_SENSORS 0 seria a validacao da placa vermelha do marcelo barros. Mas nao foi testada nesta versao.



cliente coap do marcelo
------------------------
esta na pasta C:\OWSNRIT8\ClientCoap\coap
sao dois programas:
coap_console.py
sensor_data_scanner.py


Rodando firmware no python
--------------------------
1o passo) compilar o scons
entrar no diretorio do firwmare (C:\OWSNRIT8\openwsn)
scons board=python toolchain=gcc oos_openwsn
dica:apagar a pasta build (C:\OWSNRIT8\openwsn\build)



RENATO Fernandes 06/02/15
  
Esta versao eh a ultima versao do site de berkley



