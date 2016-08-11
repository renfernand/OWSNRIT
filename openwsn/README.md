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
Version: V8_1_1_5
Date: 10-08-16
Remarks:
    Corrigido o bug #01 e Bug #08 que estava relacionado ao tamanho do STACK... Aumentado o stack de 128 para 512.
    Esta versao foi implementado o COAP interno, onde nao depende do openvis com config fixa para ate 3 saltos.
    Testado um e dois saltos automaticos...Tres saltos o SINK nao enviou o comando.   

Bugs Atuais:
 Bug#10 - Tres saltos nao envia COAP automatico
 Bug#11 - Travamento da escrita serial apos algum tempo 

------------------------------------------------------------------
Update history
Version: V8_1_1_4
Date: 25-07-16
Remarks:
  - Implementado retorno para step anterior quando ocorrer ERRO (RX83),(RX87). Anteriormente estava indo para o EndSlot.
  - testado o RPL.DAO multicanal

Bugs Atuais: #01, #02.
------------------------------------------------------------------
Update history
Version: V8_1_1_3
Date: 20-07-16
Remarks:
 Testado e validado o DIO com multi-canal. Testado com 5 nós e funcionando OK.
 Resolvido o BUG03 do Release.
 Implementado servico de livelist (multi-channel hello). Porem nao foi implementado conforme a norma. nao tem o retorno somente o ola no canal.

Bugs Atuais: #01, #02.

------------------------------------------------------------------
Update history
Version: V8_1_1_2
Date: 17-07-16
Remarks:
 Nesta versao somente foi testado o OLA.
 a) Foi alterado o mecanismo do OLA com uma versao bem mais estavel. Quase todos os Inicio de slot tem um EndSlot.
    Ainda esta ocorrendo o bug#02.
 b) Foi alterado o mecanismo de medicao do tempo do slot (getdeltaslotperiod_ms). Testes com osciloscopio OK.
 c) Tentei validar a versao release para utilizar no firmware com watchdog. Ainda nao funcionou com problema no assembler (BUG#03). 


Bugs Atuais: #01, #02, #03.


------------------------------------------------------------------

Update history
Version: V8_1_1_1
Date: 14-07-16
Remarks:
Esta versao foi implementado como remedio tambem o watchdog que eh habilitado no BOARD.H com um define. Fica ruim o debug com o watchdog ligado.
Tambem foram corrigidos problemas em relacao qdo em txmode nao recebesse olas..

Problema: Ainda esta ocorrendo o travamento do nó quando nao esta usando debug. 


------------------------------------------------------------------

Update history
Version: V8_1_1
Date: 12-07-16
Remarks:
Esta versao ja esta funcionando novamente o RPL-DIO usando a topologia.
Foi mudado o timer do slottime que agora esta usando o proprio MACTimer (exclusivo para o CSMA) porem somente utiliza ele como freerunning.
O que da para ver eh que as vezes o timer trava...nao gera mais interrupcao, porem um remedio paleativo foi quando travar a tarefa do schedule
reseta este timer. Desta forma ele esta funcionando bem.
Tambem foi feito uma mudanca para sempre a programacao dos timers dentro do slot resultar em um tempo menor que o slot time. Desta forma sempre
vai ocorrer um endslot.

Problema: Ainda esta ocorrendo o travamento do nó quando nao esta usando debug. 

------------------------------------------------------------------
Update history
Version: V8_1_1
Date: 24-06-16
Remarks:
Esta versao eh um teste do RIT Ola para validar o mecanismo do CSMA e tambem do RIT para
mais de um vizinho anunciando.
O teste consistiu de desabilitar o TX e somente ter o RX OLA e a recepcao.
Porem, aqui da para ver que ele sempre fica fixo em um vizinho e por mais que o outro anuncie
(vejo a anunciacao no wireshark) ele nao consegue ler info do cara.

-----------------------------------------------------------------

Version: V8_1_0
Date: 7-04-16
Remarks:
Nesta versao criei o AMCA multicanal

Bugs Resolvidos:
Agora o CSMA esta funcionando somente com o MAC timer.
A tarefa MAC roda em cima do systick programado de acordo com a necessidade.
A chamada do inicio do slot eh feita pela tarefa geral comum as outras tarefas.
O actualtime tambem esta funcionando...so que ele eh um incremento desde o inicio do slot.      

Bugs Atuais:
- As vezes a interrupcao do MAC Timer (CSMA) para de funcionar...apesar de continuar enviar o frame. Mas eu criei um mecanismo de quando travar eu resetar ela.

- Parece que nao esta funcionando quando eu coloco um outro no na rede.

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



