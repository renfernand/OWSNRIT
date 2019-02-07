OpenWSN firmware: stuff that runs on a mote

Part of UC Berkeley's OpenWSN project, http://www.openwsn.org/.

Update history

------------------------------------------------------------------
Version: V8_3_1_5
Date: 5-03-18
Remarks:
   - Last version of the Thesis. This version works fine using the Eclipse Platform and GCC compiler.
   
------------------------------------------------------------------
Version: V8_3_1_3
Date: 6-08-17
Remarks:
    - Implementado o TSTRFF_STOREMODE que permite mais de 5 saltos no OpenWSN com topologia forcada.
	- Agora tambem o COAP respnse tem o diagnostico TxOla, TxDio, TxDAO e TxCOAP.
	- Foi testado ate 11 saltos com o protocolo AMCA e esta funcionando (nao tao estavel).
     
------------------------------------------------------------------
Version: V8_3_1_2
Date: 19-06-17
Remarks:
    - Versao do artigo 1 - Funcionando tanto os testes de livelist quanto Coap para os 4 protocolos asincronos. 
------------------------------------------------------------------
      
Version: V8_3_1_1
Date: 19-03-17
Remarks:
    - Testes multicanal nos 4 protocolos. 
      
------------------------------------------------------------------
Version: V8_3_1_0
Date: 08-03-17
Remarks:
    - Versao do Eclipse baseado na versao V8.1.1.11 do CCS6.0. 
      Foi necessario migrar para o eclipse pois a versao do CCS nao estava funcionando mais rodando em flash, somente com o debuger...
      

------------------------------------------------------------------
Update history
Version: V8_1_1_11
Date: 21-02-17
Remarks:
        - Foi integrado a versao da blaca MBA e Texas no mesmo projeto. 
		  Procedimento para downlodar o firmware para uma placa especifica:
		  1) Escolher Properties do projeto e entao na aba geral escolher
		     Placa Texas: CPU SF53
		     Placa MBA  : CPU NF23
		  2)E entao escolher o arquivo linker com memorias diferentes:
		     Placa Texas: ld2538.lds
		     Placa MBA  : ld2538mba.lds

------------------------------------------------------------------
Update history
Version: V8_1_1_10
Date: 20-02-17
Remarks:
        - RITMCV2 - Melhorado o mecanismo de ECW...
        - Livelist - o flag FIXED_LIVELIST faz com que a livelist vire somente um comando de leitura e escrita ciclica...
                     a livelist eh fixa desde o inicio e nao eh influenciada pelas mas condicoes...
        - Esta versao que foram feitos os testes de consumo do AMCA e RITMC.
	
			
------------------------------------------------------------------
Update history
Version: V8_1_1_9
Date: 11-01-17
Remarks:
      Versao com a implementacao do AMAC versao 2:
      - O AMAC foi implementado provisoriamente com algumas caracteristicas diferentes do proposto no artigo:
        1) A proposta do mecanismo inicial do Tx é enviar o comando 0x800Rx para o nó receptor enviar da forma de um um ola com 
        este endereco...Neste caso o Receptor deveria checar os comandos de Ola e verificar se eh uma mudanca de seu endereco e comecar 
		a publicar o Ola para este Transmissor...Porem isso vai demandar muitos ciclos de Ola e fica inviavel a espera do comando de Tx
		no ambiente do OpenWSN...Pois ele vai esperar uma janela muito grande...ou ele vai esperar mais que 3 retries todo ciclo, inutilizando
		a funcao de retry...
		Desta forma como o objetivo aqui eh testar o tempo da msg...sera implementado na mesma janela...ou seja...o Tx espera um Ola do Rx
		e envia e entao espera a msg do receptor para entao enviar o hardware Ack...
      - Implementado e testado o RITMC e o AMAC somente enviando LIVELIST com uma taxa de 500ms.
      - NO RITMC foi implementado e testado a medicao do dutycycle 	  
	 	

------------------------------------------------------------------
Update history
Version: V8_1_1_8
Date: 04-01-17
Remarks:
      Versao com a implementacao do ARM e AMAC versao 1:
      - O ARM foi testado com single e multi canal e ente 4 saltos e esta funcionando ok.
      - O AMAC foi implementado provisoriamente com algumas caracteristicas diferentes do proposto no artigo:
        1) Esta versao ele nao tem o mecanismo inicial de Tx onde ele deve enviar um comando 0x800Rx para o nó receptor enviar um ola com 
        este endereco e ai sim ter um Hardware Ack...
        Esta versao tem o harware ack sempre ativo no OLA...o que eh contra a norma...e isso faz com que sempre em um comando de broadcast
        ele fique esperando um dado...
		2) O DataAck do artigo ele faz com o proprio OLA...mas aqui eh meio impossivel neste caso pois teria de ter um enderecamento mais 
		completo para enviar isso para as camadas superiores do openwsn...estou mantendo o ACK da forma do openwsn...
		

------------------------------------------------------------------
Update history
Version: V8_1_1_7
Date: 26-10-16
Remarks:
      - Versao com a implementacao do RITMC e Retries nas camadas superiores. 
	  - Foram feitos  com 5 nós somente 1 canal e com baixa perda de pacotes...MUITO BOMMMMMMM!!!!!
      - o RITMC AINDA NAO ESTA COMPLETO!!!! ELE NAO ESTA TRATANDO O FRAME DE CW...TESTES USANDO SEM_CW...
	  
Bugs Atuais:
BUG13 - MSG de ERRO NO SLOT 0
BUG14 ? Um aparente erro ocorre quando estou esperando um RXOLA (TX03) e a janela de 400ms estoura (TXe03). Aqui pode ser que do outro lado
tambem o outro nó esta esperando um ola meu, porem fica os dois esperando e nunca sera enviado. Entao falha.
 
------------------------------------------------------------------
Update history
Version: V8_1_1_6
Date: 12-08-16
Remarks:
     - Alterado o COAP para suportar o OpenVisNewDAG. Fiz o merge com a parte serial da ultima versao
   do openWSN. Tambem tenho o OpenVis e Coap mais novo.
     - Agora o codigo tanto do coap como do simulador de coap esta funcional. 
     - Esta testando tanto com o codigo Well-Known quanto com o do Marcelo Barros "/d" porem é aconselhavel
      o well-known pois o codigo /d esta voltando mensagem de estouro de memoria quando muitos saltos.
	 - A serial agora trata interrupcao de erros Overrun, Paridade, Framming error. E quando ocorrer erro ele reseta a serial 

 Bug#10 - Tres saltos nao envia COAP automatico - Resolvido
 Bug#11 - Travamento da escrita serial apos algum tempo - Resolvido com remedio 

Bugs Atuais:

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
 Testado e validado o DIO com multi-canal. Testado com 5 nÃ³s e funcionando OK.
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

Problema: Ainda esta ocorrendo o travamento do nÃ³ quando nao esta usando debug. 


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

Problema: Ainda esta ocorrendo o travamento do nÃ³ quando nao esta usando debug. 

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
 Nesta versao foi feito testes somente com o RIT para suportar atÃ© 2 saltos. 3 saltos nao conseguiu enviar o comando de DAO...
 tambem foi feito a tentativa de enviar o DIO para mais de um vizinho mas nao tive sucesso pois esta ocorrendo muitas colisoes...
 Sera necessario primeiro resolver os problemas das colisoes para depois prosseguir...

Bugs Resolvidos:
      
Bugs Atuais:
#B04 - muitas colisoes quando se coloca 3 ou 4 nÃ³s da rede. Parece que o CSMA-CA nao esta corretamente implementado.

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



