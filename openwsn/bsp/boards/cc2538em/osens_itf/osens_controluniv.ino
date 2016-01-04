#include <osens.h>
#include <IRremote.h>

#define LOG_SERIAL 0


int RECV_PIN = 11;  //Pino de Entrada de dados do sensor
int SEND_PIN = 12;  //Pino de Saida  de dados do sensor
int LED_PIN  = 13;  // the number of the LED pin
int returnok=0;

#define MAX_NR_POINTS 5

#define PARAM1_ONOFF 0x80C
#define PARAM2_PLUS  0x820
#define PARAM2_MINUS 0x821
#define PARAM3_PLUS  0x810
#define PARAM3_MINUS 0x811

/** Sensor interface standard datatypes */
enum osens_datatypes_e
{
	OSENS_DT_U8     = 0x00, /**< 8 bits unsigned */
	OSENS_DT_S8     = 0x01, /**< 8 bits signed */
	OSENS_DT_U16    = 0x02, /**< 16 bits unsigned */
	OSENS_DT_S16    = 0x03, /**< 16 bits signed */
	OSENS_DT_U32    = 0x04, /**< 32 bits unsigned */
	OSENS_DT_S32    = 0x05, /**< 32 bits signed */
	OSENS_DT_U64    = 0x06, /**< 64 bits unsigned */
	OSENS_DT_S64    = 0x07, /**< 64 bits signed */
	OSENS_DT_FLOAT  = 0x08, /**< IEEE 754 single precision */
	OSENS_DT_DOUBLE = 0x09, /**< IEEE 754 double precision */
};

enum osens_ans_status_e
{
   OSENS_ANS_OK = 0,
   OSENS_ANS_ERROR = 1,
   OSENS_ANS_CRC_ERROR = 2,
   OSENS_ANS_READY_ONLY = 3,
   OSENS_ANS_WRITE_ONLY = 4,
   OSENS_ANS_REGISTER_NOT_IMPLEMENTED = 5,
};

uint16_t paramvalue[MAX_NR_POINTS] = {
PARAM1_ONOFF,
PARAM2_PLUS,
PARAM2_MINUS,
PARAM3_PLUS,
PARAM3_MINUS
};

#define BYTE_END_FRAME 0x5A

#define MAX_BUFFER 10

IRrecv irrecv(RECV_PIN);
IRsend irsend;
Osens osens();
uint8_t cnt = 0;
uint8_t rx_msg_len=0;
uint8_t rxbuf[128];
uint8_t newmsg=0;
uint16_t param1buf[MAX_BUFFER];
uint16_t param1value;
uint8_t newconfig=0;
uint8_t cfgcount=0;


uint8_t ledState=0;
decode_results results;


void setup()
{
  pinMode(LED_PIN, OUTPUT);   
  pinMode(SEND_PIN,OUTPUT);

  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }  
  // Start the receiver
  irrecv.enableIRIn(); 
  cfgcount = 0;
  cnt = 0;
 
}

void zerabuffer(uint16_t *buf)
{
  uint8_t ui;
  
  for(ui=0;ui<MAX_BUFFER;ui++)
  {
    buf[ui]=0;
  }
}

uint16_t findbetter(uint16_t *buf)
{
  uint8_t i,j,flagfindbetter;
  uint8_t maxcount,count;
  uint16_t better,auxbetter;
  
  auxbetter = buf[0];
  flagfindbetter=0;
  maxcount = 5;
  count=0;
  //conta qtos elementos tem com o mesmo valor...se maior que 5 esta bom.
  while (flagfindbetter == 0)
  {  
    for(i=0;i<MAX_BUFFER;i++)
    {
      if (auxbetter == buf[i]) 
         count++;
    }
    
    if (count > maxcount)
    {
      flagfindbetter=1;
      better = auxbetter;
      count=0;
    }
    else
    {
      maxcount =  count;  
      j=j+1;
      if (j > MAX_BUFFER)
      {
        flagfindbetter=1;
        better = auxbetter;
        count=0;
      }
      else
      {
         auxbetter = buf[j];
         count=0;
      }  
    }
  }
  
  return better;
}


void toggleled(uint8_t led)
{
    if (ledState)
      ledState = 0;
    else
      ledState = 1;
      
  digitalWrite(led,ledState);    
}


/* caracter final de frame 0x90 = 'Z' */
uint8_t receiveframe (uint8_t *buf)
{
   uint8_t ucSerial;
   uint8_t uclen=0;
   uint8_t framefim=0;
   int quant_char;
   
  if (Serial.available() > 0) 
  {
    delay(100); // aguarda o buffer da serial 
 
    quant_char = Serial.available(); // retorna o numero de caracteres na serial
    if(quant_char > MAX_BUFFER)
    {
      quant_char = MAX_BUFFER;
    }
    
    while(quant_char--)
    {
        ucSerial = Serial.read();
        //detecta caracter de final de frame
        if (ucSerial == BYTE_END_FRAME)
        {
           uclen=cnt;
           cnt=0;
        }
        else
        {
          buf[cnt++] = ucSerial;        
        }  

    }
  }
  return uclen; 
}

void sendframe (uint8_t *buf, uint8_t len)
{
#if LOG_SERIAL
    /* imprime msg de retorno no PC */
    Serial.print(len, HEX);
    Serial.print("]=[");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
#else
    for (int i = 0; i < len; i++) {
      Serial.write(buf[i]);
    }
    
    Serial.write(BYTE_END_FRAME);
#endif
}

void  treatreadrequest(uint8_t param,uint8_t *rxbuf)
{
  uint8_t txbuf[20];
  uint8_t *pvalue;
  uint8_t pos=0,idx=0;
  
  if (newconfig){
    newconfig=0;
  }
  
  idx = param - SENS_ITF_REGMAP_READ_POINT_DATA_1;
  
  if (idx < MAX_NR_POINTS)
  {
    pvalue = (uint8_t *) &paramvalue[idx];
    
    txbuf[pos++] = 6;             //length  
    txbuf[pos++] = rxbuf[1];      //addr 
    txbuf[pos++] = OSENS_ANS_OK;           //status OK
    txbuf[pos++] = OSENS_DT_U16;  //type 
    txbuf[pos++] = *(pvalue+0);   //value
    txbuf[pos++] = *(pvalue+1);   //value
   
  }
  else
  {
    txbuf[pos++] = 6;             //length  
    txbuf[pos++] = rxbuf[1];      //addr 
    txbuf[pos++] = OSENS_ANS_REGISTER_NOT_IMPLEMENTED;           //status ERROR 
    txbuf[pos++] = OSENS_DT_U16;  //type 
    txbuf[pos++] = 0;   //value
    txbuf[pos++] = 0;   //value
  }
  
  sendframe(txbuf,pos);
}

void treatwriterequest(uint8_t param,uint8_t *rxbuf)
{
  uint8_t txbuf[20];
  uint8_t *pvalue;
  uint8_t pos=0,idx=0,returnok = 0;
  uint8_t datalen;
  uint8_t *pucdata;
  uint16_t uidataval;
  uint8_t uc;
  
  datalen = rxbuf[2];
  pucdata = (uint8_t *) &uidataval;
  *pucdata++ = rxbuf[3];  
  *pucdata   = rxbuf[4];
      
  returnok = 0; 
  
  for (uc=0;uc<datalen;uc++)
  {
    txbuf[uc] = rxbuf[uc];
  }
 
#if LOG_SERIAL
  switch (rxbuf[3]) 
  {
    case '1' : 
     uidataval = PARAM1_ONOFF;
     Serial.print("WriteValue1: ");
     Serial.println(uidataval,HEX);
     returnok =1;
     break;
    case  '2':      
     uidataval = PARAM2_PLUS;
     Serial.print("WriteValue2: ");
     Serial.println(uidataval, HEX);
     returnok =1;
     break;
    case '3' :      
     uidataval = PARAM2_MINUS;
     Serial.print("WriteValue3: ");
     Serial.println(uidataval, HEX);
     returnok =1;
     break;
    case '4' :      
     uidataval = PARAM3_PLUS;
     Serial.print("WriteValue4: ");
     Serial.println(uidataval, HEX);
     returnok =1;
     break;
    case '5' :      
     uidataval = PARAM3_MINUS;
     Serial.print("WriteValue5: ");
     Serial.println(uidataval, HEX);
     returnok =1;
     break;
   default:
     Serial.print("ERROR: ");
     Serial.println(uidataval, HEX);
     returnok =0;
     break;
  }
#else
  switch (uidataval) 
  {
    case PARAM1_ONOFF : 
    case PARAM2_PLUS :      
    case PARAM2_MINUS :      
    case PARAM3_PLUS :      
    case PARAM3_MINUS :      
     returnok =1;
     break;
   default:
     returnok =0;
     break;
  }
#endif

    if (returnok)
    {
#if (LOG_SERIAL == 0)      
      //prepara a resposta para o mestre
      datalen = 5;
      pos = 0;
      txbuf[pos++] = datalen;       //length  
      txbuf[pos++] = rxbuf[1];      //addr 
      txbuf[pos++] = OSENS_ANS_OK;           //status OK 
      txbuf[pos++] = 0;             //crc 
      txbuf[pos++] = 0;             //crc
#endif

    //envia comando para o sensor
      for (int ii=0; ii < 3; ii++){
        irsend.sendRC5(uidataval, 12);
      }
       
       
    }
    else
    {
        //prepara a resposta para o mestre
        datalen = 5;
        pos = 0;
        txbuf[pos++] = datalen;       //length  
        txbuf[pos++] = rxbuf[1];      //addr 
        txbuf[pos++] = OSENS_ANS_REGISTER_NOT_IMPLEMENTED;          //status ERROR 
        txbuf[pos++] = 0;             //crc 
        txbuf[pos++] = 0;             //crc
    }   
            
      sendframe(txbuf,datalen);

}




void loop() 
{
  int i;
  uint8_t len=0;
  uint8_t ret=0;
  //uint8_t cmd=0;
  uint8_t returnok = 0;
  uint8_t ucSerial=0,addr=0;
  sens_itf_cmd_req_t cmd;
  sens_itf_cmd_res_t ans;


    //leitura do frame da serial
    len = receiveframe(rxbuf);

    if (len >0)
    {
      //esta eh a pergunta do mote de leitura ciclica
#if LOG_SERIAL
      //parametro '20'
      if ((rxbuf[0]==0x32)){
#else
      if ((rxbuf[0]==0x02)){
#endif
         treatreadrequest(rxbuf[1],&rxbuf[0]);
      }
#if LOG_SERIAL
      //parametro '5P'  && (rxbuf[1] == SENS_ITF_REGMAP_WRITE_POINT_DATA_1)
      else if ((rxbuf[0]==0x35))
#else // parametro '05 50 02 20 08 CRC 5A'
      else if ((rxbuf[0]==0x05))
#endif
      {
         toggleled(LED_PIN);
         treatwriterequest(rxbuf[1],&rxbuf[0]);
      }
      else
      {
        rxbuf[0] |= 0x80;
        sendframe(rxbuf,len);
      }
  }
  
  //TODO!!!!! configuracao do controle
  //aqui a ideia é somente passar para a placa de cima que a config esta ok.
  if (irrecv.decode(&results)) 
  {
    param1buf[cfgcount++] = results.value;
    Serial.println(results.value, HEX);

    if (cfgcount>MAX_BUFFER){
       param1value = findbetter(&param1buf[0]);
#if LOG_SERIAL
       Serial.print("Param1Value: ");
       Serial.println(param1value, HEX);
#endif
       zerabuffer(&param1buf[0]);
       cfgcount=0;
    }       

    irrecv.resume(); // Receive the next value
  }
  
   delay(1);
}
