#include <LocoNet.h>
#include <EEPROM.h>

/******************************************************************************
 *
 *  Copyright (C) 2014 Daniel Guisado - http://www.clubncaldes.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program; if not, If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************
 *
 * DESCRIPTION
 *
 * This is a Demo for a LNCV outputs and sound module.
 *
 * VERSION 8
 * Configurable intensity for each fade output
 * Playing the sound that you are configuring to identify them
 * 8.1 Bug fixing, fade not working
 * When breaking a sound it continues after
 ******************************************************************************/
// uncomment this to debug
#define DEBUG

#define SW_VERSION 1

#define LOCONET_TX_PIN 7
#define CV_COUNT 80

#define CV_MODULENUM 1
#define CV_ADRPIN1 2
#define CV_CONFIGPIN 18
#define CV_INTENS 34
#define CV_FADESPEED 40
#define CV_FIRSTSOUND 41
#define CV_NUMSOUNDS 42
#define CV_MASTERVOL 43
#define CV_SOUNDSOURCE 44
#define CV_STOPSOUND 45
#define CV_PLAYRANDOM 46
#define CV_SNDCFG_OFFSET 50

#define CV_COMMAND 100

#define MP3_SDCARD 0xA0
#define MP3_SPI    0xA1
#define MP3_UDISK  0xA2
#define MP3_MODESINGLE 0x00
#define MP3_MODEREPEAT 0x01
#define MP3_MODEREPEATALL 0x02
#define MP3_MODERANDOM 0x03

uint8_t lncv[CV_COUNT];
lnMsg *LnPacket;
LocoNetCVClass lnCV;

boolean modeProgramming = false;

byte MP3_source;  //Source of sounds SDCARD | SPI | UDISK
unsigned char cmd_buf[10];

/* Pin assignment to each output */
uint8_t myPins[] = {2,3,4,5,6,9,10,11,12,13,14,15,16,17,18,19};

/*************************************************************************/
/*          SETUP FUNCTION                                               */
/*************************************************************************/
void setup()
{
  uint8_t i = 0;
  
  /*----------------------------------------------*/
  /*  Configuracion Loconet                       */
  /*----------------------------------------------*/
  LocoNet.init(LOCONET_TX_PIN);  
  
  /*----------------------------------------------*/
  /*  Carga la configuraciÃ³n de la aplicacion     */
  /*----------------------------------------------*/
  loadSettings();  

  if (lncv[CV_MODULENUM]>254)
    resetSettings();

  /*----------------------------------------------*/
  /*  Configuracion Pines                         */
  /*----------------------------------------------*/
  for (i = 0; i < 16; i++)
  {
    pinMode(myPins[i], OUTPUT);
    if (lncv[i + 17] == 5 || lncv[i + 17] == 0)
      digitalWrite(myPins[i], HIGH);
    else
      digitalWrite(myPins[i], LOW);
  }

  /*----------------------------------------------*/
  /*  Configuracion MP3 Shield                    */
  /*----------------------------------------------*/
  Serial.begin(9600);

  delay(1000);
  
  if (lncv[CV_FIRSTSOUND]>0)
  {
    if (lncv[CV_MASTERVOL] > 30) lncv[CV_MASTERVOL] = 15;
    MP3setVolume(lncv[CV_MASTERVOL]);

    MP3playMode(MP3_MODESINGLE);

    //Plays first file in internal memory
    MP3_source = MP3_SPI;
    MP3playFile(1);

    //Sets MP3 source
    if (lncv[CV_SOUNDSOURCE] < 3 && lncv[CV_SOUNDSOURCE] >= 0)
      MP3_source = 0xA0 + lncv[CV_SOUNDSOURCE];
    else
      MP3_source = MP3_UDISK;
  }

  #ifdef DEBUG
  Serial.print("Starting sound module CV Address "); Serial.println(lncv[CV_MODULENUM]);  
  Serial.print("Version "); Serial.println(SW_VERSION);
  Serial.println("Daniel Guisado - http://www.clubncaldes.com");
  #endif
}

/*************************************************************************/
/*          MAIN LOOP                                                    */
/*************************************************************************/
void loop()
{
  LnPacket = LocoNet.receive();

  if (LnPacket)
  {    
    uint8_t packetConsumed = LocoNet.processSwitchSensorMessage(LnPacket);
    // Chequea si es un comando de configuración de CVs
    if (packetConsumed == 0)
      processCVMessage(LnPacket);
  }
  
  while (Serial.available()>0) Serial.read(); // discard mp3 answers
}


/*************************************************************************/
/*          LOCONET SWITCH AND SENSOR FUNCTIONS                          */
/*************************************************************************/
void notifySensor( uint16_t Address, uint8_t State )
{
  #ifdef DEBUG
    Serial.print("LOCONET Sensor: ");
    Serial.print(Address, DEC);
    Serial.print(" - ");
    Serial.println( State ? "Active" : "Inactive" );
  #endif
}

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int i = 0;

  #ifdef DEBUG
    Serial.print("LOCONET Switch Request: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif

  if (!Output) return;

  for (i = 0; i < 16; i++)
  {
    //Buscamos si un pin tiene asignado este numero de salida
    if (lncv[i+CV_ADRPIN1] == Address)
    {
      #ifdef DEBUG
        Serial.print("CHANGE OUTPUT "); Serial.print(Address, DEC);
        Serial.print(", PIN "); Serial.println(myPins[i], DEC);
      #endif
      Direction ? setOutput(i, true) : setOutput(i, false);
      break;
    }    
  }

  //Solo aceptamos seÃ±al verde para lanzar el sonido
  if (!Direction) return;

  //Si no corresponde a un pin miramos si esta asignado a un sonido
  if (Address >= lncv[CV_FIRSTSOUND] && Address < lncv[CV_FIRSTSOUND] + lncv[CV_NUMSOUNDS])
  {
    #ifdef DEBUG
      Serial.print("PLAY SOUND ");
      Serial.println(Address - lncv[CV_FIRSTSOUND] + 1);
    #endif
    int direccion = Address - lncv[CV_FIRSTSOUND];

    // If playing and low priority skip command
    if (MP3isPlaying() && (lncv[CV_SNDCFG_OFFSET+direccion] & 0x20))
      return;
    
    // If volume set
    int mp3volume=(lncv[CV_SNDCFG_OFFSET+direccion] & 0x1F);
    if (mp3volume>0)
      MP3setVolume(mp3volume);
    else
      MP3setVolume(lncv[CV_MASTERVOL]);
      
    // If loop
    if (lncv[CV_SNDCFG_OFFSET+direccion] & 0x40)   
      MP3playMode(MP3_MODEREPEAT);
    else
      MP3playMode(MP3_MODESINGLE);

    if ((MP3_source==MP3_SPI) && !(lncv[CV_SNDCFG_OFFSET+direccion] & 0x20))
      MP3playFileBreak(direccion + 1);
    else
      MP3playFile(direccion + 1);
      
    return;
  }
  if (Address == lncv[CV_STOPSOUND])
  {
    MP3Stop();
    return;
  }
  if (Address == lncv[CV_PLAYRANDOM])
  {
    //Not if sound in progress
    if (MP3isPlaying()) return;

    //Search for a background sound
    int counter=0;
    randomSeed(millis());
    int rndnumber=random(1, lncv[CV_NUMSOUNDS]);
    
    do {
      rndnumber=random(1, lncv[CV_NUMSOUNDS]);
      counter++;
    } while ((lncv[CV_SNDCFG_OFFSET+rndnumber] & 0x100==0) && counter<30);
    
    if (counter>30) return;

    #ifdef DEBUG
    Serial.print("Playing random sound # ");
    Serial.println(rndnumber+1);    
    #endif
    
    MP3playFile(rndnumber+1);
  } 
}

void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
    Serial.print("LOCONET Switch Report: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif
}

void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
    Serial.print("LOCONET Switch State: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif
}


/*************************************************************************/
/*          SOUND FUNCTIONS (SAINSMART MP3)                              */
/*************************************************************************/
void ArduinoMP3Shield_SendCMD(unsigned char *cmd_buf, unsigned len)
{
  unsigned i;

  for (i = 0; i < len; i++) {
    Serial.write(cmd_buf[i]);
  }
}

void MP3setVolume(byte pVolume)
{
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x03;          // Length
  cmd_buf[2] = 0xA7;          // Command
  cmd_buf[3] = pVolume;       // new volume
  cmd_buf[4] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

void MP3playFile(uint8_t pFileNum)
{
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x04;          // Length
  cmd_buf[2] = MP3_source;    // Command
  cmd_buf[3] = 0x00;          // file number high byte
  cmd_buf[4] = pFileNum;      // file number low byte
  cmd_buf[5] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 6);

  #ifdef DEBUG
    Serial.print("Play single");    
    Serial.print("\n");
  #endif
} //end playFile

void MP3playFileBreak(uint8_t pFileNum)
{
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x04;          // Length
  cmd_buf[2] = 0xAC;          // Command
  cmd_buf[3] = 0x00;          // file number high byte
  cmd_buf[4] = pFileNum;      // file number low byte
  cmd_buf[5] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 6);
  #ifdef DEBUG
    Serial.print("Play break");    
    Serial.print("\n");
  #endif
} //end playFile

void MP3playMode(uint8_t modo)
{
  // set play mode sing
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x03;          // Length
  cmd_buf[2] = 0xA9;          // Command SET MODE
  cmd_buf[3] = modo;          // set mode
  cmd_buf[4] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

void MP3Stop()
{
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x02;          // Length
  cmd_buf[2] = 0xA4;          // Command
  cmd_buf[3] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

boolean MP3isPlaying()
{
  int incomingByte = 0;
  
  while (Serial.available()>0) Serial.read();
  
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x02;          // Length
  cmd_buf[2] = 0xC2;          // Command
  cmd_buf[3] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
  while (Serial.available() < 2) {} //wait answer
  // read the first incoming byte:
  incomingByte = Serial.read();    
  incomingByte = Serial.read();
  
  if (incomingByte==1)
    return (true);    
  else
    return (false);
}

/*************************************************************************/
/*          EEPROM FUNCTIONS                                             */
/*************************************************************************/
void resetSettings()
{
  #ifdef DEBUG
  Serial.print("RESET MODULE CONFIGURATION!!");
  #endif

  lncv[0] = SW_VERSION;
  lncv[CV_MODULENUM]=3;
  uint8_t i;
  for (i = CV_ADRPIN1; i < CV_ADRPIN1+16; i++) lncv[i] = i;
  for (i = 18; i < 34; i++) lncv[i] = 1;
  lncv[19] = 4; lncv[21] = 4; lncv[22] = 4; lncv[23] = 4; lncv[24] = 4; lncv[25] = 4; //Salidas posibles con fade
  for (i = 34; i < 40; i++) lncv[i] = 255; //intensidades fade
  lncv[CV_FADESPEED] = 5; //velocidad fade
  lncv[CV_FIRSTSOUND] = 0; lncv[CV_NUMSOUNDS] = 0; lncv[CV_MASTERVOL] = 15;
  lncv[CV_SOUNDSOURCE] = 1; lncv[CV_STOPSOUND] = 0; //Config MP3, sin sonidos, volumen 15, SPI, salida STOP
  lncv[CV_PLAYRANDOM] = 0;
  for (i = CV_SNDCFG_OFFSET; i<(CV_SNDCFG_OFFSET+30); i++) lncv[i]=0;
  
  for (i = 0; i < CV_COUNT; i++)
  {
    EEPROM.update(i, lncv[i]);    
  }
}

void loadSettings()
{
  int i = 0;

  //Check if there is no configuration stored or major version changed to reset
  /* no version check
  if (EEPROM.read(0) != MAJORVERSION)
  {
    resetSettings();
    return;
  } */

  for (i = 0; i < CV_COUNT; i++)
  {
    lncv[i] = EEPROM.read(i);    
  }  
  lncv[0]=SW_VERSION;
}

/*************************************************************************/
/*          OUTPUT FUNCTIONS                                             */
/* pOut es la salida 0-15
/*************************************************************************/
void setOutput(uint8_t pOut, boolean pState)
{
  int intens;
  int maxintens;

  if (pOut==1)
    maxintens=lncv[CV_INTENS];
  if (pOut>2 and pOut<8)
    maxintens=lncv[CV_INTENS+pOut-2];
  
  if (maxintens>255) maxintens=255;
  if (maxintens<1) maxintens=1;
  
  //Check output configuration
  switch (lncv[pOut + CV_CONFIGPIN])
  {
    case 0: // Output invertida
      #ifdef DEBUG
        Serial.print("OUTPUT INVERSE "); Serial.println(pState);
      #endif
      pState ? digitalWrite(myPins[pOut], LOW) : digitalWrite(myPins[pOut], HIGH);      
      break;
    case 1: // Output normal
      #ifdef DEBUG
        Serial.print("OUTPUT NORMAL "); Serial.println(pState);
      #endif
      pState ? digitalWrite(myPins[pOut], HIGH) : digitalWrite(myPins[pOut], LOW);
      break;
    case 2: // Pulse Thrown
    case 3: // Pulse Straight
      #ifdef DEBUG
        Serial.print("PULSE "); Serial.println(pState);
      #endif
      digitalWrite(myPins[pOut], HIGH);
      delay(100);
      digitalWrite(myPins[pOut], LOW);
      break;
    case 4: // Fade
      #ifdef DEBUG
        Serial.print("FADE "); Serial.println(pState);
      #endif
      if (pState)
      {
        for (intens = 0; intens <= maxintens; intens++)
        {
          analogWrite(myPins[pOut], intens);
          delay(lncv[CV_FADESPEED]);         
        }        
      }
      else
      {
        for (intens = maxintens; intens >= 0; intens--)
        {
          analogWrite(myPins[pOut], intens);
          delay(lncv[CV_FADESPEED]);
        }
      }
      break;
    case 5: // Fade invertido
      #ifdef DEBUG
        Serial.print("FADE INVERSE "); Serial.println(pState);
      #endif
      if (!pState)
      {
        for (intens = 0; intens <= maxintens; intens++)
        {
          analogWrite(myPins[pOut], intens);
          delay(lncv[CV_FADESPEED]);
        }
      }
      else
      {
        for (intens = maxintens; intens >= 0; intens--)
        {
          analogWrite(myPins[pOut], intens);
          delay(lncv[CV_FADESPEED]);
        }
      }      
      break;
  }
  Serial.println("fin funcion");
}

/*************************************************************************/
/*          CV PROGRAMMING FUNCTIONS                                     */
/*************************************************************************/


void processCVMessage(lnMsg *LnPacket) 
{ 
   
  uint8_t cvnum=0;
  uint8_t cvvalue=0;
  int n=0;
    
  unsigned char opcode = (int)LnPacket->sz.command;

  #ifdef DEBUG
  Serial.print("RX: ");
  uint8_t msgLen = getLnMsgSize(LnPacket); 
  for (uint8_t x = 0; x < msgLen; x++)
  {
    uint8_t val = LnPacket->data[x];
      // Print a leading 0 if less than 16 to make 2 HEX digits
    if(val < 16)
      Serial.print('0');
      
    Serial.print(val, HEX);
    Serial.print(' ');
  }
  Serial.println(" <");
  #endif
  
  if (opcode==OPC_WR_SL_DATA) //Programming track
  {
    #ifdef DEBUG
    Serial.println("OPC_WR_SL_DATA Received");
    #endif
      /*------------------------------------------------------------------------
      This OPC leads to immediate LACK codes:
      <B4>,<7F>,<7F>,<chk> Function NOT implemented, no reply.
      <B4>,<7F>,<0>,<chk> Programmer BUSY , task aborted, no reply.
      <B4>,<7F>,<1>,<chk> Task accepted , <E7> reply at completion.
      <B4>,<7F>,<0x40>,<chk> Task accepted blind NO <E7> reply at completion.

      typedef struct progtask_t {
      uint8_t command;
      uint8_t mesg_size;      ummmmm, size of the message in bytes?                    
      uint8_t slot;           slot number for this request - slot 124 is programmer    
      uint8_t pcmd;           programmer command                                       
      uint8_t pstat;          programmer status error flags in reply message           
      uint8_t hopsa;          Ops mode - 7 high address bits of loco to program        
      uint8_t lopsa;          Ops mode - 7 low  address bits of loco to program        
      uint8_t trk;            track status. Note: bit 3 shows if prog track is busy    
      uint8_t cvh;            hi 3 bits of CV# and msb of data7                        
      uint8_t cvl;            lo 7 bits of CV#                                         
      uint8_t data7;          7 bits of data to program, msb is in cvh above           
      uint8_t pad2;
      uint8_t pad3;
      uint8_t chksum;         exclusive-or checksum for the message                
      } progTaskMsg;

      #define PRG_SLOT          0x7c      This slot communicates with the programming track    

      values and macros to decode programming messages 
      #define PCMD_RW           0x40       1 = write, 0 = read                                  
      #define PCMD_BYTE_MODE    0x20       1 = byte operation, 0 = bit operation (if possible)  
      #define PCMD_TY1          0x10       TY1 Programming type select bit                      
      #define PCMD_TY0          0x08       TY0 Programming type select bit                      
      #define PCMD_OPS_MODE     0x04       1 = Ops mode, 0 = Service Mode                       
      #define PCMD_RSVRD1       0x02       reserved                                             
      #define PCMD_RSVRD0       0x01       reserved                                             
      
      programming mode mask 
      #define PCMD_MODE_MASK      (PCMD_BYTE_MODE | PCMD_OPS_MODE | PCMD_TY1 | PCMD_TY0)

      programming modes
      -----------------
      Paged mode  byte R/W on Service Track 
      #define PAGED_ON_SRVC_TRK       (PCMD_BYTE_MODE)
      Direct mode byte R/W on Service Track 
      #define DIR_BYTE_ON_SRVC_TRK    (PCMD_BYTE_MODE | PCMD_TY0)
      Direct mode bit  R/W on Service Track
      #define DIR_BIT_ON_SRVC_TRK     (PCMD_TY0)
      Physical Register byte R/W on Service Track
      #define REG_BYTE_RW_ON_SRVC_TRK (PCMD_TY1)      
      Service Track Reserved function
      #define SRVC_TRK_RESERVED       (PCMD_TY1 | PCMD_TY0)      
      Ops mode byte program - no feedback
      #define OPS_BYTE_NO_FEEDBACK    (PCMD_BYTE_MODE | PCMD_OPS_MODE)      
      Ops mode byte program - feedback 
      #define OPS_BYTE_FEEDBACK       (OPS_BYTE_NO_FEEDBACK | PCMD_TY0)      
      Ops mode bit program - no feedback
      #define OPS_BIT_NO_FEEDBACK     (PCMD_OPS_MODE)      
      Ops mode bit program - feedback 
      #define OPS_BIT_FEEDBACK        (OPS_BIT_NO_FEEDBACK | PCMD_TY0)
      
      Programmer Status error flags
      #define PSTAT_USER_ABORTED  0x08    /* User aborted this command 
      #define PSTAT_READ_FAIL     0x04    /* Failed to detect Read Compare Acknowledge from decoder 
      #define PSTAT_WRITE_FAIL    0x02    /* No Write acknowledge from decoder                      
      #define PSTAT_NO_DECODER    0x01    /* Service mode programming track empty                   
      
      bit masks for CVH
      #define CVH_CV8_CV9         0x30    /* mask for CV# bits 8 and 9    
      #define CVH_CV7             0x01    /* mask for CV# bit 7           
      #define CVH_D7              0x02    /* MSbit for data value         
      
      build data byte from programmer message
      #define PROG_DATA(ptr)      (((ptr->cvh & CVH_D7) << 6) | (ptr->data7 & 0x7f))
      
      build CV # from programmer message
      #define PROG_CV_NUM(ptr)    (((((ptr->cvh & CVH_CV8_CV9) >> 3) | (ptr->cvh & CVH_CV7)) * 128) + (ptr->cvl & 0x7f))
      ------------------------------------------------------------------------*/

      //Check for programming slot    
      if (LnPacket->pt.slot!=PRG_SLOT)
        return;

      //PCMD value of 00 aborts current SERVICE mode programming and echo <E6>RD
      if (LnPacket->pt.pcmd==0x00)
        return;

      //Bit operations not implemented
      if (((LnPacket->pt.pcmd&PCMD_BYTE_MODE)==0) || ((LnPacket->pt.pcmd&PCMD_RW)>0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)>0))
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,0x7F);
        return;
      }
      
      cvnum=(((((LnPacket->pt.cvh & CVH_CV8_CV9) >> 3) | (LnPacket->pt.cvh & CVH_CV7)) * 128) + (LnPacket->pt.cvl & 0x7f));
      cvnum++;  
      cvvalue=(((LnPacket->pt.cvh & CVH_D7) << 6) | (LnPacket->pt.data7 & 0x7f));

      //Write module number in CV1 to activate programming, after you can change it
      if (cvnum==CV_MODULENUM && cvvalue==lncv[CV_MODULENUM])
      {
        modeProgramming=true;
        #ifdef DEBUG
        Serial.println("MODE PROGRAMMING ON");
        #endif
      }

      if (!modeProgramming)
        return;
        
      //READ ON PROGRAMMING TRACK      
      if ((LnPacket->pt.pcmd&PCMD_RW) == 0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)==0 && cvnum<CV_COUNT)
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,1);
        #ifdef DEBUG
        Serial.print("Read on programming track CV "); Serial.println(cvnum);        
        #endif

        /* <R CV CALLBACKNUM CALLBACKSUB>
        *    CV: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *    
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV VALUE)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
        */    
        if (!modeProgramming)
          return;
        
        cvvalue=lncv[cvnum];
        //<0xEF>,<0E>,<7C>,<PCMD>,<0>    ,<HOPSA>,<LOPSA>,<TRK>,<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
        //<0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>,<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
        LnPacket->pt.command=OPC_SL_RD_DATA;
        LnPacket->pt.pstat=0;
        LnPacket->pt.data7=cvvalue;        
        LocoNet.send(LnPacket);
        //if sound config lncv play this sound to identify it
        if ((cvnum>=CV_SNDCFG_OFFSET) && (cvnum<=CV_SNDCFG_OFFSET+30))
        {
          // If volume set
          int mp3volume=(lncv[cvnum]);
          if (mp3volume>0)
            MP3setVolume(mp3volume);
          else
            MP3setVolume(lncv[CV_MASTERVOL]);
          MP3playFile(cvnum-CV_SNDCFG_OFFSET+1);
        }   
      }      
      //WRITE ON PROGRAMMING TRACK
      if ((LnPacket->pt.pcmd&PCMD_RW)>0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)==0)
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,1);
        #ifdef DEBUG
        Serial.print("Write on programming track CV ");Serial.print(cvnum);Serial.print(" VALUE ");Serial.println(cvvalue);
        #endif
        /* <W CV VALUE CALLBACKNUM CALLBACKSUB>
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    VALUE: the value to be written to the Configuration Variable memory location (0-255) 
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *    
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV Value)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if verificaiton read fails
        */


        //Special CV_COMMAND to command Copy USB (10), Reset (99), it's not a stored CV
        if (cvnum == CV_COMMAND)
        {
          if (cvvalue == 99)
            resetSettings();                
          
          if (cvvalue == 10)
          {
            cmd_buf[0] = 0x7E;          // START
            cmd_buf[1] = 0x03;          // Length
            cmd_buf[2] = 0xAB;          // Command SET MODE
            cmd_buf[3] = 0x00;          // set mode
            cmd_buf[4] = 0x7E;          // END
            ArduinoMP3Shield_SendCMD(cmd_buf, 5);            
          }      
        }

        if (cvnum<CV_COUNT)
        {
          lncv[cvnum]=cvvalue;
          EEPROM.update(cvnum, lncv[cvnum]);

          LnPacket->pt.command=OPC_SL_RD_DATA;
          LnPacket->pt.pstat=0;
          LnPacket->pt.data7=cvvalue;
          LocoNet.send(LnPacket);            

          //Si escribimos la direccion asignada a la salida 1, ponemos el resto correlativas
          if (cvnum == 2)
          {
            for (n = 1; n < 16; n++)
            {
              lncv[2+n] = cvvalue + n;
              EEPROM.update(2+n,cvvalue + n);
            }
          }
        
        }               
    }
  }
} // LNetCmdStation::processIncomingLoconetCommand

