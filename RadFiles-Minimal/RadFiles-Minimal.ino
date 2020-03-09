/*

 E.  Formatting and Protocols
 
 Metadata File Header Format  (ASCII):
 "Future Ocean Lab Radiometer Data File \r\n"
 "Software Version: FOL_RAD_VV"
 " .....\r\n"
 "Data Rate: SAMPLES_PER_SEC  \r\n"
 "Data Block Size: 4096  \r\n"
 "Data Format: Microseconds [2B] Photon  Count [2B]   \r\n"
 "Data Header Size: 32B  \r\n"
 "Data Header Format: \"@@...(18 times)...@@\" EpochTime[4B] \
                                  NanoSeconds[4B] Tilt[6B] \r\n"
 " .....\r\n"
 "CRUISE NAME, SHIP NAME, etc \r\n"
 "YYYY:MM:DD HH:MM:SS \r\n"
 
 
 Data Chunk Header Format (binary):
 "@@...(18 times)...@@" EpochTimeUTC[4B] NanoSeconds[4B] Tilt[6B]
 32 Bytes [= 18+14]
 
 Data Point Format:
 [2B: uint16_t time since last ping in usec] 
 [2B: uint16_t Pulses]
 [2B: uint16_t TimeHi]
 6 Bytes
 
 Heartbeat Message Format
 “RAD.VV yyyy:mm:dd  hh:mm:ssZ DDDDDDDDDD HHHHH PPPPP RRRRR \r\n"
 
 
 
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



  
--------------------------------------------------------------------------------

  Dependencies: 
  
--------------------------------------------------------------------------------

  Issues:
  
------------------------------------------------------------------------------*/
#include "RadFiles-Minimal.h"

#define CRUISE_NAME "Lindblad / Channel Island, December 2019"
#define SHIP_NAME  "Of Opportunity"
#define RAD_NAME "Statler"
//#define RAD_NAME "Waldorf"


/*    Gloabl Variables: Files and Buffers      
*/

SdFs              sd;                      // 
FsFile            bfile[N_Files];          // 
FsFile            mfile;                   // 

int               CurrentFile = 0;         // 

size_t            RUs_Written = 0;         //
size_t            LastSec_TimeHi=0,        //
                  LastSec_Pulses=0;        // 

//Global Clocks
elapsedMillis     mclock;                  // for ms since last Heartbeat 

// GlobalFlags
volatile int      fHeartbeat    = FALSE;   //
volatile int      fPayload      = FALSE;   //
volatile int      fPowerDownNow = FALSE;   //
volatile int      fStopCount    = TRUE;    //
volatile bool     fHandlePings  = FALSE;   //  In place of dettachInterrupt, set to false
                  

volatile uint8_t  Payload[PAYLOAD_BYTES];            // x
uint8_t           UTC_Buffer[UTC_BUFFER_BYTES];         // x
uint32_t*         UTC_Buffer32   = (uint32_t*) &UTC_Buffer[5]; // 
const size_t      cpu_clicks_per_us =  F_CPU / 1000000;

const size_t      size_Ring = N_BUFS * SIZE_RU;    //
MAKE_RING_BUFFER(TeensyRing, size_Ring);          // 



/*    Read and Write Buffer wrapper functions     
*/

int Write_Ring_to_SD(void) {  // DONE
    if (bfile[CurrentFile].write(TeensyRing._ring + TeensyRing.Tail, SIZE_RU) \
                                                                  != SIZE_RU) {
        Serial.println("file.write failed");
        bfile[CurrentFile].close();
        return(ERR_FILE_WRITE_FAILED);  // if file write fails, panic with -1
    }
    
    noInterrupts();
    TeensyRing.Count -= SIZE_RU;
    TeensyRing.Tail = (TeensyRing.Tail + SIZE_RU) % TeensyRing.Size;
    interrupts();

    if((++RUs_Written) * SIZE_RU >= File_Length) {
      bfile[CurrentFile].close();
      if(++CurrentFile==N_Files) return(ERR_OUT_OF_FILES);
      RUs_Written = 0;
    }
    return(0);
}

int Write_Data_to_Ring(uint8_t *data, uint8_t data_len) {  // DONE
/*------------------------------------------------------------------------------ 

    Write_Data_to_Ring  :  Inserts data into the Ring
    
    Along the way it checks for 2 edge cases:
        A. Ring Full 
            ==> returns -1
        B. Data will wrap to or past the end of the Ring
            ==> splits data into chunks and writtes across the divide.

      Note: assume Write_Data_to_Ring will run inside an interrupt routine
            and can't itself be interrupted
      
------------------------------------------------------------------------------*/
  static size_t next,nibbleA,nibbleB;
  
  if(TeensyRing.Count + data_len > TeensyRing.Size) { // Buffer Full!!
    return(ERR_BUFFER_FULL); 
  } else { TeensyRing.Count += data_len; };
  // Once stored, count is  increased to reflect new data.
   
  // next is where head will point to after this write.
  next = TeensyRing.Head + data_len;  
    
  if (next < TeensyRing.Size) { // if next is inside ring, just do it
    memcpy(TeensyRing._ring + TeensyRing.Head, data, data_len);
    TeensyRing.Head = next;
  } else { // if next is outside ring, need to 
           // break up data chunk and write across modulo:
    // Data slots remaining at end of buffer
    nibbleA = TeensyRing.Size - TeensyRing.Head;
    // Length of leftover data to push into begining of buffer
    nibbleB = data_len - nibbleA;       
    memcpy(TeensyRing._ring + TeensyRing.Head,  data,           nibbleA);
    if(nibbleB>0) memcpy(TeensyRing._ring, data + nibbleA, nibbleB);
    TeensyRing.Head = nibbleB;
  }
  return 0;  // return success to indicate successful push.
}




/*      Helper Functions
 */

void errorHalt(const char* msg) {
  Serial1.print("Error: ");
  Serial1.println(msg);
  if (sd.sdErrorCode()) {
    if (sd.sdErrorCode() == SD_CARD_ERROR_ACMD41) {
      Serial1.println("Try power cycling the SD card.");
    }
    printSdErrorSymbol(&Serial1, sd.sdErrorCode());
    Serial1.print(", ErrorData: 0X");
    Serial1.println(sd.sdErrorData(), HEX);
  }
  while (true) {} 
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void dateTime(uint16_t* date, uint16_t* time) {
  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());
}

void sprintDateTime(char* sHTime,char* sFNTime) {
  TimeElements tm;
  breakTime(now(),tm);
  
  // "YYYY-MM-DD HH:MM:SS"
  sprintf(sHTime, "%.4u-%.2u-%.2u %.2u:%.2u:%.2u",1970+tm.Year,tm.Month, tm.Day,\
                                                       tm.Hour,tm.Minute,tm.Second);
  // "YYYY_MM_DD__HH_MM_SS"
  sprintf(sFNTime,"%.4u_%.2u_%.2u__%.2u_%.2u_%.2u",1970+tm.Year,tm.Month, tm.Day,\
                                                      tm.Hour,tm.Minute,tm.Second);
}

void Set_Ns(void) {
  digitalWrite(pin_NsSel[0],( (Current_Ns >> 0)  & 0x01  ? HIGH : LOW));
  digitalWrite(pin_NsSel[1],( (Current_Ns >> 1)  & 0x01  ? HIGH : LOW));
  digitalWrite(pin_NsSel[2],( (Current_Ns >> 2)  & 0x01  ? HIGH : LOW));
}



/*      SETUP AND INITIALIZATION FUNCTIONS
 */



void setup_Interfaces() {  // DONE
// ------------------------------------------------------------ 
//   Start Serial1, Query data rate, set Ns, and start Hamamatsu
//

  Serial1.setTX(pin_Ser1_TX);
  Serial1.setRX(pin_Ser1_RX);
  Serial1.begin(115200,SERIAL_8N1);
  Serial1.println("\n\n\nRAD_Counter_Teensy Reporting for Service.");

  while (!Serial1) { };

// RTC  -- after Serial, before SD
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  if (timeStatus()!= timeSet) {
    Serial1.println("Unable to sync with the RTC");
  } else {
    Serial1.println("RTC has set the system time");
  }
  // Set FS Timestamp callback
  FsDateTime::callback = dateTime;

// I2C0

// SPI1

// CANBUS: Can0 on 29/30

// SD via SdFs

#if !ENABLE_DEDICATED_SPI
  Serial1.println(F(
    "\nFor best performance edit SdFsConfig.h\n"
    "and set ENABLE_DEDICATED_SPI nonzero")); 
#endif  // !ENABLE_DEDICATED_SPI
  // Initialize SD.
  if (!sd.begin(SD_CONFIG)) {
      Serial1.println("SD Initialization failed!!");
//      return(-1);
  }

}

void setup_Buffers() { // DONE
  for(int i=0;i<12;++i) {
    Payload[i] = 0xFF;
  }
  for(int i=0;i<18;++i) {
    UTC_Buffer[i] = 0xFF;
  }

//  Serial1.println(" ");
//  Serial1.print("Ring Dimension = ");
//  Serial1.println(size_Ring);
//  Serial1.print("FreeStack: ");
//  Serial1.println(FreeStack());

}

void setup_Timers() { 
    // Initialize DWT CPU-cycle counter, for  use  in timing pings:
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

}



/*      CLI functions
 */

int  Initial_CLI() {
  uint32_t m;
  char c,ns;

  do { delay(20); } while (Serial1.available() && Serial1.read());
  Serial1.println(" ");
  Serial1.println("Type '1' to log data");
  Serial1.println("     '2' to enter  setup");

  m = micros();
  while (!Serial1.available() && (micros()-m < 30*ONE_MILLION)) {  }
  if(!Serial1.available()) { // Unattended, start logging data at default rate: 
    Current_Ns = 0;
    Set_Ns();
    return(LOG_DATA);
  } 
  // if  we get here, there's  serial data:
  c = Serial1.read();
  
  if (c =='1') {

    do { delay(20); } while (Serial1.available() && Serial1.read());
    Serial1.println(" ");
    Serial1.println("Select Data Sampling Rate:");
    Serial1.println("Type '0' for  1 kHz");
    Serial1.println("Type '1' for  2 kHz");
    Serial1.println("Type '2' for  4 kHz");
    Serial1.println("Type '3' for  8 kHz");
    Serial1.println("Type '4' for 10 kHz");
    Serial1.println("Type '5' for 16 kHz");
//    Serial1.println("Type '6' for 24 kHz"); // 240MHz
//    Serial1.println("Type '7' for 32 kHz"); // 240MHz
    Serial1.println("Type '6' for 25 kHz"); // 250MHz
    Serial1.println("Type '7' for 40 kHz"); // 250MHz

    m =  micros();
    while (!Serial1.available() && (micros()-m < 30*ONE_MILLION)) {  }
    if(!Serial1.available()) { ns = '0';} // No reply, use default rate
    else {ns = Serial1.read();} // Use specified rate
    
    switch(ns)  {
      case '1': Current_Ns = 1; break;
      case '2': Current_Ns = 2; break;
      case '3': Current_Ns = 3; break;
      case '4': Current_Ns = 4; break;
      case '5': Current_Ns = 5; break;
      case '6': Current_Ns = 6; break;
      case '7': Current_Ns = 7; break;
      default:  Current_Ns = 0; break;
    } // Switch(ns)

    Serial1.println(" ");
    Serial1.print("Proceeding with Sampling Rate ");
    Serial1.print(Ns[Current_Ns]);  
    Serial1.println("Hz");

    Set_Ns();
    return(LOG_DATA);    
  }

  // if we get here, go to CLI
  Current_Ns = 0;
  Set_Ns();
  return(RUN_CLI);
}

void Main_CLI() {
  /* Things the user might want to do:
   */  
  /*  USB Pull Data
   *  
   */
  /*  Format SD
   *  
   */
  /*  Change Settings
   *  
   *  
   * 
   */
  
}



/*      Count Cycle Functions
 */

int  Open_Files() { // DONE
/* ------------------------------------------------------------ 
//  Build filestructure: 
//        i. Preallocate SD files: 
//           1 hour at max 32kHz data rate = 691 MB
//           ==> Filesize = 1GB
//           1 day at 10kHz nominal = 5.2GB
//           ==> Preallocate array of 6 files
//        ii. Write global headers  (first 16kB = header)
//
*/
  
  char              Filename_Root[128],
                    Filename_Text[128],
                    Filename_Num[16],
                    Filename[N_Files][128], 
                    FileNameTime[24],
                    HeaderTime[24];
                    
  sprintDateTime(HeaderTime,FileNameTime);


  sd.chdir("/");
  sd.mkdir(FileNameTime);
  sd.chdir(FileNameTime);
  
// Build Filenames
  strcpy(Filename_Root,FILENAME_ROOT);
  strcat(Filename_Root,FileNameTime);
    
  strcpy(Filename_Text,Filename_Root);
  strcat(Filename_Text,".txt");

  for(int i=0;i<N_Files;++i) {
    strcpy(Filename[i],Filename_Root);
    sprintf(Filename_Num,"_f%.2u.bin",i);
    strcat(Filename[i],Filename_Num);
  }


// Open & Preallocate Files

  for(int i=0;i<N_Files;++i) {
    if(!bfile[i].open(Filename[i], O_WRITE | O_CREAT | O_TRUNC)) { 
      errorHalt(ERR_MSG_FILE_OPEN_FAILED); 
     }
    if(!bfile[i].preAllocate(PRE_ALLOCATE_MiBS * ONE_MiB))  
      errorHalt(ERR_MSG_FILE_PREALLOC_FAILED);
  }

// Open and Fill Header File

    if(!mfile.open(Filename_Text, O_RDWR | O_CREAT)) {  errorHalt(ERR_MSG_FILE_OPEN_FAILED); }

    mfile.printf("* Future Ocean Lab Radiometer Data File \r\n");
    mfile.printf("*  \r\n");
    mfile.printf("* Software Version: %f \r\n",FOL_RAD_VV);
    mfile.printf("*  \r\n");
    mfile.printf("* %s, %s \r\n",CRUISE_NAME,SHIP_NAME);
    mfile.printf("*  \r\n");
    mfile.printf("* File Created at %s \r\n",HeaderTime);
    mfile.printf("*  \r\n");
    mfile.printf("* Sampling Rate: 1GHz FPGA subsampled at %uHz\r\n",Ns[Current_Ns]);
    mfile.printf("*  \r\n");
    mfile.printf("* Each Ping generates a 6B binary data packet: \r\n");
    mfile.printf("*    [2B] <Microseconds since last ping>  \r\n");
    mfile.printf("*    [2B] <Pulse Count>  \r\n");
    mfile.printf("*    [2B] <TimeHi mod 16>  \r\n");
    mfile.printf("*  \r\n");
    mfile.printf("* Sensor data is stored asynchronously in  12B packets:  \r\n");
    mfile.printf("*    [2B] 0xFFFF  \r\n");
    mfile.printf("*    [2B] <Tilt1> \r\n");
    mfile.printf("*    [2B] <Tilt2>  \r\n");
    mfile.printf("*    [2B] 0xFFFF   \r\n");
    mfile.printf("*    [2B] <Temp>  \r\n");
    mfile.printf("*    [2B] <Depth> \r\n");
    mfile.printf("*  \r\n");
    mfile.printf("* A 1Hz Heartbeat is stored in a 18B packet as: \r\n");
    mfile.printf("*    [5B] 0xFFFFFFFFFF  \r\n");
    mfile.printf("*    [4B] <UTC seconds> \r\n");
    mfile.printf("*    [4B] <Microseconds>  \r\n");
    mfile.printf("*    [5B] 0xFFFFFFFFFF  \r\n");
    mfile.printf("*  \r\n");
    mfile.printf("*  \r\n");
    mfile.flush();

    Serial1.println(" ");
    Serial1.println("Files opened and preallocated...  ");

    return 0;
}

void Start_Count() { // DONE
  Serial1.println(" ");

  // Startup Hamamatsu
  digitalWrite(pin_HamPwr,HIGH);
  Serial1.println("Hamamatsu Powered Up... ");

  // Start Counting
  digitalWrite(pin_Reset,LOW);
  Serial1.println("Fabric Counters Running... ");
  
  // Start Ping Interrupt
  fHandlePings = TRUE;
  Serial1.println(" ");
  Serial1.println("Photon counting has begun!");
  Serial1.println(" ");

  fStopCount = FALSE;
  
}

void Log_Data() {
  static int Count=0,         // Result of Write_Ring_to_SD()
             eval=0,
             HamWasRdy=1,    // Start high to trigger opening "not ready" msg
             HamIsRdy=1;
  static char cmd;

//  Serial1.print("In Log_Data(), fStopCount = ");
//  Serial1.println(fStopCount);

  while(fStopCount == FALSE) {
    
    // ------------------------------------------------------------ 
    //  1. If there's an RU+ in the cache, log  to SD  
    noInterrupts();
    Count = hTeensyRing->Count ;
    interrupts();
    if(Count > SIZE_RU) {
      if( (eval = Write_Ring_to_SD()) != 0) {
        if(eval == ERR_FILE_WRITE_FAILED) {
          /*  TODO: panic, file write failed*/
          Serial1.println("File Write Failed! Shutting Down!");
          fStopCount = TRUE;
          }
        else if(eval == ERR_OUT_OF_FILES) {
            /* TODO:  OUT OF FILES! Hold Ints, ADD MORE FILES, 
                      EAT THE LOST DATA FOR A BIT, restore Ints*/
          Serial1.println("File Write Failed! Shutting Down!");
          fStopCount = TRUE;
        }  
      }
    } 

    
    // ------------------------------------------------------------ 
    //  2. Query Sensors, Fill Payload                 SKIP for now
    //   

    
    // ------------------------------------------------------------ 
    //  3. Heartbeat      // DONE
    if(fHeartbeat==HIGH) {
      fHeartbeat=LOW;    
      Serial1.print(" Pulses: ");
      Serial1.println((uint32_t)LastSec_Pulses);
      Serial1.print(" TimeHi: ");
      Serial1.println((uint32_t)LastSec_TimeHi);
      
      // Check for change of HamRdy signal once per Heartbeat
      HamIsRdy = digitalRead(pin_HamRdy);
      if (HamIsRdy!=HamWasRdy) {
        if(HamIsRdy == 1) {Serial1.println(" Hamamatsu Ready! ");}
        else              {Serial1.println(" Hamamatsu Warming up... ");};
        HamWasRdy = HamIsRdy;
      } // if (HamIsRdy!=HamWasRdy)
    } // if(fHeartbeat==HIGH)


    // ------------------------------------------------------------ 
    //  4. Minimal Serial CLI 
    if (Serial1.available()) {
      cmd = Serial1.read();
      switch(cmd)  {
        case 'q': 
          Serial1.println(" ");
          Serial1.println(" Stopping count... ");
          Serial1.println(" ");
          fStopCount = TRUE; 
          break;
        default:
          Serial1.println(" ");
          Serial1.println(" Type 'q' to stop count and enter command shell");
          Serial1.println(" ");
          break;
      }
    }


    // ------------------------------------------------------------ 
    //  5. Check PwrDown Pin
    eval = digitalRead(pin_PwrDwn);
//    if(eval==LOW) {
//      fStopCount = TRUE;
//      fPowerDownNow=TRUE;
//    }

    
  } // while(fStopCount == FALSE)
}

void Stop_Count() {

  fStopCount = TRUE;

  // Start Ping Interrupt
  fHandlePings = FALSE;
  Serial1.println(" ");
  Serial1.println("Photon counting has stopped. ");
  Serial1.println(" ");
  
  digitalWrite(pin_Reset,HIGH);
  Serial1.println("Fabric Counters Zeroed... ");

  digitalWrite(pin_HamPwr,LOW);
  Serial1.println("Hamamatsu Powered Down... ");

}

void Close_Files() {
  char              FileNameTime[24],
                    HeaderTime[24];
                    
  sprintDateTime(HeaderTime,FileNameTime);
    
  mfile.printf("* File Closed at %s \r\n",HeaderTime);
  mfile.flush();
  mfile.truncate();
  mfile.close();
    
  for(int j=0;j<N_Files;++j) {
    bfile[j].flush();
    bfile[j].truncate();
    if(bfile[j].fileSize()==0) {bfile[j].remove();};
    bfile[j].close();
  }

  sd.chdir("/");

  Serial1.println(" ");
  Serial1.println(" Files Closed! ");
  
}



/*    Interrupt Routines  
*/

FASTRUN void ISR_Ping(void) { // DONE
/*    ISR_Ping: respond to ping by reading data and storing to RingBuffer
   
x         Record new ping time
x         Write 2B us-since-last-ping into local buffer
x         Set Dtog High
x         Read 16-bit bus into local buffer
x         Set Dtog Low
x         Read 16-bit bus into local buffer
x         Push local buffer into RingBuffer
x         If new Payload is available (sensor data, etc) push to RingBuffer
x         Incriment Per-sec counts
x         Decriment Ping_Count
x         If 1s of Pings have gone by, 
x             add UTC to UTC_buffer:
x             Ping_Count = Ns;
x             Write local buffer into global read buffer

------------------------------------------------------------------------------*/

  static uint8_t   NewData[6];
  static uint16_t* NewData16 = (uint16_t*) NewData;
  static uint32_t  _Pulses=0,_TimeHi=0; // Pulses and Duty over last Sec
  static uint16_t  PingCount   = 0;
  static uint32_t  cycles = 0,
                   cycles_last = 0;

  if(fHandlePings==TRUE) {

/*    0. Suspend Interrupts and check time
*/
    noInterrupts();
    
    cycles = ARM_DWT_CYCCNT; // Use free-running DWT cpu-click-counter on the K77 M4...
    NewData16[0] = (cycles - cycles_last)/cpu_clicks_per_us; // us between pings, up to 65ms
    cycles_last = cycles;
  

/*    1. PULL DATA FROM CMOD
 */
//          Set Dtog High and read 16-bit bus into local buffer
    digitalWriteFast(pin_Dtog,HIGH);
    NewData[2] = GPIOC_PDIR & 0xFF; // ~20ns
    NewData[3] = GPIOD_PDIR & 0xFF; // ~20ns
//          Set Dtog Low and read 16-bit bus into local buffer
    digitalWriteFast(pin_Dtog,LOW);
    NewData[4] = GPIOC_PDIR & 0xFF; // ~20ns
    NewData[5] = GPIOD_PDIR & 0xFF; // ~20ns
//          Push local buffer into RingBuffer
    Write_Data_to_Ring(NewData,6);


/*    2. CATCH AND EXPORT SENSOR PAYLOAD
 */    
    if(fPayload == TRUE) {
      fPayload = FALSE;
      Write_Data_to_Ring((uint8_t *)Payload, PAYLOAD_BYTES);
    } // if(fPayload == TRUE)


/*    3. UPON Ns SAMPLES, WRITE 1s MARKER TO SD AND TRIGGER HEARTBEAT
 */    
//        Increment per-second counters
    _Pulses += NewData[1];
    _TimeHi += NewData[2];
//       Check if it's time for a Heartbeat
    if(--PingCount == 0) {
      PingCount = Ns[Current_Ns];
      UTC_Buffer32[0] = now();
      UTC_Buffer32[1] = mclock;
      Write_Data_to_Ring(UTC_Buffer, UTC_BUFFER_BYTES);
      LastSec_Pulses = _Pulses;
      LastSec_TimeHi = _TimeHi;
      _Pulses = _TimeHi = 0;
      fHeartbeat = HIGH;
    } // if(--PingCount == 0)

    interrupts();
  } // if(fHandlePings==TRUE)
}



/*    System  Shutdown 
*/

void Shutdown() {  // Done-ish
  Serial1.println("Shutting down!!");
  digitalWrite(pin_KillMePls,HIGH);  // Tell Cmod to kill me!!
  while(1) {};
} // setup_Shutdown()

void setup() { // DONE 
  setup_GPIO();
  setup_Interfaces();
  setup_Buffers();
  setup_Timers();
  Open_Files();
  Close_Files();
} // Setup()

void loop() { // DONE
  int code = Initial_CLI(); 
  if(code == LOG_DATA) {
    delay(1000);     
    if(fPowerDownNow==TRUE) Shutdown();
  }
  else if(code  == RUN_CLI) {
    Main_CLI();
  }
}
