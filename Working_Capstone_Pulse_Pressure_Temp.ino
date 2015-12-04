
#include <Wire.h>
#include <Adafruit_BMP085.h>
//  Variables
int ANALOG_IN_PIN = 4;                 // Pulse Sensor purple wire connected to analog pin 0
int DIGITAL_OUT_PIN = 0;
// Volatile Variables, used in the interrupt service routine!
int BPM;                   // int that holds raw Analog in POS. updated every 2mS
int Signal;                // holds the incoming raw data
int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
boolean QS = false;        // becomes true when Arduoino finds a beat.


int rate[10];                    // array to hold last ten IBI values
unsigned long sampleCounter = 0;          // used to determine pulse timing
unsigned long lastBeatTime = 0;           // used to find IBI
int P =512;                      // used to find peak in pulse wave, seeded
int T = 512;                     // used to find trough in pulse wave, seeded
int thresh = 525;                // used to find instant moment of heart beat, seeded
int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

Ticker ticker3;
Adafruit_BMP085 bmp;


//Variables for BMP 180
uint32_t bar; 
uint16_t hot;
  
#define TXRX_BUF_LEN                      20

#define DIGITAL_OUT_PIN                   A3
#define ANALOG_IN_PIN                     A4
#define ANALOG_IN_PIN1                    A5



BLE                                       ble;
Ticker                                    ticker;

static boolean analog_enabled = false;

// The Nordic UART Service
static const uint8_t service1_uuid[]                = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]             = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]             = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]           = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};

GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

void disconnectionCallBack(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
  Serial1.println("Disconnected!");
  Serial1.println("Restarting the advertising process");
  ble.startAdvertising();
}

void writtenHandle(const GattWriteCallbackParams *Handler)
{
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t bytesRead, index;

  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    Serial1.print("bytesRead: ");
    Serial1.println(bytesRead, HEX);
    for (byte index = 0; index < bytesRead; index++) {
      Serial1.write(buf[index]);
    }
    Serial1.println("");
//    //Process the data
//    if (buf[0] == 0x01)  // Command is to control digital out pin
//    {
//      if (buf[1] == 0x01)
//        digitalWrite(DIGITAL_OUT_PIN, HIGH);
//      else
//        digitalWrite(DIGITAL_OUT_PIN, LOW);
//    }
//    else if (buf[0] == 0xA0) // Command is to enable analog in reading
//    {
//      if (buf[1] == 0x01)
//        analog_enabled = true;
//      else
//        analog_enabled = false;
//    }
//    else if (buf[0] == 0x04)
//    {
//      analog_enabled = false;
//      digitalWrite(DIGITAL_OUT_PIN, LOW);
//    }

  }
}

void m_status_check_handle()
{ 
     uint8_t buf[5];
      bar = (uint32_t)(bmp.readPressure());
      buf[0] = 0x0A;
      buf[1] = (bar>>24);
      buf[2] = (bar>>16);
      buf[3] = (bar>>8);
      buf[4] = (bar);
      ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 5);

   uint8_t tmp[5];
      hot = (uint32_t)(bmp.readTemperature());
      tmp[0] = 0x0B;
      tmp[1] = (hot>>8);
      tmp[2] = (hot);
      ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), tmp, 3);
  
}

// THIS IS THE Ticker INTERRUPT SERVICE ROUTINE.
// It makes sure that we take a reading every 2 miliseconds
void heartbeatcheck(){                         // triggered when Timer2 counts to 124
  Signal = analogRead(ANALOG_IN_PIN);              // read the Pulse Sensor
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                        // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave
    }
  }

  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        return;                              // IBI value is unreliable so discard it
      }


      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
                                // enable interrupts when youre done!
}// end ticker

void setup() {
  // put your setup code here, to run once
  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  digitalWrite(DIGITAL_OUT_PIN, LOW);
  Serial1.begin(9600);
  bmp.begin();
  
  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(writtenHandle);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"TXRX", sizeof("TXRX") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));

  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)"Multi Input");
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();

  ticker.attach_us(m_status_check_handle, 200000);

  Serial1.println("Advertising Start!");

  ticker3.attach_us(heartbeatcheck, 2000);// sets up to read Pulse Sensor signal every 2mS

}
void loop(){
  if (QS == true){     // A Heartbeat Was Found
                       // BPM and IBI have been Determined
                       // Quantified Self "QS" true when arduino finds a heartbeat
    if (BPM > 40 && BPM < 175){ 
    digitalWrite (DIGITAL_OUT_PIN, HIGH);
    delay (200);
    digitalWrite(DIGITAL_OUT_PIN, LOW);
    }       
uint8_t buf[2];
    buf[0] = (0x0C); // set flag for incoming data
    //buf[1] = (0x00); //only 1 8 bit number going out, so first buffer is 0
    buf[1] = ((uint8_t) BPM); // cast BPM as unsigned 8 bit int
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 2);

uint8_t buff[3];

  uint16_t value = analogRead(ANALOG_IN_PIN);
  buff[0] = (0x0B); // why is this B in HEX
  buff[1] = (value >> 8);
  buff[2] = (value);
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buff, 3);

uint8_t test [3];

  uint16_t value1 = analogRead(ANALOG_IN_PIN1);
  test[0] = (0x0C); // why is this B in HEX
  test[1] = (value1 >> 8);
  test[2] = (value1);
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), test, 3);


//Add ble.update
        QS = false;                      // reset the Quantified Self flag for next time





    
  }
}
