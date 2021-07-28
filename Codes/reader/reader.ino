/*
  MCU: Atemage328PB
  Function:
  - Read 4 IMUs data    -- 2x I2C
  - Transfer to Master  -- SPI

  TODO:
  delete thread locks
*/
#define SERIAL_DEBUG
#include<Wire1.h>
#include "imu.h"
#include <SPI.h>

#define IMU_ADDRESS_A 0x4A
#define IMU_ADDRESS_B 0x4B


#include "protothreads.h"
pt pt_reader0;
pt pt_reader1;

// 0/1/2/3
// !!! - Change This When Loading to different Chips - !!!
#define MUX_INDEX 0
#if MUX_INDEX == 0
IMU imu1(IMU_ADDRESS_A, Wire, 12);
IMU imu2(IMU_ADDRESS_B, Wire, 9);
IMU imu3(IMU_ADDRESS_A, Wire1, 13);
IMU imu4(IMU_ADDRESS_B, Wire1, 14);
#elif MUX_INDEX == 1
IMU imu1(IMU_ADDRESS_A, Wire, 7);
IMU imu2(IMU_ADDRESS_B, Wire, 8);
IMU imu3(IMU_ADDRESS_A, Wire1, 10);
IMU imu4(IMU_ADDRESS_B, Wire1, 11);
#elif MUX_INDEX == 2
IMU imu1(IMU_ADDRESS_A, Wire, 4);
IMU imu2(IMU_ADDRESS_B, Wire, 5);
IMU imu3(IMU_ADDRESS_A, Wire1, 6);
IMU imu4(IMU_ADDRESS_B, Wire1, 3);
#elif MUX_INDEX == 3
IMU imu1(IMU_ADDRESS_A, Wire, 1);
IMU imu2(IMU_ADDRESS_B, Wire, 2);
IMU imu3(IMU_ADDRESS_A, Wire1, 0);
IMU imu4(IMU_ADDRESS_B, Wire1, 15);
#endif

IMU *imuList[4] = {&imu1, &imu2, &imu3, &imu4};

//bool data_reading = false; // lock data access when writting/reading
//bool data_writting = false;
RealtimeData rtData[4];

byte response_msg[DATA_BYTES * 4]; //TODO if need time info?
bool dataReady() {
  bool ret = true;
  for (int i = 0; i < 4; i++) {
    if (rtData[i].been_read) {
      ret = false;
      break;
    }
  }
  return ret;
}
void dataBeenRead() {
  for (int i = 0; i < 4; i++)
    rtData[i].been_read = true;
}

void setRtData(int idx, byte * data) {
  //data_writting = true;
  for (int i = 0; i < DATA_BYTES; i++) {
    response_msg[idx * DATA_BYTES + i] = data[i];
  }
  rtData[idx].been_read = false;
  //data_writting = false;
}

int wire0reader(struct pt* pt) {
  PT_BEGIN(pt);
  byte readbuffer[DATA_BYTES];
  byte read_index;
  while (true) {
    if (rtData[0].sample_time > rtData[1].sample_time) read_index = 1;
    else read_index = 0;
    while(!imuList[read_index]->process()){
      PT_YIELD(pt);
    }
    if(imuList[read_index]->data_ava()){
      imuList[read_index]->getQuat(readbuffer);
      setRtData(read_index, readbuffer);
    }
  }
  PT_END(pt);
}
int wire1reader(struct pt* pt) {
  PT_BEGIN(pt);
  byte readbuffer[DATA_BYTES];
  byte read_index;
  while (true) {
    if (rtData[2].sample_time > rtData[3].sample_time) read_index = 3;
    else read_index = 2;
    while(!imuList[read_index]->process()){
      PT_YIELD(pt);
    }
    if(imuList[read_index]->data_ava()){
      imuList[read_index]->getQuat(readbuffer);
      while(data_reading){
        PT_YIELD(pt); // respond to SPI call first
      }
      setRtData(read_index, readbuffer);
    }
  }
  PT_END(pt);
}
/*
void task_wire1_read() {
  byte readbuffer[DATA_BYTES];
  while (true) {
    if (rtData[2].sample_time > rtData[3].sample_time) {
      // read 3
      if (imuList[3]->getQuat(readbuffer)) {
        setRtData(3, readbuffer);
      }
    }
    else {
      // read 2
      if (imuList[2]->getQuat(readbuffer)) {
        setRtData(2, readbuffer);
      }
    }
  }
}

  wire0_reader->onRun(task_wire0_read);
  wire0_reader->enabled = true;
  wire0_reader->setInterval(10);
  wire0_reader->run();
  wire1_reader->onRun(task_wire1_read);
  wire1_reader->enabled = true;
  wire1_reader->setInterval(10);
  wire1_reader->run();
*/

volatile bool newcommand = false;;
volatile byte command;
volatile byte data_sending_index;
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  command = SPDR;
  //if(newcommand) error
  newcommand = true;
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.printf(F("-- BNO Reader Atmega328PB --\n"));
#endif
  // I2C
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);
  for (int i = 0; i < 4; i++) {
    imuList[i]->begin();
  }
  // start readers

  // SPI
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  SPDR = 0xFF; // no data
  SPCR =  0xC1;//11000001b
  SPSR |= 1; //1b

  // Reading task  
  PT_INIT(&pt_reader0);
  PT_INIT(&pt_reader1);
}

/*
   Master:
   0xAF - Start
   0xBx - require the x^st byte
   Slave:
   0xFF - Not Ready/ Error
   0xFA - Ready to start
   data
*/

byte status = 0x00;
void loop() {
  if(newcommand){
#ifdef SERIAL_DEBUG
  Serial.printf("SPI recv: %.2x\n", command);
#endif
  }
  else{
    // keep reading.  
    PT_SCHEDULE(wire0reader(&pt_reader0));
    PT_SCHEDULE(wire1reader(&pt_reader1));
  }
  // respond to Master Command.
  switch (status)
  {
    case 0x00: //waiting for reading data
      if (dataReady()) {
        SPDR = 0xFA;
        status = 0x01;
#ifdef SERIAL_DEBUG
  Serial.printf("Data Ready\n");
#endif
      }
      else SPDR = 0xFF;
      break;
    case 0x01: // data ready
      if (newcommand) {
        if (command == 0xAF) { // Start
          //while (data_writting) {}
          data_reading = true;
          SPDR = response_msg[0];
          data_sending_index = 1;
          status = 0x02;
#ifdef SERIAL_DEBUG
  Serial.printf(F("Sending Data...\n"));
#endif
          break;
        }
        else {
          //Error
          SPDR = 0xFF;
          status = 0x00;
        }
      }
      break;
    case 0x02: // communication
      if (newcommand) {
        //TODO do we need set timeout?
        if (command - 0xB0 == data_sending_index) {
          SPDR = response_msg[data_sending_index];
          data_sending_index ++;
          if (data_sending_index == DATA_BYTES * 4) {
            // communication complete!
            dataBeenRead();
            data_reading = false;
            status = 0x00;
          }
        }
        //else if(command < 0xA0){
        else {
          // Error
          SPDR = 0xFF;
          dataBeenRead();
          data_reading = false;
          status = 0x00;
#ifdef SERIAL_DEBUG
  Serial.printf(F("Data Sending complete\n"));
#endif
        }
      }
      break;

    default:
      break;
  }
  newcommand = false;
}

/* ----------- SPI Register-------------
   SPCR - SPI Control Register ---------
    7     6     5     4     3     2     1     0
   SPIE  SPE  DORD  MSTR  CPOL  CPHA  SPR1  SPR0
   SPIE - interrupt Enable
   SPE  - SPI Enable
   DORD - Data Order
   MSTR - Master/Slave Select
   CPOL - Clock Polarity
   CPHA - Clock Phase
   SPR  - Clcok divide
          4 / (SPI2X << 1) * <SPI1:SPI0>
   SPSR - SPI Status Register ----------
    7     6     5     4     3     2     1     0
   SPIF  WCOL   -     -     -     -     -    SPI2X
   SPIF - SPI Interrupt Flag
   WCOL - Write Collision Flag
   SPDR - SPI Data Register -------------
   MSB - LSB
*/
