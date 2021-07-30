/*
 * MCU: ESP32
 * Function:
 * - Reading Quaternion data from slaves with SPI
 * - *Saving data to SD card
 * - *need some buttons to set start/stop
 * - Communicate through Socket to Host
 *      - Sending data
 *      - *Act
 *          - Start/Stop
 *          - IMU settings
 * Note:
 * - Disable Serial print to speed up
 * - Comment Test code -- search for "random data"
 *   This part -- communication with slave mcu, is not tested
 */

#include <WiFi.h>

const char* ssid     = "Pixel_7407";//"BIGAI-OFFICE03";"BIGAI-OFFICE01";//
const char* password = "3.1415926535";//"BIGAI-2021";"BIGAI_2020";//

const char* server_addr = "192.168.166.9";
const int server_port = 9090;
/*
 * Left:
 * Index
 * 14   11  8   5
 * 13   10  7   4
 * 12   9   6   3
 *        15        0   1   2
 * I2C port
 * 1B   3B  2B  4B
 * 1A   3A  2A  4A
 * 0A   0B  5A  5B
 *        7B        7A  6A  6B  
 * Right:
 * Index
 *         5   8   11  14   
 *         4   7   10  13   
 *         3   6   9   12   
 * 2  1  0       15        
 * I2C port
 *         4B  2B  3B  1B
 *         4A  2A  3A  1A
 *         5B  5A  0B  0A
 * 6B 6A 7A       7B       
 */
#include<SPI.h>

#define HAND_LEFT
#define PIN_SS_0 12
#define PIN_SS_1 13
#define PIN_SS_2 14
#define PIN_SS_3 15
// check pin settings in
// ~/.arduino15/packages/esp32/hardware/esp32/1.0.6/variants/pico32
const int PIN_SS[4] = {PIN_SS_0, PIN_SS_1, PIN_SS_2, PIN_SS_3};

#define QUATERNION_QUATERNION
#ifdef QUATERNION_EULER
    #define DATA_BYTES 3
#else
    #define DATA_BYTES 4
#endif

bool gSet_working = true;

bool data_reading = false;
bool data_writting = false;
const uint8_t idx2i2c[16] = {
    0x7A, 0x6A, 0x6B,
    0x5B, 0x4A, 0x4B,
    0x5A, 0x2A, 0x2B,
    0x0B, 0x3A, 0x3B,
    0x0A, 0x1A, 0x1B,
    0x7B
};
const uint8_t i2c2idx[16] = {
    12, 9, 13, 14,
    7, 8, 10, 11,
    4, 5, 6, 3,
    1, 2, 0, 15
};
union {
    float float_variable;
    byte temp_array[4];
  } float_byte_convert;
class Knuckle{
public:
    Knuckle(){};
    ~Knuckle(){};
    Knuckle(uint8_t _idx):idx(_idx){
        channel = idx2i2c[idx]; // not used
    };
    void setOri(uint8_t* buffer){
        while (data_reading){}        
        data_writting = true;
        timestamp = millis();
        
        /*
        for(uint8_t i = 0; i<DATA_BYTES; i++){
            #ifdef QUATERNION_EULER
            orientation[i] = (((int16_t)buffer[i*2]) | (((int16_t)buffer[i*2+1]) << 8))/16.0;// 900 for rad
            #else
            orientation[i] = (int16_t)((((uint16_t)buffer[2*i+1]) << 8) | ((uint16_t)buffer[2*i]))/((double)(1<<12));
            #endif
        }*/
        for(uint8_t i = 0; i<DATA_BYTES*2; i++){
          databytes[i] = buffer[i];
        }
        
        data_updated = true;
        data_writting = false;
    }
    bool data_updated = false;
    void getOri(uint8_t* buffer){
        /*
        for(int i = 0; i< DATA_BYTES; i++){
            float_byte_convert.float_variable = orientation[i];            
            memcpy(buffer+i*4, float_byte_convert.temp_array, 4);
        }
        */
        for(uint8_t i = 0; i<DATA_BYTES*2; i++){
          buffer[i] = databytes[i];
        }
        
        data_updated = false;
    }
private:
    uint8_t idx;
    uint8_t channel;
    //float orientation[DATA_BYTES];
    uint8_t databytes[DATA_BYTES*2];
    unsigned long timestamp;
};
Knuckle *knuckleList [16] = {};

bool getQuat(uint8_t idx, uint8_t* buffer, unsigned long timeout = 8/*ms*/){
    // random data for test
    for(int i = 0; i<DATA_BYTES*2*4; i++) buffer[i] = 0x01;
    return true;
    
    uint8_t readbyte, sendbyte = 0xBF;
    digitalWrite(PIN_SS[idx],LOW);

    readbyte = SPI.transfer(sendbyte);
    if(readbyte != 0xFA){
        return false;
    }
    for(uint8_t i = 0; i<8*DATA_BYTES; i++){
        sendbyte ++;
        readbyte = SPI.transfer(sendbyte);
        //if(readbyte == 0xFF) // one 0xFF is possible but two. // TODO
        buffer[i] = readbyte;
    }

    digitalWrite(PIN_SS[idx],HIGH);
    return true;
}

uint8_t wait2read[4] = {0,1,2,3};
uint8_t wait2read_num = 4;
bool prepared = false;
void task_getdata(){//void *pvParameters){
    //(void) pvParameters;
    //Serial.println("Hello from task get");
    //while(true){
        if(gSet_working){
            uint8_t readbuffer[DATA_BYTES*2*4];
            if(wait2read_num > 0){
                uint8_t i = wait2read[0];
                if(getQuat(i, readbuffer)){
                    for(uint8_t j = 0; j<4; j++){
                        uint8_t idx = i2c2idx[i*4+j];
                        knuckleList[idx]->setOri(readbuffer+j*DATA_BYTES*2);
                    }
                    wait2read_num --;
                    for(uint8_t k=0; k<wait2read_num; k++){
                        wait2read[k] = wait2read[k+1];
                    }
                }
                else{
                    // Fail to get data
                    // report?
                    for(uint8_t k=0; k<wait2read_num; k++){
                        wait2read[k] = wait2read[k+1];
                    }
                    wait2read[wait2read_num] = i;
                }
            }else{      
              prepared = true;
              //Serial.println("Read 1 Frame");
              for(uint8_t k=0; k<4; k++){
                  wait2read[k] = k;
              }  
              wait2read_num = 4;
            }
        }
    //}    
    //vTaskDelete(NULL); // use this when this function is a task
    return;
}

byte wlsend_buffer[512];
WiFiClient wificlient;
/*
 * wireless sending buffer
 * Timestamp: 4byte
 * Data: 16*(3/4)*2 = 96/128 (Euler/Quaternion)
 * MsgLength = 100/132
 */
void task_senddata(){//void *pvParameters){
    //(void) pvParameters;
    //Serial.println("Hello from task send");
    //while (true)    {
    if(wificlient.connected()){
        if(gSet_working){
            if(prepared){
                //Serial.println("Send");
                // format msg
                data_reading = true;
                //wlsend_buffer[0] = 0xAA;
                uint8_t idx = 0;
                unsigned long timestamp = millis();
                for(int i = 0; i<4; i++){
                    wlsend_buffer[idx] = (uint8_t)(timestamp >> (8*i));
                    idx ++;
                }
                for(int i = 0; i<16; i++){
                    knuckleList[i]->getOri(wlsend_buffer+idx);
                    idx += (DATA_BYTES*2);
                }
                prepared = false;
                data_reading = false;
                //wlsend_buffer[idx] = 0xFF; idx ++;
                // send msg
                wificlient.write(wlsend_buffer, idx);
                //debug
                /*for(int i=0; i<idx; i++){
                  Serial.print(wlsend_buffer[i]);
                  Serial.print(" ");
                }
                Serial.println("");*/
            }
        }
    }    
    //vTaskDelete(NULL); // use this when this function is a task
    return;
};

void setup(){
    Serial.begin(115200);
    Serial.println("!!!Begin");
    // SPI
    for(int i=0; i<4; i++){
      pinMode(PIN_SS[i], OUTPUT);
      digitalWrite(PIN_SS[i], HIGH);
    }
    SPI.begin();    
    SPISettings setting1;
    setting1._bitOrder = MSBFIRST;
    setting1._clock = 2000000;
    setting1._dataMode = SPI_MODE0;
    SPI.beginTransaction(setting1);
    //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();
    //SPI.setClockDivider(SPI_CLOCK_DIV8); // same as 2000000 if 16Mhz osc

    // WiFi Client
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    
    for(int i = 0;i<16; i++){
        knuckleList[i] = new Knuckle(i);
    }
    
}
void loop(){
  /*
    // start read task
    xTaskCreatePinnedToCore(
    task_getdata
    ,  "Get Data"   // A name just for humans
    ,  1024         // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2            // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);          // Core

    // Sending task
    xTaskCreatePinnedToCore(
    task_senddata
    ,  "Send Data"    // A name just for humans
    ,  1024           // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3              // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);            // Core
    */
    task_getdata();
    task_senddata();
    
    if(!wificlient.connected()){
        do{
            Serial.println("Unconnected...");
            delay(1000);
        }while(false);
        if(wificlient.connect(server_addr, server_port)){          
          Serial.println("Connected");
        }
    }else{      
        // Handling command
        while(wificlient.available()) {
            String line = wificlient.readStringUntil('\r');
            Serial.print(line);
            // TODO
            // gSet_working
    
        }
    }
    

}
