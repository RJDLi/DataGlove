/*
 * MCU: ESP32
 * Function:
 * - Reading Quaternion data from slaves with SPI
 * - *Saving data to SD card
 * - *need some buttons to set start/stop
 * - Communicate through Socket to Host
 *      - Sending data
 *      - Act
 *          - Start/Stop
 *          - IMU settings
 */

#include <WiFi.h>

const char* ssid     = "BIGAI-OFFICE01";//"Pixel_7407";//"BIGAI-OFFICE03";
const char* password = "BIGAI_2020";//"3.1415926535";//"BIGAI-2021";

const char* server_addr = "192.168.8.115";
const int server_port = 10000;
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
#define PIN_SS_0 1
#define PIN_SS_1 1
#define PIN_SS_2 1
#define PIN_SS_3 1
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
    1, 2, 9, 15
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
        for(uint8_t i = 0; i<DATA_BYTES; i++){
            #ifdef QUATERNION_EULER
            orientation[i] = (((int16_t)buffer[i*2]) | (((int16_t)buffer[i*2+1]) << 8))/16.0;// 900 for rad
            #else
            orientation[i] = (int16_t)((((uint16_t)buffer[2*i+1]) << 8) | ((uint16_t)buffer[2*i]))/((double)(1<<12));
            #endif
        }
        data_updated = true;
        data_writting = false;
    }
    bool data_updated = false;
    void getOri(uint8_t* buffer){
        for(int i = 0; i< DATA_BYTES; i++){
            float_byte_convert.float_variable = orientation[i];            
            memcpy(buffer+i*4, float_byte_convert.temp_array, 4);
        }
    }
private:
    uint8_t idx;
    uint8_t channel;
    float orientation[DATA_BYTES];
    unsigned long timestamp;
};
Knuckle *knuckleList [16] = {};

bool getQuat(uint8_t idx, uint8_t* buffer, unsigned long timeout = 8/*ms*/){
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

void task_getdata(void *pvParameters){
    (void) pvParameters;
    while(true){
        if(gSet_working){
            uint8_t readbuffer[DATA_BYTES*2*4];
            uint8_t wait2read[4] = {0,1,2,3};
            uint8_t wait2read_num = 4;
            while(wait2read_num > 0){
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
            }
        }
    }
}

byte wlsend_buffer[512];
WiFiClient wificlient;
/*
 * wireless sending buffer
 * Timestamp: 4byte
 * Data: 16*(3/4)*4 = 192/256
 */
void task_senddata(void *pvParameters){
    (void) pvParameters;
    while (true)
    {
        if(gSet_working){
            bool prepared = true;
            for(int i =0;i<16;i++){
                if(!knuckleList[i]->data_updated){
                    prepared = false;
                    break;
                }
            }
            if(prepared){
                // format msg
                data_reading = true;
                //wlsend_buffer[0] = 0xAA;
                uint8_t idx = 0;
                unsigned long timestamp = millis();
                for(int i = 0; i<4; i++){
                    wlsend_buffer[idx] = (uint8_t)(timestamp >> 8);
                    idx ++;
                }
                for(int i = 0; i<16; i++){
                    knuckleList[i]->getOri(wlsend_buffer+idx);
                    idx += (DATA_BYTES*4);
                }
                data_reading = false;
                //wlsend_buffer[idx] = 0xFF;
                //idx ++;
                // send msg
                wificlient.write(wlsend_buffer, idx);
            }
        }
    }    
};

void setup(){
    // SPI
    for(int i=0; i<4; i++){
      pinMode(PIN_SS[i], OUTPUT);
      digitalWrite(PIN_SS[i], HIGH);
    }
    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    //SPI.setClockDivider(SPI_CLOCK_DIV8); // same as 2000000 if 16Mhz osc
    for(int i = 0;i<16; i++){
        knuckleList[i] = new Knuckle(i);
    }
    // start read task
    xTaskCreatePinnedToCore(
    task_getdata
    ,  "Get Data"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);    

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

    // Sending task
    xTaskCreatePinnedToCore(
    task_senddata
    ,  "Send Data"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);    
}
void loop(){
    while(!wificlient.connected()){
        do{
            Serial.println("Unconnected...");
            delay(1000);
        }while(!wificlient.connect(host, port));
        Serial.println("Connected");
    }

    // Handling command
    while(wificlient.available()) {
        String line = wificlient.readStringUntil('\r');
        Serial.print(line);
        // TODO

    }
}
