/*
* imu.h
* Read Quaternion Data From IMU
* Mark the index
*/
#include "Adafruit_BNO055.h"
test not be complied
#define BUFFER_LENGTH 32

#define QUATERNION_QUATERNION
#ifdef QUATERNION_EULER
#define DATA_BYTES 6
#else
#define DATA_BYTES 8
#endif

struct RealtimeData
{
    unsigned long sample_time;
    byte data[DATA_BYTES];
    bool been_read;
};


class IMU{
public:
    IMU(){}
    ~IMU(){}
    /*
    * addr(of BNO): 0x28 or 0x29
    * port(of I2C): 1 or 2
    * index(of ): 0-15
    */
    IMU(byte addr, TwoWire *wire, byte index){
        _addr = addr;
        _wire = wire;
        _index = index;
        _bno = Adafruit_BNO055(-1, _addr, _wire);
    }
    bool begin(Adafruit_BNO055::adafruit_bno055_opmode_t mode = Adafruit_BNO055::OPERATION_MODE_NDOF){    //     OPERATION_MODE_IMUPLUS
        /* Enable I2C */
        _wire->begin();

        /* Make sure we have the right device */
        /*uint8_t id = _bno.read8(BNO055_CHIP_ID_ADDR);
        if (id != BNO055_ID) {
            delay(1000); // hold on for boot
            id = read8(BNO055_CHIP_ID_ADDR);
            if (id != BNO055_ID) {
            return false; // still not? ok bail
            }
        }*/

        /* Switch to config mode (just in case since this is the default) */
        _bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
        /* Reset */
        _bno.write8(Adafruit_BNO055::BNO055_SYS_TRIGGER_ADDR, 0x20);
        /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
        delay(30);
        while (_bno.read8(Adafruit_BNO055::BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            delay(10);
        }
        delay(50);
        /* Set to normal power mode */
        _bno.write8(Adafruit_BNO055::BNO055_PWR_MODE_ADDR, Adafruit_BNO055::POWER_MODE_NORMAL);
        delay(10);
        /* Try to avoid sleep */
        _bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1);
        _bno.write8(Adafruit_BNO055::BNO055_GYRO_DATA_Y_LSB_ADDR, 0xFF); //0x16        
        _bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 0);
        _bno.write8(Adafruit_BNO055::BNO055_SYS_TRIGGER_ADDR, 0x0);
        delay(10);
        /* Set the requested operating mode (see section 3.3) */
        _bno.setMode(mode);
        delay(20);

        return true;
    }
    /*check if bno is sleeping*/
    bool sleeped(){
        byte pwrmode = _bno.read8(Adafruit_BNO055::BNO055_PWR_MODE_ADDR);
        return (pwrmode != 0);
    }

    bool getQuat(double* data){
        imu::Quaternion quat = _bno.getQuat();
        data[0] = quat.w();
        data[1] = quat.x();
        data[2] = quat.y();
        data[3] = quat.z();
        return true;
    };
    bool getQuat(byte* data){
        _bno.readLen(Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR, data, 8);
        return true;
        // reread?
        //_bno.rereadLen(data, 8);
    };
    bool getEuler(double* data){
        imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        data[0] = euler.x();
        data[1] = euler.y();
        data[2] = euler.z();
        return true;
    };
    bool getEuler(byte* data){
        _bno.readLen(Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR, data, 6);
        // reread?
        //_bno.rereadLen(data, 6);
        return true;
    };
    byte index(){
        return _index;
    };

private:
    Adafruit_BNO055 _bno;
    byte _index;
    byte _addr;
    TwoWire *_wire;
};
