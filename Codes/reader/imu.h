/*
  imu.h
  Read Quaternion Data From IMU
  Mark the index
*/
#include "BNO080_multiI2C.h"

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

enum RotaionVectorType {
  DOF9,
  ARVR_DOF9,
  Game,
  ARVR_Game,
  GyroInt
};


class IMU {
  public:
    IMU() {}
    ~IMU() {}
    /*
      addr(of BNO): 0x4A or 0x4B(Default)
      port(of I2C): 1 or 2
      index(of Knuckle): 0-15
    */
    IMU(byte addr, TwoWire &wire, byte index) {
      _addr = addr;
      _wire = &wire;
      _index = index;
      _bno = new BNO080();
    }
    bool begin(RotaionVectorType rvt = DOF9, unsigned int internal_ms = 10) {
      if (_bno->begin(_addr, *_wire, 255)) {
        switch (rvt)
        {
          case DOF9:
            _bno->enableRotationVector(internal_ms);
            break;
          case ARVR_DOF9:
            _bno->enableARVRStabilizedRotationVector(internal_ms);
            break;
          case Game:
            _bno->enableGameRotationVector(internal_ms);
            break;
          case ARVR_Game:
            _bno->enableARVRStabilizedGameRotationVector(internal_ms);
            break;
          case GyroInt:
            _bno->enableGyroIntegratedRotationVector(internal_ms);
            break;
          default:
            break;
        }
        return true;
      }
      return false;
    }

    bool getQuat(float* data) {
      while (!_bno->dataAvailable()) {
        // TODO: timeout
      }
      _bno->getQuat(data[1], data[2], data[3], data[0], radAccuracy, accuracy);
      return true;
    };
    bool wait_getQuat(byte* data) {
      while (!_bno->dataAvailable()) {
        // TODO: timeout
      }
      _bno->getQuat(data);
      return true;
    };
    bool process(){
      return _bno->cus_askdata();
    }
    bool data_ava(){
      return _bno->cus_dataavailable();
    }
    /*div 2^14*/
    void getQuat(byte* data){
      _bno->getQuat(data);
    }
    bool getEuler(float* data) {
      while (!_bno->dataAvailable()) {
        // TODO: timeout
      }
      data[0] = _bno->getRoll();
      data[1] = _bno->getPitch();
      data[2] = _bno->getYaw();
      return true;
    };
    byte index() {
      return _index;
    };

private:
  BNO080 *_bno;
  byte _index;
  byte _addr;
  TwoWire *_wire;

  float radAccuracy;
  uint8_t accuracy;
};
