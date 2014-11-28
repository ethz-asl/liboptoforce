#include "StateMachine.hpp"
#include <boost/concept_check.hpp>

using namespace std;

namespace optoforce {
  
/*****************************************************************************/
/* Constructors                                                              */
/*****************************************************************************/
  
StateMachine::StateMachine() :
  currentState(state_XX_CheckH),
  currentValue(0),
  currentChecksumByte(0),
  currentChecksumWord(0) {
}

StateMachine::StateMachine(const StateMachine& src) :
  currentState(src.currentState),
  currentPackage(src.currentPackage),
  currentValue(src.currentValue),
  currentChecksumByte(src.currentChecksumByte),
  currentChecksumWord(src.currentChecksumWord) {
}
  
/*****************************************************************************/
/* Destructor                                                                */
/*****************************************************************************/

StateMachine::~StateMachine() {
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

StateMachine& StateMachine::operator=(const StateMachine& src) {
  currentState = src.currentState;

  currentPackage = src.currentPackage;
  
  currentValue = src.currentValue;
  currentChecksumByte = src.currentChecksumByte;
  currentChecksumWord = src.currentChecksumWord;
  
  return *this;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

size_t StateMachine::process(const std::vector<unsigned char>& data, int64_t 
    timestamp, std::deque<SensorPackage>& packages) {
  size_t numPackages = 0;

  for (int i = 0; i < data.size(); ++i) {
    unsigned char c = data[i]; // the actual byte in processing
    
    switch (currentState) { // branch with actual state in fsm
      case state_XX_CheckH:
        if (c == 55)
          // first checker byte
          currentState = state_XX_CheckL;
        else
          // wrong input byte -> restart this state
          currentState = state_XX_CheckH; 
        break;
      case state_XX_CheckL:
        // second checker byte for .66 version
        if (c == 66) {
          // jump the .66 version reading
          currentState = state_66_Start;
          currentPackage.version = SensorPackage::version_66;
          currentPackage.timestamp = timestamp;
        }
        // seconf checker byte for .67 version
        else if (c == 67) {
          // jump the .67 version reading
          currentState = state_67_Start;
          currentPackage.version = SensorPackage::version_67;
          currentPackage.timestamp = timestamp;
        }
        // seconf checker byte for .67 version
        else if (c == 68) {
          // jump the .67 version reading
          currentState = state_68_Start;
          currentPackage.version = SensorPackage::version_68;
          currentPackage.timestamp = timestamp;
        }
        // seconf checker byte for .67 version
        else if (c == 94) {
          // jump the .67 version reading
          currentState = state_94_Start;
          currentPackage.version = SensorPackage::version_94;
          currentPackage.timestamp = timestamp;
        }
        else {
          // wrong input byte -> restart fsm
          restart();
        }
        break;

      /***********************/
      /* .66 version package */
      /***********************/
      
      case state_66_Start:
        currentState = state_66_S1H;
      case state_66_S1H:
        // read a one byte - read the high part of first data
        currentValue = ((unsigned short)c)*256;
        currentState = state_66_S1L; // next state...
        break;
      case state_66_S1L:
        // read a one byte - read the low part of first data - and add
        // to high part
        currentValue += (unsigned short)c;
        // this data convert to (platform specific/default) int type
        currentPackage.rawSignals.resize(4);
        currentPackage.rawSignals[0] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentState = state_66_S2H;  // next state..
        break;
      case state_66_S2H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_66_S2L;
        break;
      case state_66_S2L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[1] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentState = state_66_S3H;
        break;
      case state_66_S3H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_66_S3L;
        break;
      case state_66_S3L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[2] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentState = state_66_S4H;
        break;
      case state_66_S4H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_66_S4L;
        break;
      case state_66_S4L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[3] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        packages.push_back(currentPackage);
        ++numPackages;
        restart();
        break;
        
      /***********************/
      /* .67 version package */
      /***********************/
      
      case state_67_Start:
        currentState = state_67_Config;
      case state_67_Config:
        // read the configuration
        currentPackage.config.fromByte(c);
        // first step of calculating of the checksum
        currentChecksumByte = c;
        currentState = state_67_I1H;
        break;
      case state_67_I1H:
        // read a one byte - read the high part of first data
        currentValue = ((unsigned short)c)*256;
        currentState = state_67_I1L; // next state...
        break;
      case state_67_I1L:
        // read a one byte - read the low part of first data - and add 
        // to high part
        currentValue += (unsigned short)c;
        // branch with configure of actual package
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          // mode_raw branch: reading the mode_raw datas
          // this data convert to (platform specific/default) int type
          currentPackage.rawSignals.resize(4);
          currentPackage.rawSignals[0] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        }
        else {
          // force branch: reading the force vectors
          // this data convert to (platform specific/default) int type
          currentPackage.force.resize(3);
          currentPackage.force[0] = ((int16_t)currentValue);
        }
        currentChecksumByte += currentValue;
        currentState = state_67_I2H; // next state...
        break;
      case state_67_I2H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_67_I2L;
        break;
      case state_67_I2L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[1] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[1] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_67_I3H;
        break;
      case state_67_I3H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_67_I3L;
        break;
      case state_67_I3L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals[2] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          // mode_raw case: normal fsm runing
          currentState = state_67_I4H;
        }
        else {
          currentPackage.force[2] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          // force case: jump two state in fsm because is only three
          // packages in the force case
          currentState = state_67_TH;
        }
        currentChecksumByte += currentValue;
        break;
      case state_67_I4H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_67_I4L;
        break;
      case state_67_I4L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[3] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumByte += currentValue;
        currentState = state_67_TH;
        break;
      case state_67_TH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_67_TL;
        break;
      case state_67_TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature.resize(1);
        currentPackage.temperature[0] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumByte += currentValue;
        currentState = state_67_Checksum;
        break;
      case state_67_Checksum:
        // check the checksum
        if (currentChecksumByte == c)
          currentPackage.checksum = SensorPackage::checksum_okay;
        else
          currentPackage.checksum = SensorPackage::checksum_error;
        packages.push_back(currentPackage);
        ++numPackages;
        // restart the fsm
        restart();
        break;

      /***********************/
      /* .68 version package */
      /***********************/
      
      case state_68_Start:
        currentState = state_68_Config;
      case state_68_Config:
        currentPackage.config.fromByte(c);
        currentChecksumWord = c;
        currentState = state_68_FXH;
        break;
      case state_68_FXH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_FXL;
        break;
      case state_68_FXL:
        currentValue += (unsigned short)c;
        currentPackage.force.resize(3);
        currentPackage.force[0] = ((int16_t)currentValue);
        currentChecksumWord += c;
        currentState = state_68_FYH;
        break;
      case state_68_FYH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_FYL;
        break;
      case state_68_FYL:
        currentValue += (unsigned short)c;
        currentPackage.force[1] = ((int16_t)currentValue);
        currentChecksumWord += c;
        currentState = state_68_FZH;
        break;
      case state_68_FZH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_FZL;
        break;
      case state_68_FZL:
        currentValue += (unsigned short)c;
        currentPackage.force[2] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentState = state_68_TH;
        currentChecksumWord += c;
        break;
      case state_68_TH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_TL;
        break;
      case state_68_TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature.resize(1);
        currentPackage.temperature[0] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S1H;
        break;
      case state_68_S1H:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += (unsigned short)c;
        currentState = state_68_S1L;
        break;
      case state_68_S1L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals.resize(4);
        currentPackage.rawSignals[0] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumWord += c;
        currentState = state_68_S2H;
        break;
      case state_68_S2H:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S2L;
        break;
      case state_68_S2L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[1] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S3H;
        break;
      case state_68_S3H:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S3L;
        break;
      case state_68_S3L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[2] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S4H;
        break;
      case state_68_S4H:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S4L;
        break;
      case state_68_S4L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[3] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S1TH;
        break;
      case state_68_S1TH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S1TL;
        break;
      case state_68_S1TL:
        currentValue += (unsigned short)c;
        currentPackage.compensatedSignals.resize(4);
        currentPackage.compensatedSignals[0] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S2TH;
        break;
      case state_68_S2TH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S2TL;
        break;
      case state_68_S2TL:
        currentValue += (unsigned short)c;
        currentPackage.compensatedSignals[1] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S3TH;
        break;
      case state_68_S3TH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S3TL;
        break;
      case state_68_S3TL:
        currentValue += (unsigned short)c;
        currentPackage.compensatedSignals[2] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_S4TH;
        break;
      case state_68_S4TH:
        currentValue = ((unsigned short)c)*256;
        currentChecksumWord += c;
        currentState = state_68_S4TL;
        break;
      case state_68_S4TL:
        currentValue += (unsigned short)c;
        currentPackage.compensatedSignals[3] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumWord += c;
        currentState = state_68_ChecksumH;
        break;
      case state_68_ChecksumH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_68_ChecksumL;
        break;
      case state_68_ChecksumL:
        currentValue += (unsigned short)c;
        // check the checksum
        if (currentChecksumWord == currentValue)
          currentPackage.checksum = SensorPackage::checksum_okay;
        else
          currentPackage.checksum = SensorPackage::checksum_error;
        packages.push_back(currentPackage);
        ++numPackages;
        // restart the fsm
        restart();
        break;

      /***********************/
      /* .94 version package */
      /***********************/
      
      case state_94_Start:
        currentState = state_94_Config;
      case state_94_Config:
        currentPackage.config.fromByte(c);
        currentChecksumByte = c;
        currentState = state_94_I1H;
        break;

      // 6-axis sensor number 1
      // reading package 1 to buffer 1
          
      case state_94_I1H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I1L;
        break;
      case state_94_I1L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals.resize(16);
          currentPackage.rawSignals[0] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        }
        else {
          currentPackage.force.resize(12);
          currentPackage.force[0] = ((int16_t)currentValue);
        }
        currentChecksumByte += currentValue;
        currentState = state_94_I2H;
        break;
      case state_94_I2H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I2L;
        break;
      case state_94_I2L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[1] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[1] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I3H;
        break;
      case state_94_I3H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I3L;
        break;
      case state_94_I3L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals[2] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_I4H;
        }
        else {
          currentPackage.force[2] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_S1TH;
        }
        currentChecksumByte += currentValue;
        break;
      case state_94_I4H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I4L;
        break;
      case state_94_I4L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[3] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumByte += currentValue;
        currentState = state_94_S1TH;
        break;
      case state_94_S1TH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_S1TL;
        break;
      case state_94_S1TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature.resize(4);
        currentPackage.temperature[0] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumByte += currentValue;
        currentState = state_94_I5H;
        break;

      // 6-axis sensor number 2
      // reading package 2 to buffer 2
          
      case state_94_I5H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I5L;
        break;
      case state_94_I5L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[4] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[3] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I6H;
        break;
      case state_94_I6H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I6L;
        break;
      case state_94_I6L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[5] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[4] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I7H;
        break;
      case state_94_I7H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I7L;
        break;
      case state_94_I7L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals[6] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_I8H;
        }
        else {
          currentPackage.force[5] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_S2TH;
        }
        currentChecksumByte += currentValue;
        break;
      case state_94_I8H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I8L;
        break;
      case state_94_I8L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[7] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumByte += currentValue;
        currentState = state_94_S2TH;
        break;
      case state_94_S2TH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_S2TL;
        break;
      case state_94_S2TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature[1] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumByte += currentValue;
        currentState = state_94_I9H;
        break;

      // 6-axis sensor number 3
      // reading package 3 to buffer 3

      case state_94_I9H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I9L;
        break;
      case state_94_I9L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[8] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[6] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I10H;
        break;
      case state_94_I10H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I10L;
        break;
      case state_94_I10L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[9] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[7] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I11H;
        break;
      case state_94_I11H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I11L;
        break;
      case state_94_I11L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals[10] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_I12H;
        }
        else {
          currentPackage.force[8] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_S3TH;
        }
        currentChecksumByte += currentValue;
        break;
      case state_94_I12H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I12L;
        break;
      case state_94_I12L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[11] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumByte += currentValue;
        currentState = state_94_S3TH;
        break;
      case state_94_S3TH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_S3TL;
        break;
      case state_94_S3TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature[2] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumByte += currentValue;
        currentState = state_94_I13H;
        break;

      // 6-axis sensor number 4
      // reading package 4 to buffer 4

      case state_94_I13H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I13L;
        break;
      case state_94_I13L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[12] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[9] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I14H;
        break;
      case state_94_I14H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I14L;
        break;
      case state_94_I14L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw)
          currentPackage.rawSignals[13] =
            ((unsigned int)currentValue)+UINT_MAX+1;
        else
          currentPackage.force[10] = ((int16_t)currentValue);
        currentChecksumByte += currentValue;
        currentState = state_94_I15H;
        break;
      case state_94_I15H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I15L;
        break;
      case state_94_I15L:
        currentValue += (unsigned short)c;
        if (currentPackage.config.getMode() == SensorConfig::mode_raw) {
          currentPackage.rawSignals[14] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_I16H;
        }
        else {
          currentPackage.force[11] =
            ((unsigned int)currentValue)+UINT_MAX+1;
          currentState = state_94_S4TH;
        }
        currentChecksumByte += currentValue;
        break;
      case state_94_I16H:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_I16L;
        break;
      case state_94_I16L:
        currentValue += (unsigned short)c;
        currentPackage.rawSignals[15] =
          ((unsigned int)currentValue)+UINT_MAX+1;
        currentChecksumByte += currentValue;
        currentState = state_94_S4TH;
        break;
      case state_94_S4TH:
        currentValue = ((unsigned short)c)*256;
        currentState = state_94_S4TL;
        break;
      case state_94_S4TL:
        currentValue += (unsigned short)c;
        currentPackage.temperature[3] =
          (((unsigned int)currentValue)+UINT_MAX+1);
        currentChecksumByte += currentValue;
        currentState = state_94_Checksum;
        break;
      case state_94_Checksum:
        // checksum ingnored
        currentPackage.checksum = SensorPackage::checksum_none;
        packages.push_back(currentPackage);
        ++numPackages;
        // restart the fsm
        restart();
        break;
    }
  }
  
  return numPackages;
}

void StateMachine::restart() {
  currentState = state_XX_CheckH;
  
  currentPackage.clear();
  
  currentValue = 0;
  currentChecksumByte = 0;
  currentChecksumWord = 0;
}

}
