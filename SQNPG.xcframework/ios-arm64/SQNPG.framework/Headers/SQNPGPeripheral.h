//
//  SQNPGPeripheral.h
//

#ifndef SQNPGPeripheral_h
#define SQNPGPeripheral_h

#include <stdint.h>

typedef struct SQ_LLA_msg_s {
    double lat;  // lattitude in degrees
    double lon;  // longitude in degrees
    double alt;  // altitude in meters
    double hAcc; // horizontal accuracy in meters
    double vAcc; // vertical accuracy in meters
} SQ_LLA_msg_t;

typedef struct SQ_status_msg_s {
    double iTOW;
    uint8_t status;
    uint8_t numSVs;
    uint8_t battery;
} SQ_status_msg_t;

typedef struct SQ_lb_config_msg_s {
    uint8_t mode;
    double lat;        // degrees
    double lon;        // degrees (not radians Andrew!)
    double alt;        // meters (WGS84)
    double hAcc;       // meters
    double vAcc;       // meters
    uint32_t duration; // auto survey duration in seconds
    double altMsl;     // meters (height above mean sea level)
} SQ_lb_config_msg_t;

// Convert a Legacy BT interface LLA data buffer to an LLA message structure
// returns 0 on success, negative on failure
int data2LLA(const uint8_t *data, uint16_t length, SQ_LLA_msg_t *msg);

// Convert a Legacy BT interface Status data buffer to a Status message structure
// returns 0 on success, negative on failure
int data2status(const uint8_t *data, uint16_t length, SQ_status_msg_t *msg);

// Convert a BT interface lb_config data buffer to an lb_config message structure
// returns 0 on success, negative on failure
int data2lbConfig(const uint8_t *data, uint16_t length, SQ_lb_config_msg_t *msg);

// Convert an lb_config message structure into an lb_config data buffer for BT interface
// returns 0 on success, negative on failure
int lbConfig2data(const SQ_lb_config_msg_t *msg, uint8_t *data, uint16_t length);

#endif /* SQNPGPeripheral_h */

