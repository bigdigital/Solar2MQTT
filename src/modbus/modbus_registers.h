#ifndef SRC_MODBUS_REGISTERS_H_
#define SRC_MODBUS_REGISTERS_H_
#include "Arduino.h"

typedef enum {
    MODBUS_TYPE_HOLDING = 0x00,         /*!< Modbus Holding register. */
//    MODBUS_TYPE_INPUT,                  /*!< Modbus Input register. */
//    MODBUS_TYPE_COIL,                   /*!< Modbus Coils. */
//    MODBUS_TYPE_DISCRETE,               /*!< Modbus Discrete bits. */
//    MODBUS_TYPE_COUNT,
//    MODBUS_TYPE_UNKNOWN = 0xFF
} modbus_entity_t;

typedef enum {
//    REGISTER_TYPE_U8 = 0x00,                   /*!< Unsigned 8 */
    REGISTER_TYPE_U16 = 0x01,                  /*!< Unsigned 16 */
//    REGISTER_TYPE_U32 = 0x02,                  /*!< Unsigned 32 */
//    REGISTER_TYPE_FLOAT = 0x03,                /*!< Float type */
//    REGISTER_TYPE_ASCII = 0x04,                 /*!< ASCII type */
    REGISTER_TYPE_DIEMATIC_ONE_DECIMAL = 0x05,
    REGISTER_TYPE_BITFIELD = 0x06,
    REGISTER_TYPE_DEBUG = 0x07,
    REGISTER_TYPE_CUSTOM_VAL_NAME = 0x08,
} register_type_t;
 
 
typedef union {
    const char* bitfield[16];
} optional_param_t;

typedef struct {
    uint16_t            id;
    modbus_entity_t     modbus_entity;      /*!< Type of modbus parameter */
    register_type_t     type;               /*!< Float, U8, U16, U32, ASCII, etc. */
    const char*         name;
    optional_param_t    optional_param;
} modbus_register_t;

const modbus_register_t registers_live[] = {
    { 25201, MODBUS_TYPE_HOLDING, REGISTER_TYPE_CUSTOM_VAL_NAME, "Inverter_Operation_Mode",{ .bitfield = {
            "Power On",
            "Self Test",
            "OffGrid",
            "GridTie",
            "ByPass",
            "Stop",
            "GridCharging",
    } } },
    
   // { 25205, MODBUS_TYPE_HOLDING, REGISTER_TYPE_DIEMATIC_ONE_DECIMAL, "Battery_Voltage"},
   // { 25207, MODBUS_TYPE_HOLDING, REGISTER_TYPE_DIEMATIC_ONE_DECIMAL, "Grid_voltage"}, 
   // { 25208, MODBUS_TYPE_HOLDING, REGISTER_TYPE_DIEMATIC_ONE_DECIMAL, "Inverter_Bus_Voltage"},
   // { 25216, MODBUS_TYPE_HOLDING, REGISTER_TYPE_U16, "Output_load_percent"},
    //{ 15205, MODBUS_TYPE_HOLDING, REGISTER_TYPE_DIEMATIC_ONE_DECIMAL, "PV_Input_Voltage"},
 
};
 
#endif  // SRC_MODBUS_REGISTERS_H_