#include "SoftwareSerial.h"
#ifndef MODBUS_H
#define MODBUS_H

#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include "modbus_registers.h"
extern JsonObject deviceJson;
extern JsonObject staticData;
extern JsonObject liveData;

#define RS485_TX D6
#define RS485_RX D7
#define RS485_DIR_PIN D5
#define RS485_BAUDRATE 19200

#define INVERTER_MODBUS_ADDR 4

#define MODBUS_RETRIES 2

typedef struct
{
    JsonObject *variant;
    const modbus_register_t *registers;
    uint8_t array_size;
    uint8_t curr_register;
} modbus_register_info_t;

class MODBUS
{
public:
    bool requestStaticData = true;
    bool connection = false;
    modbus_register_info_t live_info;
    MODBUS();

    /**
     * @brief Initializes this driver
     * @details Configures the serial peripheral and pre-loads the transmit buffer with command-independent bytes
     */
    bool Init();

    /**
     * @brief Updating the Data from the BMS
     */
    bool loop();

    /**
     * @brief Send custom command to the device
     */
    String sendCommand(String command);

    /**
     * @brief callback function
     *
     */
    void callback(std::function<void()> func);
    std::function<void()> requestCallback;
    bool readModbusRegisterToJson(const modbus_register_t *reg, JsonObject *variant);
    bool parseModbusToJson(modbus_register_info_t& register_info );

private:
    unsigned long previousTime = 0;
    unsigned long delayTime = 200;
    unsigned long cmdDelayTime = 3000;
    byte requestCounter = 0;

    long long int connectionCounter = 0;

    byte qexCounter = 0;

    String customCommandBuffer;

    static void preTransmission();
    static void postTransmission();
    String toBinary(uint16_t input);
    bool decodeDiematicDecimal(uint16_t int_input, int8_t decimals, float *value_ptr);
    bool getModbusResultMsg(uint8_t result);
    bool getModbusValue(uint16_t register_id, modbus_entity_t modbus_entity, uint16_t *value_ptr);
    /**
     * @brief get the crc from a string
     */
    uint16_t getCRC(String data);

    /**
     * @brief get the crc from a string
     */
    byte getCHK(String data);

    /**
     * @brief Sends a complete packet with the specified command
     * @details calculates the checksum and sends the command over the specified serial connection
     */
    String requestData(String command);

    /**
     * @brief Serial interface used for communication
     * @details This is set in the constructor
     */
    SoftwareSerial *my_serialIntf;

    ModbusMaster mb;
};

#endif
