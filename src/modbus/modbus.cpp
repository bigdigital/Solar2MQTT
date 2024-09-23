// #define isDEBUG
#include "ArduinoJson.h"
#include "modbus.h"

SoftwareSerial myPort;
extern void writeLog(const char *format, ...);
//----------------------------------------------------------------------
//  Public Functions
//----------------------------------------------------------------------

MODBUS::MODBUS()
{
    this->my_serialIntf = &myPort;
}

void MODBUS::preTransmission()
{
    digitalWrite(RS485_DIR_PIN, 1);
}

void MODBUS::postTransmission()
{
    digitalWrite(RS485_DIR_PIN, 0);
}

bool MODBUS::Init()
{
    // Null check the serial interface
    if (this->my_serialIntf == NULL)
    {
        writeLog("No serial specificed!");
        return false;
    }
    this->my_serialIntf->setTimeout(500);
    this->my_serialIntf->begin(RS485_BAUDRATE, SWSERIAL_8N1, RS485_RX, RS485_TX, false);

    // Init in receive mode
    pinMode(RS485_DIR_PIN, OUTPUT);
    this->postTransmission();

    // Callbacks allow us to configure the RS485 transceiver correctly
    mb.preTransmission(preTransmission);
    mb.postTransmission(postTransmission);

    mb.begin(INVERTER_MODBUS_ADDR, *this->my_serialIntf);

    live_info = {
        .variant = &liveData,
        .registers = registers_live,
        .array_size = sizeof(registers_live) / sizeof(modbus_register_t),
        .curr_register = 0};

    return true;
}

bool MODBUS::loop()
{
    if (millis() - previousTime > delayTime)
    {
        if (parseModbusToJson(live_info))
        {
            connectionCounter = 0;
        }
        else
        {
            connectionCounter++;
        }
        connection = (connectionCounter < 5) ? true : false;
        previousTime = millis();

        if (live_info.curr_register == live_info.array_size)
        {
            requestCallback();
        }
    }
    return true;
}

void MODBUS::callback(std::function<void()> func)
{
    requestCallback = func;
}

String MODBUS::sendCommand(String command)
{
    if (command == "")
    {
        return command;
    }
    customCommandBuffer = command;
    return command;
}

bool MODBUS::getModbusResultMsg(uint8_t result)
{
    String tmpstr2 = "";
    switch (result)
    {
    case mb.ku8MBSuccess:
        return true;
        break;
    case mb.ku8MBIllegalFunction:
        tmpstr2 = "Illegal Function";
        break;
    case mb.ku8MBIllegalDataAddress:
        tmpstr2 = "Illegal Data Address";
        break;
    case mb.ku8MBIllegalDataValue:
        tmpstr2 = "Illegal Data Value";
        break;
    case mb.ku8MBSlaveDeviceFailure:
        tmpstr2 = "Slave Device Failure";
        break;
    case mb.ku8MBInvalidSlaveID:
        tmpstr2 = "Invalid Slave ID";
        break;
    case mb.ku8MBInvalidFunction:
        tmpstr2 = "Invalid Function";
        break;
    case mb.ku8MBResponseTimedOut:
        tmpstr2 = "Response Timed Out";
        break;
    case mb.ku8MBInvalidCRC:
        tmpstr2 = "Invalid CRC";
        break;
    default:
        tmpstr2 = "Unknown error: " + String(result);
        break;
    }
    writeLog("%s", tmpstr2.c_str());
    return false;
}

bool MODBUS::getModbusValue(uint16_t register_id, modbus_entity_t modbus_entity, uint16_t *value_ptr)
{
    // writeLog("Requesting data");
    for (uint8_t i = 0; i < MODBUS_RETRIES; ++i)
    {
        if (MODBUS_RETRIES > 1)
        {
            writeLog("Trial %d/%d", i + 1, MODBUS_RETRIES);
        }
        if (modbus_entity == MODBUS_TYPE_HOLDING)
        {
            uint8_t result;
            result = mb.readHoldingRegisters(register_id, 1);
            bool modbusResult;
            modbusResult = getModbusResultMsg(result);
            if (modbusResult)
            {
                *value_ptr = mb.getResponseBuffer(0);
                return true;
            }
        }
        else
        {
            writeLog("Unsupported Modbus entity type");
            value_ptr = nullptr;
            return false;
        }
    }
    // Time-out
    writeLog("Time-out");
    value_ptr = nullptr;
    return false;
}

String MODBUS::toBinary(uint16_t input)
{
    String output;
    while (input != 0)
    {
        output = (input % 2 == 0 ? "0" : "1") + output;
        input /= 2;
    }
    return output;
}

bool MODBUS::decodeDiematicDecimal(uint16_t int_input, int8_t decimals, float *value_ptr)
{
    if (int_input == 65535)
    {
        value_ptr = nullptr;
        return false;
    }
    else
    {
        uint16_t masked_input = int_input & 0x7FFF;
        float output = static_cast<float>(masked_input);
        if (int_input >> 15 == 1)
        {
            output = -output;
        }
        *value_ptr = output / pow(10, decimals);
        return true;
    }
}

bool MODBUS::readModbusRegisterToJson(const modbus_register_t *reg, JsonObject *variant)
{
    // register found
    if (reg == nullptr || variant == nullptr)
    {
        return false; // Return false if invalid input
    }
    writeLog("Register id=%d type=0x%x name=%s", reg->id, reg->type, reg->name);
    uint16_t raw_value = 0;

    float final_value;
    if (getModbusValue(reg->id, reg->modbus_entity, &raw_value))
    {
        // writeLog("Raw value: %s=%#06x", reg->name, raw_value);

        switch (reg->type)
        {
        case REGISTER_TYPE_U16:
            // writeLog("Value: %u", raw_value);
            (*variant)[reg->name] = raw_value;
            break;

        case REGISTER_TYPE_DIEMATIC_ONE_DECIMAL:
            if (decodeDiematicDecimal(raw_value, 1, &final_value))
            {
                // writeLog("Value: %.1f", final_value);
                (*variant)[reg->name] = final_value;
            }
            else
            {
                writeLog("Invalid Diematic value");
            }
            break;

        case REGISTER_TYPE_DIEMATIC_TWO_DECIMAL:
            if (decodeDiematicDecimal(raw_value, 2, &final_value))
            {
                // writeLog("Value: %.1f", final_value);
                (*variant)[reg->name] = final_value;
            }
            else
            {
                writeLog("Invalid Diematic value");
            }
            break;
        case REGISTER_TYPE_BITFIELD:

            for (uint8_t j = 0; j < 16; ++j)
            {
                const char *bit_varname = reg->optional_param.bitfield[j];
                if (bit_varname == nullptr)
                {
                    writeLog("[bit%02d] end of bitfield reached", j);
                    break;
                }
                const uint8_t bit_value = raw_value >> j & 1;
                writeLog(" [bit%02d] %s=%d", j, bit_varname, bit_value);
                (*variant)[bit_varname] = bit_value;
            }
            break;

        case REGISTER_TYPE_CUSTOM_VAL_NAME:
        {
            bool isfound = false;
            for (uint8_t j = 0; j < 16; ++j)
            {
                const char *bit_varname = reg->optional_param.bitfield[j];
                if (bit_varname == nullptr)
                {
                    writeLog("bitfield[%d] is null", j);
                    break;
                }
                if (j == raw_value)
                {
                    // writeLog("Match found, value: %s", bit_varname);
                    (*variant)[reg->name] = bit_varname;
                    isfound = true;
                    break;
                }
            }
            if (!isfound)
            {
                writeLog("CUSTOM_VAL_NAME not found for raw_value=%d", raw_value);
            }
            break;
        }
        case REGISTER_TYPE_DEBUG:

            writeLog("Raw DEBUG value: %s=%#06x %s", reg->name, raw_value, toBinary(raw_value).c_str());
            break;

        default:
            // Unsupported type

            writeLog("Unsupported register type");
            break;
        }
    }
    else
    {
        writeLog("Request failed!");
        return false;
    }
    writeLog("Request OK!");
    return true;
}

bool MODBUS::parseModbusToJson(modbus_register_info_t &register_info)
{
    // writeLog("Parsing Modbus registers");
    if (register_info.curr_register >= register_info.array_size)
    {
        register_info.curr_register = 0;
    }
    // writeLog("Registers size %d, curr %d", register_info.array_size, register_info.curr_register);
    for (; register_info.curr_register <= register_info.array_size - 1; register_info.curr_register++)
    {
        bool ret_val = readModbusRegisterToJson(&register_info.registers[register_info.curr_register], register_info.variant);

        if (!ret_val)
        {
            return false;
        }
    }

    return true;
}