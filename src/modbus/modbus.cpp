// #define isDEBUG
#include "modbus.h"

//----------------------------------------------------------------------
//  Public Functions
//----------------------------------------------------------------------

MODBUS::MODBUS(SoftwareSerial *port)
{
    my_serialIntf = port;
}

bool MODBUS::Init()
{
    // Null check the serial interface
    if (this->my_serialIntf == NULL)
    {
        writeLog("No serial specificed!");
        return false;
    }
    // this->my_serialIntf->setTimeout(2000);

    return true;
}

bool MODBUS::setProtocol(protocol_type_t protocol)
{
    char modelName[30];
    if (device != nullptr)
    {
        delete device;
        device = nullptr;
    }
    switch (protocol)
    {
    case MODBUS_DEYE:
        device = new Deye();
        break;
    case MODBUS_ANENJI:
        device = new Anenji();
        break;
    case MODBUS_MUST:
        device = new MustPV_PH18();
        break;
    case MODBUS_POW_HVM:
        device = new Pow_Hvm();
        break;
    default:
        break;
    }

    if (device != nullptr)
    {
        device->init(*my_serialIntf, _mCom);
        prepareRegisters();
        requestStaticData = true;
        connectionCounter = 0;
        previousTime = millis();
        bool ret = device->retrieveModel(_mCom, modelName, sizeof(modelName));
        if (ret && strlen(modelName) != 0)
        {
            staticData["Device_Model"] = modelName;
            return true;
        }
    }
    else
    {
        writeLog("Unsupported protocol or failed to initialize device.");
    }
    return false;
}

void MODBUS::prepareRegisters()
{
    live_info = {
        .variant = &liveData,
        .registers = device->getLiveRegisters(),
        .array_size = device->getLiveRegistersCount(),
        .curr_register = 0};
    static_info = {
        .variant = &staticData,
        .registers = device->getStaticRegisters(),
        .array_size = device->getStaticRegistersCount(),
        .curr_register = 0};
}

void MODBUS::loop()
{
    if (device == nullptr)
    {
        return;
    }

    if (millis() - previousTime < cmdDelayTime)
    {
        return;
    }
    modbus_register_info_t *cur_info_registers = &live_info;
    if (requestStaticData)
    {
        cur_info_registers = &static_info;
    }
    switch (_mCom.parseModbusToJson(*cur_info_registers))
    {
    case READ_OK:
        connectionCounter = 0;
        break;
    case READ_FAIL:
        connectionCounter++;
        break;
    default:
        break;
    }

    connection = connectionCounter < MAX_CONNECTION_ATTEMPTS;
    if (_mCom.isAllRegistersRead(*cur_info_registers))
    {
        requestStaticData = false;
        requestCallback();
    }

    previousTime = millis();
}

void MODBUS::callback(std::function<void()> func)
{
    requestCallback = func;
}

String MODBUS::requestData(String command)
{
    uint16_t registerAddress = 0;
    uint16_t data[10]; // Assume a maximum of 10 data points
    size_t dataSize = 0;
    bool isGetCommand = false;
    if (parseCommand(command, registerAddress, data, dataSize, isGetCommand))
    {
        if (!isGetCommand)
        {
            _mCom.sendDataToModbus(registerAddress, data, dataSize);
            requestStaticData = true;
        }
        else
        {
            _mCom.fetchDataFromModbus(registerAddress, dataSize);
        }
    }
    return "";
}



// parse the command, extracting the Modbus register and data
// set_r=186,0x5020, or set_r=186,35 , where 186, is register address, and data
// get_r=186,10 , where 186, is register address, and data registers need to read

bool MODBUS::parseCommand(const String &input, uint16_t &registerAddress, uint16_t *data, size_t &dataSize, bool &isGetCommand)
{
     if (input.isEmpty()) {
        return false;  // Invalid input
    }
    char buffer[64];
    strncpy(buffer, input.c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';               

    char *token = strtok(buffer, "=");                  // Split the command into parts, command and data
    dataSize = 0;

    if (strcmp(token, "get_r") == 0)
    {
        isGetCommand = true;
    }
    else if (strcmp(token, "set_r") == 0)
    {
        isGetCommand = false;
    }
    else
    {
        writeLog("Invalid command");
        return false;
    }

    token = strtok(nullptr, ",");  // Extract register address
    if (!token) {
        writeLog("Missing register address");
            return false;
        return false;  // Missing register address
    }

    // First token is the register address
    char *endptr;
    registerAddress = strtoul(token, &endptr, 10); // Convert to uint16_t register address
    if (*endptr != '\0')
    {
        writeLog("Invalid register address");
        return false;
    }
    if (isGetCommand)
    {
        token = strtok(nullptr, ",");
        if (!token)
        {
            writeLog("No number of registers specified");
            return false;
        }
        dataSize = strtoul(token, &endptr, 10);
        if (*endptr != '\0')
        {
            writeLog("Invalid number of registers");
            return false;
        }

        return true;
    }
     // If it's a set_r command
    while ((token = strtok(nullptr, ",")) != nullptr)
    {
        if (dataSize >= 10)// Limit to 10 values for safety
        { 
            return false;
        }

        if (strstr(token, "0x") == token)
        {
            data[dataSize] = strtoul(token, &endptr, 16); // Parse as hexadecimal
        }
        else
        {
            data[dataSize] = strtoul(token, &endptr, 10); // Parse as decimal
        }

        if (*endptr != '\0')
        {
            writeLog("Invad data value");
            return false;
        }

        dataSize++;
    }

    if (dataSize == 0)
    {
        writeLog("No valid data provided");
        return false;
    }

    return true;
}

//----------------------------------------------------------------------
// Private Functions
//----------------------------------------------------------------------
protocol_type_t MODBUS::autoDetect() // function for autodetect the inverter type
{
    writeLog("Try Autodetect Modbus device");
    const size_t deviceCount = sizeof(modbus_protocols) / sizeof(modbus_protocols[0]);

    for (size_t i = 0; i < deviceCount; ++i)
    {
        if (setProtocol(modbus_protocols[i]))
        {
            writeLog("<Autodetect> Found Modbus device: %s", staticData["Device_Model"]);
            return modbus_protocols[i];
        }
    }

    return NoD;
}
