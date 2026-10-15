#include "motor6014_protocol.h"

#include <float.h>
#include <limits.h>
#include <string.h>

#define MOTOR6014_TWO_PI 6.283185307179586476925286766559

static uint16_t ReadU16LE(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static uint32_t ReadU32LE(const uint8_t *bytes)
{
    return (uint32_t)bytes[0]
        | ((uint32_t)bytes[1] << 8)
        | ((uint32_t)bytes[2] << 16)
        | ((uint32_t)bytes[3] << 24);
}

static int32_t ReadI16LE(const uint8_t *bytes)
{
    const uint16_t raw = ReadU16LE(bytes);
    return (raw & 0x8000U) != 0U ? (int32_t)raw - 65536 : (int32_t)raw;
}

static int32_t ReadI32LE(const uint8_t *bytes)
{
    const uint32_t raw = ReadU32LE(bytes);
    if ((raw & 0x80000000UL) != 0U) {
        return (int32_t)((int64_t)raw - INT64_C(4294967296));
    }
    return (int32_t)raw;
}

static void WriteU16LE(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static void WriteU32LE(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
}

/* No libm dependency, and no undefined float-to-integer conversions. */
static Motor6014_Status_t Quantize(
    float value, double scale, int64_t minimum, int64_t maximum, int64_t *raw)
{
    double scaled;
    int64_t rounded;

    if (value != value || value > FLT_MAX || value < -FLT_MAX) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    scaled = (double)value * scale;
    if (scaled < (double)minimum || scaled > (double)maximum) {
        return MOTOR6014_OUT_OF_RANGE;
    }

    /* All callers limit scaled to at most the 32-bit integer range. */
    rounded = (int64_t)(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
    if (rounded < minimum || rounded > maximum) {
        return MOTOR6014_OUT_OF_RANGE;
    }
    *raw = rounded;
    return MOTOR6014_OK;
}

uint32_t Motor6014_ProtocolCRC(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    size_t offset;

    if ((bytes == NULL && length != 0U) || (length % 4U) != 0U) {
        return 0U;
    }
    for (offset = 0U; offset < length; offset += 4U) {
        uint32_t bit;
        crc ^= ReadU32LE(&bytes[offset]);
        for (bit = 0U; bit < 32U; ++bit) {
            crc = (crc & UINT32_C(0x80000000)) != 0U
                ? (crc << 1) ^ UINT32_C(0x04C11DB7)
                : crc << 1;
        }
    }
    return crc;
}

Motor6014_Status_t Motor6014_EncodeCommand(
    uint8_t id,
    const Motor6014_Command_t *command,
    uint8_t frame[MOTOR6014_TX_SIZE])
{
    uint8_t encoded[MOTOR6014_TX_SIZE] = {0};
    int64_t torque;
    int64_t velocity;
    int64_t position;
    int64_t kp;
    int64_t kd;
    Motor6014_Status_t status;

    if (id < MOTOR6014_ID_MIN || id > MOTOR6014_ID_MAX) {
        return MOTOR6014_INVALID_ID;
    }
    if (command == NULL || frame == NULL
        || (command->mode != MOTOR6014_MODE_STOP
            && command->mode != MOTOR6014_MODE_FOC)) {
        return MOTOR6014_INVALID_ARGUMENT;
    }

    status = Quantize(command->torque, 2560.0, INT16_MIN, INT16_MAX, &torque);
    if (status != MOTOR6014_OK) {
        return status;
    }
    status = Quantize(command->velocity, 64.0 / MOTOR6014_TWO_PI,
        INT16_MIN, INT16_MAX, &velocity);
    if (status != MOTOR6014_OK) {
        return status;
    }
    status = Quantize(command->position, 32768.0 / MOTOR6014_TWO_PI,
        INT32_MIN, INT32_MAX, &position);
    if (status != MOTOR6014_OK) {
        return status;
    }
    /* Official int16 gain fields reserve negative encodings. */
    status = Quantize(command->kp, 12800.0, 0, INT16_MAX, &kp);
    if (status != MOTOR6014_OK) {
        return status;
    }
    status = Quantize(command->kd, 51200.0, 0, INT16_MAX, &kd);
    if (status != MOTOR6014_OK) {
        return status;
    }

    encoded[0] = 0xFEU;
    encoded[1] = 0xEEU;
    encoded[2] = (uint8_t)(id | ((uint8_t)command->mode << 4)
        | (command->timeout_enable ? 0x80U : 0U));
    /* Byte 3 is reserved and must be transmitted as zero. */
    WriteU16LE(&encoded[4], (uint16_t)torque);
    WriteU16LE(&encoded[6], (uint16_t)velocity);
    WriteU32LE(&encoded[8], (uint32_t)position);
    WriteU16LE(&encoded[12], (uint16_t)kp);
    WriteU16LE(&encoded[14], (uint16_t)kd);
    WriteU32LE(&encoded[16], Motor6014_ProtocolCRC(encoded, 16U));
    memcpy(frame, encoded, sizeof(encoded));
    return MOTOR6014_OK;
}

Motor6014_Status_t Motor6014_DecodeFeedback(
    const uint8_t frame[MOTOR6014_RX_SIZE],
    Motor6014_Data_t *data)
{
    Motor6014_Data_t decoded = {0};
    uint8_t id;

    if (frame == NULL || data == NULL) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    if (frame[0] != 0xFCU || frame[1] != 0xEEU) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    id = frame[2] & 0x0FU;
    if (id < MOTOR6014_ID_MIN || id > MOTOR6014_ID_MAX) {
        return MOTOR6014_INVALID_ID;
    }
    if (Motor6014_ProtocolCRC(&frame[2], 20U) != ReadU32LE(&frame[22])) {
        return MOTOR6014_CRC_ERROR;
    }

    decoded.id = id;
    decoded.mode = (Motor6014_Mode_t)((frame[2] >> 4) & 0x07U);
    decoded.timeout_triggered = (frame[2] & 0x80U) != 0U;
    decoded.driver_temperature = frame[3] < 0x80U
        ? (int8_t)frame[3] : (int8_t)((int16_t)frame[3] - 256);
    decoded.winding_temperature = frame[4];
    decoded.voltage = (float)frame[5] / 2.0f;
    decoded.torque = (float)((double)ReadI16LE(&frame[6]) / 2560.0);
    decoded.velocity = (float)((double)ReadI16LE(&frame[8]) * MOTOR6014_TWO_PI / 64.0);
    decoded.position = (float)((double)ReadI32LE(&frame[10]) * MOTOR6014_TWO_PI / 32768.0);
    decoded.error = ReadU32LE(&frame[14]);
    decoded.warning = (uint8_t)((ReadU16LE(&frame[18]) >> 13) & 0x07U);
    decoded.valid = true;
    *data = decoded;
    return MOTOR6014_OK;
}

Motor6014_MechanicalState_t Motor6014_RotorToOutput(
    Motor6014_MechanicalState_t rotor)
{
    Motor6014_MechanicalState_t output;
    output.position = rotor.position * (3.0f / 38.0f);
    output.velocity = rotor.velocity * (3.0f / 38.0f);
    output.torque = rotor.torque * (38.0f / 3.0f);
    return output;
}

Motor6014_MechanicalState_t Motor6014_OutputToRotor(
    Motor6014_MechanicalState_t output)
{
    Motor6014_MechanicalState_t rotor;
    rotor.position = output.position * (38.0f / 3.0f);
    rotor.velocity = output.velocity * (38.0f / 3.0f);
    rotor.torque = output.torque * (3.0f / 38.0f);
    return rotor;
}
