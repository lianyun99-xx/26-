#ifndef MOTOR6014_PROTOCOL_H
#define MOTOR6014_PROTOCOL_H

#include <stddef.h>
#include "motor6014_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unitree IM6014 / N6014B-12.6 protocol:
 * https://doc-cdn.unitree.com/30/1383/zh/30_1383_zh
 * All multibyte fields and stored CRC values are little endian.
 */

/*
 * CRC-32: init 0xFFFFFFFF, polynomial 0x04C11DB7, no reflection/xorout.
 * Assemble little-endian 32-bit words and process each word MSB first.
 * length must be divisible by four. Invalid input returns zero; an empty
 * input returns the initial CRC value. The caller supplies the exact range.
 */
uint32_t Motor6014_ProtocolCRC(const uint8_t *bytes, size_t length);

/*
 * Encode a 20-byte command for IDs 1..12. Rejects unsupported modes,
 * non-finite numbers, negative gains and any unrepresentable raw value.
 * Kp/Kd are signed 16-bit fields with only 0..32767 permitted; their
 * negative encodings are reserved by the official protocol.
 * Quantization rounds to nearest, with half values away from zero. Values
 * outside the continuous raw range are rejected before rounding. Neither
 * command nor frame is changed on failure.
 */
Motor6014_Status_t Motor6014_EncodeCommand(
    uint8_t id,
    const Motor6014_Command_t *command,
    uint8_t frame[MOTOR6014_TX_SIZE]);

/*
 * Validate header, ID and CRC before changing data. Success sets valid=true
 * and leaves timestamp/sequence zero; the transport supplies those values.
 * Reserved feedback mode values are retained for diagnostic visibility.
 */
Motor6014_Status_t Motor6014_DecodeFeedback(
    const uint8_t frame[MOTOR6014_RX_SIZE],
    Motor6014_Data_t *data);

/* Gear ratio is exactly 38/3. Torque conversion assumes ideal efficiency. */
Motor6014_MechanicalState_t Motor6014_RotorToOutput(
    Motor6014_MechanicalState_t rotor);
Motor6014_MechanicalState_t Motor6014_OutputToRotor(
    Motor6014_MechanicalState_t output);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR6014_PROTOCOL_H */
