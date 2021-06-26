#ifndef LORA_H
#define LORA_H

#include "../../include/configs.h"

#ifdef USE_LORA_COMMUNICATION
#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <cppQueue.h>

// For mathematical variables
#include "../Helper_3dmath/Helper_3dmath.h"

#define TX_BUFFER_LEN 5
#define RX_BUFFER_LEN 5

/* The new style packet use a LoraPacket class as a middle data type *
 * , once you want to design a new type packet, you should provide   *
 * the following functions to your new class inherited from          *
 * LoraPacket.                                                       *
 * - The two constructors                                            *
 * - serialize(): which copy the data to buffer.                     */
class LoraPacket
{
protected:
    void pack_header(const uint8_t size, uint8_t *buf);

public:
    uint8_t id;
    uint8_t timestamp;

    /* Feed with the data to send. */
    LoraPacket(uint8_t _id);

    /* Copy the serialized data into buffer. */
    virtual void serialize(uint8_t *buf) = 0;
};

class LoraPacketFloat_3d : public LoraPacket
{
private:
public:
    static const uint8_t size = 6;
    union {
        float vector[3];
        uint8_t raw[size];
    } data;

    /* Feed with the data to send. */
    LoraPacketFloat_3d(uint8_t _id, const VectorFloat &v);

    /* Construct the packet with raw data. */
    LoraPacketFloat_3d(uint8_t *buf);

    /* Copy the serialized data into buffer. */
    virtual void serialize(uint8_t *buf);
};

class LoraPacketMessage : public LoraPacket
{
private:
public:
    static const uint8_t size = 15;
    char *msg;

    /* Feed with the data to send. */
    LoraPacketMessage(const uint8_t _id, const uint8_t tag, char *_msg);

    /* Construct the packet with raw data. */
    LoraPacketMessage(uint8_t *buf);

    /* Copy the serialized data into buffer. */
    virtual void serialize(uint8_t *buf);
};

class LoraCommunication
{
private:
public:
    cppQueue q_tx;
    cppQueue q_rx;
    time_t cadTime;
    bool cadBusy;
    uint8_t TxBuffer[LORA_PACKET_SIZE];
    uint8_t RxBuffer[LORA_PACKET_SIZE];

    LoraCommunication();

    void begin(void);

    /* Return the number of packet received. */
    int available();

    /* Return true if succeed to push. */
    bool send(LoraPacket *p);

    void update();
};

void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);

#endif

#endif