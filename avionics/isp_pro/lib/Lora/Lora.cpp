#include "Lora.h"

#ifdef USE_LORA_COMMUNICATION
hw_config hwConfig;
RadioEvents_t RadioEvents;

LoraCommunication lora;

LoraPacket::LoraPacket(uint8_t _id)
{
    id = _id;
    // crop into 8 bits variable
    // unit in 1/16 sec, overflow interval: 16 sec
    timestamp = (millis() / 16) & 0xFF;
}

void LoraPacket::pack_header(const uint8_t size, uint8_t *buf)
{
    buf[0] = size + 3;
    buf[1] = id;
    buf[2] = timestamp;
}

LoraPacketFloat_3d::LoraPacketFloat_3d(uint8_t _id, const VectorFloat &v)
    : LoraPacket(_id)
{
    // copy the data
    data.vector[0] = v.x;
    data.vector[1] = v.y;
    data.vector[2] = v.z;
}

LoraPacketFloat_3d::LoraPacketFloat_3d(uint8_t *buf) : LoraPacket(buf[1])
{
    timestamp = buf[2];
    memcpy(data.raw, buf + 3, size);
}

void LoraPacketFloat_3d::serialize(uint8_t *buf)
{
    pack_header(size, buf);
    memcpy(buf + 3, data.raw, size);
}

LoraPacketMessage::LoraPacketMessage(const uint8_t _id,
                                     const uint8_t tag,
                                     char *_msg)
    : LoraPacket(_id)
{
    // copy the data
    msg = _msg;
}

LoraPacketMessage::LoraPacketMessage(uint8_t *buf) : LoraPacket(buf[1])
{
    timestamp = buf[2];
    msg = (char *) buf + 3;
}

void LoraPacketMessage::serialize(uint8_t *buf)
{
    pack_header(size, buf);
    memcpy(buf + 3, msg, size);
}

LoraCommunication::LoraCommunication()
    : q_tx(sizeof(uint8_t) * LORA_PACKET_SIZE, TX_BUFFER_LEN, FIFO, false),
      q_rx(sizeof(uint8_t) * LORA_PACKET_SIZE, RX_BUFFER_LEN, FIFO, false)
{
    cadBusy = false;
}

void LoraCommunication::begin()
{
    // Example uses an eByte E22 module with an SX1262
    hwConfig.CHIP_TYPE = SX1268_CHIP;
    hwConfig.PIN_LORA_RESET = PIN_SLORA_RESET;      // LORA RESET
    hwConfig.PIN_LORA_NSS = PIN_SLORA_SELECT;       // LORA SPI CS
    hwConfig.PIN_LORA_SCLK = PIN_SLORA_SCLK;        // LORA SPI CLK
    hwConfig.PIN_LORA_MISO = PIN_SLORA_MISO;        // LORA SPI MISO
    hwConfig.PIN_LORA_DIO_1 = PIN_SLORA_INTERRUPT;  // LORA DIO_1
    hwConfig.PIN_LORA_BUSY = PIN_SLORA_BUSY;        // LORA SPI BUSY
    hwConfig.PIN_LORA_MOSI = PIN_SLORA_MOSI;        // LORA SPI MOSI
    hwConfig.RADIO_TXEN = PIN_RADIO_TXEN;           // LORA ANTENNA TX ENABLE
    hwConfig.RADIO_RXEN = PIN_RADIO_RXEN;           // LORA ANTENNA RX ENABLE

    // Example uses an CircuitRocks Alora RFM1262 which uses DIO2
    // pins as antenna control
    hwConfig.USE_DIO2_ANT_SWITCH = false;

    // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to
    // control oscillator voltage
    hwConfig.USE_DIO3_TCXO = false;

    // Only Insight ISP4520 module uses DIO3 as antenna control
    hwConfig.USE_DIO3_ANT_SWITCH = false;

    // Get lora module ID
    uint8_t deviceId[8];
    BoardGetUniqueId(deviceId);

    lora_hardware_init(hwConfig);

    // Initialize the Radio callbacks
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    // Initialize the Radio
    Radio.Init(&RadioEvents);

    // Set Radio channel
    Radio.SetChannel(RF_FREQUENCY);

    // Set Radio TX configuration
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0,
                      0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

    // Set Radio RX configuration
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.Rx(RX_TIMEOUT_VALUE);
}

void LoraCommunication::update()
{
    if (!cadBusy && !q_tx.isEmpty()) {
        q_tx.pop(TxBuffer);
        Radio.Standby();
        Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10,
                           LORA_CAD_ONLY, 0);
        cadTime = millis();
        cadBusy = true;
        Radio.StartCad();
    }
    Radio.IrqProcess();
    delay(50);
    yield();
}

bool LoraCommunication::send(LoraPacket *p)
{
    uint8_t packet[LORA_PACKET_SIZE];
    p->serialize(packet);
    return q_tx.push(packet);
}

int LoraCommunication::available()
{
    return q_rx.getCount();
}

void OnTxDone(void)
{
    // Radio.Rx(RX_TIMEOUT_VALUE);
    lora.cadBusy = false;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    // TODO: check overflow error
    lora.q_rx.push(payload);
}

void OnCadDone(bool cadResult)
{
    if (cadResult) {
        // Cad busy
    } else {
        // Serial.println("Message sent!!!");
        // The first byte indicate the size of the packet (including itself)
        const uint8_t packet_size = (lora.TxBuffer[0] + 1 < LORA_PACKET_SIZE)
                                        ? lora.TxBuffer[0]
                                        : LORA_PACKET_SIZE;
        Radio.Send(lora.TxBuffer, packet_size);
    }
    // Serial.println("OnCadDone");
}

void OnTxTimeout(void)
{
    // Need to be logged
}

void OnRxTimeout(void) {}

void OnRxError(void) {}

#endif