/* ================================================================
   telemetry.c  â€“  BLE UART telemetry + Zigbee mesh relay
   ================================================================ */
#include "telemetry.h"
#include "config.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
 
/* Binary frame: [0xAA][type][len][payload...][CRC8] */
#define FRAME_SOF   0xAA

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}
 
static void uart_send_byte(uint8_t b)
{
    while (!(BLE_UART->STATUS & USART_STATUS_TXBL));
    BLE_UART->TXDATA = b;
}

static void uart_send_buf(const uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) uart_send_byte(buf[i]);
}
 
void telemetry_init(void)
{
    CMU_ClockEnable(cmuClock_USART1, true);

    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
    init.baudrate = 115200;
    USART_InitAsync(BLE_UART, &init);

    GPIO_PinModeSet(BLE_TX_PORT, BLE_TX_PIN, gpioModePushPull,       1);
    GPIO_PinModeSet(BLE_RX_PORT, BLE_RX_PIN, gpioModeInputPullFilter, 1);

    GPIO->USARTROUTE[1].TXROUTE = (BLE_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                | (BLE_TX_PIN  << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[1].RXROUTE = (BLE_RX_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                | (BLE_RX_PIN  << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[1].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN
                                | GPIO_USART_ROUTEEN_RXPEN;
}
 
void telemetry_send(const Target *t, bool laser_on, float temp_c)
{
    char json[180];
    int  n = snprintf(json, sizeof(json),
        "{\"node\":%u,\"az\":%.1f,\"el\":%.1f,"
        "\"dist\":%.0f,\"spd\":%.1f,\"conf\":%u,"
        "\"laser\":%u,\"tmp\":%.1f,\"valid\":%u}\n",
        NODE_ID,
        t->valid ? t->az_deg : 0.0f,
        t->valid ? t->el_deg : 0.0f,
        t->valid ? t->dist_cm : 0.0f,
        t->speed_kmh,
        t->confidence,
        laser_on ? 1u : 0u,
        temp_c,
        t->valid ? 1u : 0u);

    /* Frame: SOF | PKT_TELEMETRY | len | json | crc */
    uint8_t header[3] = { FRAME_SOF, PKT_TELEMETRY, (uint8_t)n };
    uart_send_buf(header, 3);
    uart_send_buf((const uint8_t *)json, (uint8_t)n);
    uart_send_byte(crc8((const uint8_t *)json, (uint8_t)n));
}
 
#pragma pack(push,1)
typedef struct {
    uint8_t  src_node;
    float    az_deg;
    float    el_deg;
    float    dist_cm;
    float    speed_kmh;
    uint8_t  confidence;
} MeshTargetPayload;
#pragma pack(pop)

void mesh_broadcast_target(const Target *t)
{
    if (!t->valid) return;

    MeshTargetPayload p = {
        .src_node   = NODE_ID,
        .az_deg     = t->az_deg,
        .el_deg     = t->el_deg,
        .dist_cm    = t->dist_cm,
        .speed_kmh  = t->speed_kmh,
        .confidence = t->confidence
    };

    uint8_t len = sizeof(p);
    uint8_t header[3] = { FRAME_SOF, PKT_MESH_TARGET, len };
    uart_send_buf(header, 3);
    uart_send_buf((const uint8_t *)&p, len);
    uart_send_byte(crc8((const uint8_t *)&p, len));
}
 
#define RX_BUF_SIZE 64
static uint8_t s_rxbuf[RX_BUF_SIZE];
static uint8_t s_rxidx = 0;

bool mesh_receive_target(Target *out)
{
    /* Non-blocking: drain available RX bytes */
    while (BLE_UART->STATUS & USART_STATUS_RXDATAV) {
        uint8_t b = (uint8_t)BLE_UART->RXDATA;
        if (s_rxidx == 0 && b != FRAME_SOF) continue;  /* wait for SOF */
        if (s_rxidx < RX_BUF_SIZE) s_rxbuf[s_rxidx++] = b;

        /* Check if we have a complete mesh target frame */
        uint8_t expected = 3 + sizeof(MeshTargetPayload) + 1;
        if (s_rxidx >= expected &&
            s_rxbuf[0] == FRAME_SOF &&
            s_rxbuf[1] == PKT_MESH_TARGET) {

            uint8_t len    = s_rxbuf[2];
            uint8_t rx_crc = s_rxbuf[3 + len];
            uint8_t calc   = crc8(&s_rxbuf[3], len);

            if (rx_crc == calc) {
                MeshTargetPayload *p = (MeshTargetPayload *)&s_rxbuf[3];
                /* Ignore own broadcasts */
                if (p->src_node != NODE_ID) {
                    out->az_deg     = p->az_deg;
                    out->el_deg     = p->el_deg;
                    out->dist_cm    = p->dist_cm;
                    out->speed_kmh  = p->speed_kmh;
                    out->confidence = p->confidence;
                    out->valid      = (p->confidence >= THREAT_CONFIRM_N);
                    s_rxidx = 0;
                    return true;
                }
            }
            s_rxidx = 0;  /* discard corrupted frame */
        }
    }
    return false;
}
