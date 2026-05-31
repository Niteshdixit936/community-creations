#ifndef TELEMETRY_H
#define TELEMETRY_H

/* ================================================================
   telemetry.h  â€“  BLE / Zigbee telemetry + mesh node sharing
   Sends JSON packets over USART1 to EFR32 BLE radio subsystem.
   For Zigbee mesh, the same UART bridge is used with a different
   packet type byte.
   ================================================================ */
#include "sensors.h"
#include <stdbool.h>

/* Packet type IDs */
#define PKT_TELEMETRY   0x01
#define PKT_MESH_TARGET 0x02
#define PKT_HEARTBEAT   0x03

typedef struct {
    uint8_t  node_id;
    float    az_deg;
    float    el_deg;
    float    dist_cm;
    float    speed_kmh;
    uint8_t  confidence;
    bool     laser_active;
    float    mcu_temp_c;
} TelemetryPacket;

void telemetry_init(void);

/* Send telemetry JSON over BLE UART */
void telemetry_send(const Target *t, bool laser_on, float temp_c);

/* Broadcast target over Zigbee mesh to peer nodes */
void mesh_broadcast_target(const Target *t);

/* Poll for incoming mesh target from peer nodes.
   Returns true and fills *out if a valid packet arrived.         */
bool mesh_receive_target(Target *out);

#endif /* TELEMETRY_H */
