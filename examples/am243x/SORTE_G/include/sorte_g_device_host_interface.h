#ifndef __SORTE_G_SLAVE_HOST_INTERFACE_H__
#define __SORTE_G_SLAVE_HOST_INTERFACE_H__

#include "sorte_g_shared_interface.h"

//INDATA -> device to controller data
#define DEVICE_INDATA_FRAME_BUFFER_PTR       0x0060      // 16-bit
#define DEVICE_INDATA_FRAME_BUFFER_SIZE      0x0062      // 16-bit
//OUTDATA -> controller to device data
#define DEVICE_OUTDATA_FRAME_BUFFER_PTR      0x0064      // 16-bit
#define DEVICE_OUTDATA_FRAME_BUFFER_SIZE     0x0066      // 16-bit
#define DEVICE_MASTER_FRAME_BUFFER_PTR       0x0068      // 16-bit

#define DEVICE_IOEX_DATA_FRAME_BUFFER_BASE       0x0000      //

#define DEVICE_INDATA_FRAME_BUFFER_LOCATION      0x0000      // INDATA -> slave to master data
#define DEVICE_OUTDATA_FRAME_BUFFER_LOCATION     0x0040      // OUTDATA -> master to slave data
#define DEVICE_MASTER_FRAME_BUFFER_LOCATION      0x0080      // frame buffer for the master frame

#define DEVICE_ADDR_OFFSET      0x0024      // .field8

//----------------------------------------------------
//      PARAM.field offsets
//----------------------------------------------------

#define SLAVE_PARAM_DATA_OFFSET           0x0100

//#define CYCLE_TIME                  0x00    // 32-bit
//#define SYNC_TIME                   ???   // 32-bit
//#define DELAY_PERIOD                ???   // 32-bit
//#define DELAY_BURST                 ???   // 8-bit
//#define DELAY_GAP                   ???   // 8-bit
//#define DELAY_FORWARD_TIME          ???   // 16-bit
//#define INTER_PACKET_GAP            ???   // 16-bit
//#define T_IN                        ???   // 16-bit
//#define T_OUT                       ???   // 16-bit
//#define TOPOLOGY                    ???   // 8-bit
//#define DIAGNOSTICS                 ???   // 8-bit
//#define ALARM                       ???   // 8-bit
//#define CRC_MODE                    ???   // 8-bit
#define BROADCAST_LENGTH            0x25    // 8-bit

#endif // __SORTE_G_SLAVE_HOST_INTERFACE_H__
