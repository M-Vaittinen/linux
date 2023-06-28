#ifndef __SORTE_G_MASTER_HOST_INTERFACE_H__
#define __SORTE_G_MASTER_HOST_INTERFACE_H__

#include "sorte_g_shared_interface.h"

//------------------------------------------------------------------------------
//                       PRU0 Data RAM defines
//------------------------------------------------------------------------------

#define MASTER_INDATA_FRAME_BUFFER_PTR        0x0060      // 16-bit
#define MASTER_OUTDATA_FRAME_BUFFER_PTR       0x0062      // 16-bit
#define MASTER_MASTER_FRAME_BUFFER_PTR        0x0064      // 16-bit

#define MASTER_MASTER_FRAME_BUFFER_LOCATION   0x0200      // frame buffer for the master frame
#define MASTER_OUTDATA_FRAME_BUFFER_LOCATION  0x0700      // OUTDATA -> master to slave data
#define MASTER_INDATA_FRAME_BUFFER_LOCATION   0x0800      // INDATA -> slave to master data


// use first 256 bytes for configuration and status registers
#define SORTE_REG_INTERFACE         0x0

#define STATUS_REG_OFFSET           0x0004

// reserved              .set    0x001C
#define MASTER_PARAM_DATA_OFFSET    0x0020
// reserved              .set    0x0040

#define Broadcast_Length    0x1E        //  .byte

//----------------------------------------------------
//      ISR Service ID
//----------------------------------------------------

#define IRQ_SVC_ID_OFFSET                       0x0200

#define IRQ_SVC_DISCOVERY_READY                 0x01
#define IRQ_SVC_LINK_UP                         0x02
#define IRQ_SVC_LINK_DOWN                       0x04

#endif // __SORTE_G_MASTER_HOST_INTERFACE_H__
