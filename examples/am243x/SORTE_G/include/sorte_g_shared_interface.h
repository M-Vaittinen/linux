#ifndef __SORTE_G_SHARED_INTERFACE_H__
#define __SORTE_G_SHARED_INTERFACE_H__


//----------------------------------------------------
//      ISR Service ID
//----------------------------------------------------

#define IRQ_SVC_ID_OFFSET                       0x0200

#define IRQ_SVC_RESET                           0x01    //  << 0

#define IRQ_SVC_MASTER_PORT_LINK_UP             0x02    //  << 1
#define IRQ_SVC_MASTER_PORT_LINK_DOWN           0x04    //  << 2

#define IRQ_SVC_SLAVE_PORT_LINK_UP              0x08    //  << 3
#define IRQ_SVC_SLAVE_PORT_LINK_DOWN            0x10    //  << 4

#endif // __SORTE_G_SHARED_INTERFACE_H__
