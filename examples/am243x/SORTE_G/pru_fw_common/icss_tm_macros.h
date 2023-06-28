;************************************************************************************
;**+------------------------------------------------------------------------------+**
;**|                              ******                                          |**
;**|                              ******     o                                    |**
;**|                              *******__////__****                             |**
;**|                              ***** /_ //___/ ***                             |**
;**|                           ********* ////__ ******                            |**
;**|                             *******(_____/ ******                            |**
;**|                                 **********                                   |**
;**|                                   ******                                     |**
;**|                                      ***                                     |**
;**|                                                                              |**
;**|            Copyright (c) 2017 Texas Instruments Incorporated                 |**
;**|                           ALL RIGHTS RESERVED                                |**
;**|                                                                              |**
;**|    Permission is hereby granted to licensees of Texas Instruments            |**
;**|    Incorporated (TI) products to use this computer program for the sole      |**
;**|    purpose of implementing a licensee product based on TI products.          |**
;**|    No other rights to reproduce, use, or disseminate this computer           |**
;**|    program, whether in part or in whole, are granted.                        |**
;**|                                                                              |**
;**|    TI makes no representation or warranties with respect to the              |**
;**|    performance of this computer program, and specifically disclaims          |**
;**|    any responsibility for any damages, special or consequential,             |**
;**|    connected with the use of this program.                                   |**
;**|                                                                              |**
;**+------------------------------------------------------------------------------+**
;************************************************************************************
; file:     icss_tm_macros.h
;
; brief:    PRU Task Manager Related Macro definitions
;           Includes:
;           1.
;


    .if !$isdefed("____icss_tm_macros_h")
____icss_tm_macros_h   .set    1

    .include "pru_reg_structs.h"
    .include "icss_xfer_defines.h"

    ;Task IDs
TS1_S0  .set    0
TS1_S1  .set    1
TS1_S2  .set    2
TS1_S3  .set    3
TS1_S4  .set    4
TS2_S0  .set    5
TS2_S1  .set    6
TS2_S2  .set    7
TS2_S3  .set    8
TS2_S4  .set    9

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_rxtxmode_enable
;
;  Task Manager CFG for PRU to enable TS1_S{0 to 4} and TS2_S{0 to 4}
;
;   PEAK cycles:
;       5 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write to PRU0_TM_GLOBAL_CFG register to enable following
;           ts1_en_s0
;           ts1_en_s1
;           ts1_en_s2
;           ts1_en_s3
;           ts1_en_s4
;           ts2_en_s0
;           ts2_en_s1
;           ts2_en_s2
;           ts2_en_s3
;           ts2_en_s4
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 2 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_rxtxmode_enable .macro TEMP_REG, TM_BASE
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S4_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S4_SHIFT)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_rxtxmode_ts1_pc_set
;
;  Task Manager set the PC values for TS1_S{0 to 4}
;
;   PEAK cycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write ts1_pc_s0 to TS1_PC_S0 register
;       Write ts1_pc_s1 to TS1_PC_S1 register
;       Write ts1_pc_s2 to TS1_PC_S2 register
;       Write ts1_pc_s3 to TS1_PC_S3 register
;       Write ts1_pc_s4 to TS1_PC_S4 register
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 4 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts1_pc_s0 -> Program counter value for TS1_S0
;       ts1_pc_s1 -> Program counter value for TS1_S1
;       ts1_pc_s2 -> Program counter value for TS1_S2
;       ts1_pc_s3 -> Program counter value for TS1_S3
;       ts1_pc_s4 -> Program counter value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_rxtxmode_ts1_pc_set .macro TEMP_REG, TM_BASE, ts1_pc_s0, ts1_pc_s1, ts1_pc_s2, ts1_pc_s3, ts1_pc_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, $CODE(ts1_pc_s0)
    ldi TEMP_REG.reg3.x, $CODE(ts1_pc_s1)
    ldi TEMP_REG.reg4.x, $CODE(ts1_pc_s2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S0, 12
    ldi TEMP_REG.reg2.x, $CODE(ts1_pc_s3)
    ldi TEMP_REG.reg3.x, $CODE(ts1_pc_s4)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_rxtxmode_ts2_pc_set
;
;  Task Manager set the PC values for TS2_S{0 to 4}
;
;   PEAK cycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       TEMP_REG -> Register Structure with atleast 4 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       Write ts2_pc_s0 to TS2_PC_S0 register
;       Write ts2_pc_s1 to TS2_PC_S1 register
;       Write ts2_pc_s2 to TS2_PC_S2 register
;       Write ts2_pc_s3 to TS2_PC_S3 register
;       Write ts2_pc_s4 to TS2_PC_S4 register
;
;   Parameters:
;       ts2_pc_s0 -> Program counter value for TS2_S0
;       ts2_pc_s1 -> Program counter value for TS2_S1
;       ts2_pc_s2 -> Program counter value for TS2_S2
;       ts2_pc_s3 -> Program counter value for TS2_S3
;       ts2_pc_s4 -> Program counter value for TS2_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_rxtxmode_ts2_pc_set .macro TEMP_REG, TM_BASE, ts2_pc_s0, ts2_pc_s1, ts2_pc_s2, ts2_pc_s3, ts2_pc_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, $CODE(ts2_pc_s0)
    ldi TEMP_REG.reg3.x, $CODE(ts2_pc_s1)
    ldi TEMP_REG.reg4.x, $CODE(ts2_pc_s2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S0, 12
    ldi TEMP_REG.reg2.x, $CODE(ts2_pc_s3)
    ldi TEMP_REG.reg3.x, $CODE(ts2_pc_s4)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_enable
;
;  Task Manager CFG for PRU to setup TM in General Purpose Mode.
;
;   PEAK cycles:
;       5 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       2 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write to PRU0_TM_GLOBAL_CFG register to set tasks_mgr_mode to 2.
;       Also enables all subtasks.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 2 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_enable .macro TEMP_REG, TM_BASE
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, (0x2 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TASKS_MGR_MODE_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS1_EN_S4_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S0_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S1_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S2_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S3_SHIFT) | (0x1 << CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG_TS2_EN_S4_SHIFT)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_pc_set
;
;  Task Manager set the PC value for given task.
;
;   PEAK cycles:
;       7 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write ts_pc to register for ts_number.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts_pc -> Program counter value for task
;       ts_number -> Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_pc_set .macro TEMP_REG, TM_BASE, ts_pc, ts_number
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, $CODE(ts_pc)
    ldi TEMP_REG.reg3.x, (ts_number << 2)
    add TEMP_REG.reg3.x, TEMP_REG.reg3.x, 8
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, TEMP_REG.reg3.x, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_ts1_pc_set
;
;  Task Manager set the PC values for TS1_S{0 to 4}
;
;   PEAK cycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write ts1_pc_s0 to TS1_PC_S0 register
;       Write ts1_pc_s1 to TS1_PC_S1 register
;       Write ts1_pc_s2 to TS1_PC_S2 register
;       Write ts1_pc_s3 to TS1_PC_S3 register
;       Write ts1_pc_s4 to TS1_PC_S4 register
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 4 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts1_pc_s0 -> Program counter value for TS1_S0
;       ts1_pc_s1 -> Program counter value for TS1_S1
;       ts1_pc_s2 -> Program counter value for TS1_S2
;       ts1_pc_s3 -> Program counter value for TS1_S3
;       ts1_pc_s4 -> Program counter value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_ts1_pc_set .macro TEMP_REG, TM_BASE, ts1_pc_s0, ts1_pc_s1, ts1_pc_s2, ts1_pc_s3, ts1_pc_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, $CODE(ts1_pc_s0)
    ldi TEMP_REG.reg3.x, $CODE(ts1_pc_s1)
    ldi TEMP_REG.reg4.x, $CODE(ts1_pc_s2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S0, 12
    ldi TEMP_REG.reg2.x, $CODE(ts1_pc_s3)
    ldi TEMP_REG.reg3.x, $CODE(ts1_pc_s4)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_ts2_pc_set
;
;  Task Manager set the PC values for TS2_S{0 to 4}
;
;   PEAK cycles:
;       14 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       4 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       TEMP_REG -> Register Structure with atleast 4 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       Write ts2_pc_s0 to TS2_PC_S0 register
;       Write ts2_pc_s1 to TS2_PC_S1 register
;       Write ts2_pc_s2 to TS2_PC_S2 register
;       Write ts2_pc_s3 to TS2_PC_S3 register
;       Write ts2_pc_s4 to TS2_PC_S4 register
;
;   Parameters:
;       ts2_pc_s0 -> Program counter value for TS2_S0
;       ts2_pc_s1 -> Program counter value for TS2_S1
;       ts2_pc_s2 -> Program counter value for TS2_S2
;       ts2_pc_s3 -> Program counter value for TS2_S3
;       ts2_pc_s4 -> Program counter value for TS2_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_ts2_pc_set .macro TEMP_REG, TM_BASE, ts2_pc_s0, ts2_pc_s1, ts2_pc_s2, ts2_pc_s3, ts2_pc_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    ldi TEMP_REG.reg2.x, $CODE(ts2_pc_s0)
    ldi TEMP_REG.reg3.x, $CODE(ts2_pc_s1)
    ldi TEMP_REG.reg4.x, $CODE(ts2_pc_s2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S0, 12
    ldi TEMP_REG.reg2.x, $CODE(ts2_pc_s3)
    ldi TEMP_REG.reg3.x, $CODE(ts2_pc_s4)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_PC_S3, 8
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_disable_task
;
;  Disable given task in Task Manager.
;
;   PEAK cycles:
;       8 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Disable given task (ts_number) in Task Manager.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts_number -> Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_disable_task .macro TEMP_REG, TM_BASE, ts_number
    ldi32 TEMP_REG.reg1.x, TM_BASE
    lbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    clr TEMP_REG.reg2.x, TEMP_REG.reg2.x, (ts_number + 2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_enable_task
;
;  Enable given task in Task Manager.
;
;   PEAK cycles:
;       8 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Enable given task (ts_number) in Task Manager.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts_number -> Task number (TS1_S0 to TS2_S4)
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_enable_task .macro TEMP_REG, TM_BASE, ts_number
    ldi32 TEMP_REG.reg1.x, TM_BASE
    lbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    set TEMP_REG.reg2.x, TEMP_REG.reg2.x, (ts_number + 2)
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_GLOBAL_CFG, 4
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_ts1_mux_set
;
;  Task Manager CFG for PRU to setup TM GP Mode Mux for TS1.
;
;   PEAK cycles:
;       11 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write to TS1_GEN_CFG1 and TS1_GEN_CFG2 register to setup GP mode mux for TS1.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts1_mx_s0 -> Mux value for TS1_S0
;       ts1_mx_s1 -> Mux value for TS1_S1
;       ts1_mx_s2 -> Mux value for TS1_S2
;       ts1_mx_s3 -> Mux value for TS1_S3
;       ts1_mx_s4 -> Mux value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_ts1_mux_set .macro TEMP_REG, TM_BASE, ts1_mx_s0, ts1_mx_s1, ts1_mx_s2, ts1_mx_s3, ts1_mx_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    zero &TEMP_REG.reg2.x, 8
    or TEMP_REG.reg2.x_b.b0, TEMP_REG.reg2.x_b.b0, ts1_mx_s0
    or TEMP_REG.reg2.x_b.b1, TEMP_REG.reg2.x_b.b1, ts1_mx_s1
    or TEMP_REG.reg2.x_b.b2, TEMP_REG.reg2.x_b.b2, ts1_mx_s2
    or TEMP_REG.reg2.x_b.b3, TEMP_REG.reg2.x_b.b3, ts1_mx_s3
    or TEMP_REG.reg3.x_b.b0, TEMP_REG.reg3.x_b.b0, ts1_mx_s4
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS1_GEN_CFG1, 5
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_set_cfg_gpmode_ts2_mux_set
;
;  Task Manager CFG for PRU to setup TM GP Mode Mux for TS2.
;
;   PEAK cycles:
;       11 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       3 Registers required in TEMP_REG defined by user
;
;   Pseudo code:
;       Write to TS2_GEN_CFG1 and TS2_GEN_CFG2 register to setup GP mode mux for TS2.
;
;   Parameters:
;       TEMP_REG -> Register Structure with atleast 3 Registers
;       TM_BASE -> Base address of the corresponding PRU's task manager
;       ts2_mx_s0 -> Mux value for TS1_S0
;       ts2_mx_s1 -> Mux value for TS1_S1
;       ts2_mx_s2 -> Mux value for TS1_S2
;       ts2_mx_s3 -> Mux value for TS1_S3
;       ts2_mx_s4 -> Mux value for TS1_S4
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_set_cfg_gpmode_ts2_mux_set .macro TEMP_REG, TM_BASE, ts2_mx_s0, ts2_mx_s1, ts2_mx_s2, ts2_mx_s3, ts2_mx_s4
    ldi32 TEMP_REG.reg1.x, TM_BASE
    zero &TEMP_REG.reg2.x, 8
    or TEMP_REG.reg2.x_b.b0, TEMP_REG.reg2.x_b.b0, ts2_mx_s0
    or TEMP_REG.reg2.x_b.b1, TEMP_REG.reg2.x_b.b1, ts2_mx_s1
    or TEMP_REG.reg2.x_b.b2, TEMP_REG.reg2.x_b.b2, ts2_mx_s2
    or TEMP_REG.reg2.x_b.b3, TEMP_REG.reg2.x_b.b3, ts2_mx_s3
    or TEMP_REG.reg3.x_b.b0, TEMP_REG.reg3.x_b.b0, ts2_mx_s4
    sbbo &TEMP_REG.reg2.x, TEMP_REG.reg1.x, CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_TS2_GEN_CFG1, 5
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield
;
;  Current Task yield to Task Manager
;
;   PEAK cycles:
;       4 cycles
;
;   Invokes:
;       None
;
;   Registers:
;       R0.b3
;
;   Pseudo code:
;       xin to TM_YIELD_XID with 1 byte
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield .macro
    xin TM_YIELD_XID, &R0.b3,1
deadloop_tm_yield?:
    jmp deadloop_tm_yield?
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield_only
;
;  Current Task yield to Task Manager
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0.b3
;
;   Pseudo code:
;       xin to TM_YIELD_XID with 1 byte
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield_only .macro
    xin TM_YIELD_XID, &R0.b3,1
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_yield_self_loop
;
;  Deadloop during Task yield to Task Manager
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       Deadloop.
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_yield_self_loop .macro
deadloop_tm_yield?:
    jmp deadloop_tm_yield?
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_enable
;
;  Task Manager enable
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       A new instruction "TSEN 1" is defined for Task Manager Enable
;       Using .word as a workaround because CLPRU doesnt support TSEN instruction
;
;   Parameters:
;       None
;
;   Returns:
;      None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_enable .macro
    ;.word 0x32800000
    tsen 1
    .endm


;******************************************************************************
;
;   Macro: m_pru_tm_disable
;
;  Task Manager disable
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       None
;
;   Pseudo code:
;       A new instruction "TSEN 0" is defined for Task Manager Disable
;       Using .word as a workaround because CLPRU doesnt support TSEN instruction
;
;   Parameters:
;       None
;
;   Returns:
;       None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_disable .macro
     ;.word 0x32000000
     tsen 0
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_context_save
;
;  Save previous task's context (R0-R29) to Scratch PAD.
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0
;
;   Pseudo code:
;       xout to TM_SAVE_XID with 120 bytes
;
;   Parameters:
;       None
;
;   Returns:
;None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_context_save .macro bank_id
    xout bank_id, &R0, 120
    .endm

;******************************************************************************
;
;   Macro: m_pru_tm_context_restore
;
;  Restore previous task's context (R0-R29) to Scratch PAD.
;
;   PEAK cycles:
;       1 cycle
;
;   Invokes:
;       None
;
;   Registers:
;       R0
;
;   Pseudo code:
;       xin 120 bytes from TM_SAVE_XID SPAD bank
;
;   Parameters:
;       None
;
;   Returns:
;None
;
;   See Also:
;
;******************************************************************************
m_pru_tm_context_restore .macro bank_id
    xin bank_id, &R0, 120
    .endm

    .endif ; ____icss_tm_macros_h
