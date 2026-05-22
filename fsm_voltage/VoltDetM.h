/*****************************************************************************//**
 * @file VoltDetM.h
 * @brief Experimental header for new High/Low Voltage Detection management module
 * @copyright Copyright (c) 2023 HSAE co. Ltd.
 * @details
 * #### Changelog
 * |    Date    | Ver | Author(s) | Description |
 * | ---------- | --- | --------- | ----------- |
 * | 2023/03/21 | 0.1 |    Xya    | First draft version |
 *//*****************************************************************************/

#ifndef VOLTDETM_HEADER_1679369062
#define VOLTDETM_HEADER_1679369062

//*****************************************************************************
//
//! @addtogroup VoltDetM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================
*                                        INCLUDE FILES
==============================================================================*/

#include "Platform_Types.h"
#include "string.h"

/*============================================================================
*                                         TYPEDEFS
==============================================================================*/

typedef uint8 IdtVoltDetStaType;
#ifndef VOLTDETM_STATE_INIT
#define VOLTDETM_STATE_INIT ((IdtVoltDetStaType)0)
#endif /*VOLTDETM_STATE_INIT*/
#ifndef VOLTDETM_STATE_CRIT_LOW
#define VOLTDETM_STATE_CRIT_LOW ((IdtVoltDetStaType)1)
#endif /*VOLTDETM_STATE_CRIT_LOW*/
#ifndef VOLTDETM_STATE_LOW
#define VOLTDETM_STATE_LOW ((IdtVoltDetStaType)2)
#endif /*VOLTDETM_STATE_LOW*/
#ifndef VOLTDETM_STATE_NORMAL
#define VOLTDETM_STATE_NORMAL ((IdtVoltDetStaType)3)
#endif /*VOLTDETM_STATE_NORMAL*/
#ifndef VOLTDETM_STATE_HIGH
#define VOLTDETM_STATE_HIGH ((IdtVoltDetStaType)4)
#endif /*VOLTDETM_STATE_HIGH*/
#ifndef VOLTDETM_STATE_CRIT_HIGH
#define VOLTDETM_STATE_CRIT_HIGH ((IdtVoltDetStaType)5)
#endif /*VOLTDETM_STATE_CRIT_HIGH*/

/**
 * @brief Typedef enumeration of Voltage States
 */
typedef	enum _VoltDetMState
{
    VOLT_STATE_INIT, /**< Initial state, will transition to other state when Volt_stable has a valid value  */
    VOLT_STATE_CRIT_LOW, /**< Battery voltage critically low state, SoC will be shutdown and MCU will go to STR mode in this state */
    VOLT_STATE_LOW, /**< Battery voltage low state, screen will be turn off and Amp will be mute in this state */
    VOLT_STATE_NORMAL, /**< Battery voltage normal state, everything should be working fine */
    VOLT_STATE_HIGH, /**< Battery voltage high state, screen will be turn off and Amp will be mute in this state */
    VOLT_STATE_CRIT_HIGH, /**< Battery voltage critically high state, SoC will be shutdown and MCU will go to STR mode in this state */
    VOLT_STATE_MAX, /**< End of enumeration, not a legit state*/
}VoltDetMState;

/**
 * @brief State Machine's states' handler function storage structure typedef
 */
typedef struct VoltDetMStateMachineHandlerFunctionStorageStructure
{
    void (* const Entry)(void); /**< Entry handler function, called once when entering this state*/
    void (* const OnDo)(void); /**< OnDo handler function, called every time to do the needed work*/
    VoltDetMState (* const Transition)(VoltDetMState cur_st); /**< Transition handler function, called every time to determine which state should be transitioned into*/
    void (* const Exit)(void); /**< Exit handler function, called once when leaving this state*/
} VoltDetM_Handler_t;

/*============================================================================
*                                    FUNCTION PROTOTYPES
==============================================================================*/

extern void VoltDetM_Init(void);

extern void VoltDetM_MainFunction(void);

extern void VoltDet_GetVolSta(IdtVoltDetStaType *const Sta); //Please refer to VoltDetMState enum
extern uint16 VoltDet_GetVolAD(void);
extern float32 VoltDet_GetVolAD_Float(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif //VOLTDETM_HEADER_1679369062
