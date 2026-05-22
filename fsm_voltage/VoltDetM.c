/*****************************************************************************//**
 * @file VoltDetM.c
 * @brief Experimental code for new High/Low Voltage Detection management module
 * @copyright Copyright (c) 2023 HSAE co. Ltd.
 * @details
 * #### Changelog
 * |    Date    | Ver | Author(s) | Description |
 * | ---------- | --- | --------- | ----------- |
 * | 2023/03/21 | 0.1 |    Xya    | First draft version |
 * | 2023/03/31 | 0.2 |    Xya    | Improved calculation algorithm. Filter logic added. |
 * | 2023/04/07 | 0.2 |    Xya    | Doxygen style comment added. |
 * | 2026/05/15 | 2.0 |    Xya    | Adapt to BTET95xD platform. |
 *//*****************************************************************************/
//*****************************************************************************
//
//! @addtogroup VoltDetM
//! @brief High/Low Voltage Detection management module
//! @{
//
//*****************************************************************************

/*============================================================================
*                                        INCLUDE FILES
==============================================================================*/
#include "VoltDetM.h"
#include "HwIOAbs.h"

extern void HwIOAbs_GetAdcData(const IdtAdcChannelType Channel, uint16* Data);

/*============================================================================
*                                       DEFINES AND MACROS
==============================================================================*/

#define VOLTDETM_USE_FLOAT  STD_ON


/* Set Trigger voltage ADC value for entering each mode */
/* Calculated by following equation : 
   ADC_Value = (VBatt * (20 / (270 + 20))) * 4095 / 1.8  */

#if (VOLTDETM_USE_FLOAT == STD_ON)

#define ENTER_CRIT_LOW_VOLTAGE 6.0F	   //ill off //Enter critically low voltage state when battery voltage is below 6.0V 
#define EXIT_CRIT_LOW_VOLTAGE  6.5F    //ill off //Exit critically low voltage state when battery voltage is above 6.5V

#define ENTER_LOW_VOLTAGE 9.0F  //Enter low voltage state when battery voltage is below 9V
#define EXIT_LOW_VOLTAGE  9.5F  //Exit low voltage state when battery voltage is above 9.5V

#define ENTER_HIGH_VOLTAGE 16.0F  //ill off//Enter high voltage state when battery voltage is above 16.0V
#define EXIT_HIGH_VOLTAGE  15.5F  //Exit high voltage state when battery voltage is below 16.5V

#define ENTER_CRIT_HIGH_VOLTAGE 18.0F  //ill off//Enter critically high voltage state when battery voltage is above 18.0V
#define EXIT_CRIT_HIGH_VOLTAGE  17.5F  //ill off//Exit critically high voltage state when battery voltage is below 17.5V

#else

#define ENTER_CRIT_LOW_VOLTAGE 860U	   //943U  //ill off //Enter critically low voltage state when battery voltage is below 6.0V 
#define EXIT_CRIT_LOW_VOLTAGE  940U    //1020U //ill off //Exit critically low voltage state when battery voltage is above 6.5V

#define ENTER_LOW_VOLTAGE 1347U  //Enter low voltage state when battery voltage is below 9V
#define EXIT_LOW_VOLTAGE  1399U  //Exit low voltage state when battery voltage is above 9.5V

#define ENTER_HIGH_VOLTAGE 2535U  //ill off//Enter high voltage state when battery voltage is above 16.5V
#define EXIT_HIGH_VOLTAGE  2455U  //Exit high voltage state when battery voltage is below 16.0V

#define ENTER_CRIT_HIGH_VOLTAGE 2856U  //ill off//Enter critically high voltage state when battery voltage is above 18.5V
#define EXIT_CRIT_HIGH_VOLTAGE  2777U  //ill off//Exit critically high voltage state when battery voltage is below 18.0V

#endif

/* Convert ADC counts to the measured voltage at the resistor divider input. */
#define GET_ADC2VOLT_COEFF(r1, r2) ((float32)(((1.8F / 4095.0F) / ((float32)(r1) / ((float32)(r1) + (float32)(r2))))))

/* Battery voltage scaling factor for the 20 ohm / 270 ohm divider. */
#define BATT_ADC_TO_VOLT_COEFF GET_ADC2VOLT_COEFF(20U, 270U)

#define VOLT_VALUE_INVALID_FLOAT 0.0F
#define VOLT_VALUE_INVALID_ADC 0x0U

#ifndef LOG_W
#define LOG_W(...) ((void)0)
#endif

/* Debug define */
#ifdef __LOCAL_DEBUG_MODE_MY__
//#define ADC_VALUE_DEBUG
#endif

/*============================================================================
*                                       LOCAL TYPEDEFS
==============================================================================*/

/**
 * @brief State Machine State storage structure typedef
 * 
 * @details This structure stores state machine's state status for VoltDetM_Manage() 
 *          to use. Including Current state, Next state, and Last state.
 */
typedef struct VoltDetMStateMachineStateStorageStructure
{
    VoltDetMState Current; /**< Current State*/
    VoltDetMState Next; /**< Next State, the state that will be transitoined into*/
    VoltDetMState Last; /**< Last State, the previous state before transitioned into this one*/
} VoltDetManage_t;


/*============================================================================
*                                    FUNCTION PROTOTYPES
==============================================================================*/

// extern Std_ReturnType Rte_Call_CtApAdas_PiIpcSendData_IpcSendData(const IdtIpcHandle handel, uint8* pData, const uint8 len);

static void VoltDetM_Init_Entry(void);
static void VoltDetM_Init_OnDo(void);
static VoltDetMState VoltDetM_Init_Transition(VoltDetMState Sta);
static void VoltDetM_Init_Exit(void);

static void VoltDetM_CritLowVolt_Entry(void);
static void VoltDetM_CritLowVolt_OnDo(void);
static VoltDetMState VoltDetM_CritLowVolt_Transition(VoltDetMState Sta);
static void VoltDetM_CritLowVolt_Exit(void);

static void VoltDetM_LowVolt_Entry(void);
static void VoltDetM_LowVolt_OnDo(void);
static VoltDetMState VoltDetM_LowVolt_Transition(VoltDetMState Sta);
static void VoltDetM_LowVolt_Exit(void);

static void VoltDetM_NormalVolt_Entry(void);
static void VoltDetM_NormalVolt_OnDo(void);
static VoltDetMState VoltDetM_NormalVolt_Transition(VoltDetMState Sta);
static void VoltDetM_NormalVolt_Exit(void);

static void VoltDetM_HighVolt_Entry(void);
static void VoltDetM_HighVolt_OnDo(void);
static VoltDetMState VoltDetM_HighVolt_Transition(VoltDetMState Sta);
static void VoltDetM_HighVolt_Exit(void);

static void VoltDetM_CritHighVolt_Entry(void);
static void VoltDetM_CritHighVolt_OnDo(void);
static VoltDetMState VoltDetM_CritHighVolt_Transition(VoltDetMState Sta);
static void VoltDetM_CritHighVolt_Exit(void);

static void VoltDetM_Manage(void);

static void VoltDetM_SetState(VoltDetMState Sta);

static void Send_VoltDet_IPC_Msg(void);

/* TODO Remove after next davinci integration */
extern void SocSM_Get_ForceSyncTimer(uint16 * const Timer_val);

static uint16_t Get_Volt_ADC(void);
static inline float32 Convert_Batt_ADC_To_Volt(uint16_t ADC_Value);

/*============================================================================
*                                      LOCAL VARIABLES
==============================================================================*/

/**
 * @brief Actual State Machine State storage structure
 */
static VoltDetManage_t VoltDetMState_storage = 
{
    VOLT_STATE_MAX,
    VOLT_STATE_MAX,
    VOLT_STATE_MAX,
};

/**
 * @brief Actual structure storing every states' handler functions
 */
static const VoltDetM_Handler_t VoltDetM_StateHandler[VOLT_STATE_MAX] = 
{
    {&VoltDetM_Init_Entry,         &VoltDetM_Init_OnDo,         &VoltDetM_Init_Transition,         &VoltDetM_Init_Exit        },
    {&VoltDetM_CritLowVolt_Entry,  &VoltDetM_CritLowVolt_OnDo,  &VoltDetM_CritLowVolt_Transition,  &VoltDetM_CritLowVolt_Exit },
    {&VoltDetM_LowVolt_Entry,      &VoltDetM_LowVolt_OnDo,      &VoltDetM_LowVolt_Transition,      &VoltDetM_LowVolt_Exit     },
    {&VoltDetM_NormalVolt_Entry,   &VoltDetM_NormalVolt_OnDo,   &VoltDetM_NormalVolt_Transition,   &VoltDetM_NormalVolt_Exit  },
    {&VoltDetM_HighVolt_Entry,     &VoltDetM_HighVolt_OnDo,     &VoltDetM_HighVolt_Transition,     &VoltDetM_HighVolt_Exit    },
    {&VoltDetM_CritHighVolt_Entry, &VoltDetM_CritHighVolt_OnDo, &VoltDetM_CritHighVolt_Transition, &VoltDetM_CritHighVolt_Exit}
};

/**
 * @brief Variable storing RTE Type of VoltDetM state machine status
 */
static IdtVoltDetStaType Current_Volt_State = VOLTDETM_STATE_INIT;

static IdtVoltDetStaType Prev_Volt_State = VOLTDETM_STATE_INIT;

/**
 * @brief Array storing battery voltage value history for filtering use
 */
static uint16 Volt_history[10] = {0U};
 
/**
 * @brief Volt_history counter for filtering use
 */
static uint8 Volt_history_cnt = 0U;

/**
 * @brief Variable storing filtered stable Battery voltage value
 */
static uint16 Volt_ADC_stable = VOLT_VALUE_INVALID_ADC;

static volatile float32 Volt_float_stable = VOLT_VALUE_INVALID_FLOAT;

#if (VOLTDETM_USE_FLOAT == STD_ON)
#define VOLT_READOUT_STABLE Volt_float_stable
#else
#define VOLT_READOUT_STABLE Volt_ADC_stable
#endif

/*============================================================================
*                                      LOCAL FUNCTIONS
==============================================================================*/

/**
 * @brief Init state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_INIT
 */
static void VoltDetM_Init_Entry(void)
{
    Current_Volt_State = VOLTDETM_STATE_INIT;
}

/**
 * @brief Init state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Wait for the first time voltage 
 *          detection complete and transition to other state.
 */
static void VoltDetM_Init_OnDo(void)
{
    //nop
}

/**
 * @brief Init state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to corresponding state according to 
 *          value of Volt_ADC_stable variable. Stay in Init state 
 *          if the value is 0.
 */
static VoltDetMState VoltDetM_Init_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_INIT;

    (void)Sta;

    if (VOLT_READOUT_STABLE == 0U)
    {
        ret_val = VOLT_STATE_INIT;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_NORMAL;
    }
    else if (VOLT_READOUT_STABLE < ENTER_CRIT_LOW_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }
    else if ((VOLT_READOUT_STABLE < ENTER_LOW_VOLTAGE) && (VOLT_READOUT_STABLE >= ENTER_CRIT_LOW_VOLTAGE))
    {
        ret_val = VOLT_STATE_LOW;
    }
    else if (VOLT_READOUT_STABLE >= ENTER_CRIT_HIGH_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_HIGH_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_INIT;
    }

    return ret_val;
}

/**
 * @brief Init state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_Init_Exit(void)
{
    //nop
}


/**
 * @brief CritLowVolt state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_CRIT_LOW
 */
static void VoltDetM_CritLowVolt_Entry(void)
{
    LOG_W("VOLTDET", "Volt state now at CRIT LOW");
    Current_Volt_State = VOLTDETM_STATE_CRIT_LOW;
}

/**
 * @brief CritLowVolt state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do.
 */
static void VoltDetM_CritLowVolt_OnDo(void)
{
    //nop
}

/**
 * @brief CritLowVolt state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to VOLT_STATE_LOW if VOLT_READOUT_STABLE is higher than EXIT_CRIT_LOW_VOLTAGE, 
 *          otherwise Stay in CritLowVolt state.
 */
static VoltDetMState VoltDetM_CritLowVolt_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_CRIT_LOW;

    (void)Sta;

    if ((VOLT_READOUT_STABLE >= EXIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_NORMAL;
    }
    else if ((VOLT_READOUT_STABLE > EXIT_CRIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < EXIT_LOW_VOLTAGE))
    {
        ret_val = VOLT_STATE_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_HIGH_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }

    return ret_val;
}

/**
 * @brief CritLowVolt state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_CritLowVolt_Exit(void)
{
    //nop
}


/**
 * @brief LowVolt state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_LOW
 */
static void VoltDetM_LowVolt_Entry(void)
{
    LOG_W("VOLTDET", "Volt state now at LOW");
    Current_Volt_State = VOLTDETM_STATE_LOW;
}

/**
 * @brief LowVolt state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do.
 */
static void VoltDetM_LowVolt_OnDo(void)
{
    //nop
}

/**
 * @brief LowVolt state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to VOLT_STATE_NORMAL if VOLT_READOUT_STABLE is higher than EXIT_LOW_VOLTAGE. 
 *          Transition to VOLT_STATE_CRIT_LOW if VOLT_READOUT_STABLE is lower than ENTER_CRIT_LOW_VOLTAGE. 
 *          Otherwise Stay in LowVolt state.
 */
static VoltDetMState VoltDetM_LowVolt_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_LOW;

    (void)Sta;

    if (VOLT_READOUT_STABLE < ENTER_CRIT_LOW_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }
    else if ((VOLT_READOUT_STABLE > EXIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_NORMAL;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_HIGH_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_LOW;
    }

    return ret_val;
}

/**
 * @brief LowVolt state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_LowVolt_Exit(void)
{
    //nop
}


/**
 * @brief NormalVolt state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_NORMAL
 */
static void VoltDetM_NormalVolt_Entry(void)
{
    LOG_W("VOLTDET", "Volt state now at NORMAL");
    Current_Volt_State = VOLTDETM_STATE_NORMAL;
}

/**
 * @brief NormalVolt state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do.
 */
static void VoltDetM_NormalVolt_OnDo(void)
{
    //nop
}

/**
 * @brief NormalVolt state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to VOLT_STATE_HIGH if VOLT_READOUT_STABLE is higher than ENTER_HIGH_VOLTAGE. 
 *          Transition to VOLT_STATE_LOW if VOLT_READOUT_STABLE is lower than ENTER_LOW_VOLTAGE. 
 *          Otherwise Stay in NormalVolt state.
 */
static VoltDetMState VoltDetM_NormalVolt_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_NORMAL;

    (void)Sta;

    if (VOLT_READOUT_STABLE < ENTER_CRIT_LOW_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_LOW_VOLTAGE))
    {
        ret_val = VOLT_STATE_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }
    else if ((VOLT_READOUT_STABLE > ENTER_HIGH_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_CRIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_NORMAL;
    }

    return ret_val;
}

/**
 * @brief NormalVolt state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_NormalVolt_Exit(void)
{
    //nop
}


/**
 * @brief HighVolt state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_HIGH
 */
static void VoltDetM_HighVolt_Entry(void)
{
    LOG_W("VOLTDET", "Volt state now at HIGH");
    Current_Volt_State = VOLTDETM_STATE_HIGH;
}

/**
 * @brief HighVolt state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do.
 */
static void VoltDetM_HighVolt_OnDo(void)
{
    //nop
}

/**
 * @brief HighVolt state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to VOLT_STATE_CRIT_HIGH if VOLT_READOUT_STABLE is higher than ENTER_CRIT_HIGH_VOLTAGE. 
 *          Transition to VOLT_STATE_NORMAL if VOLT_READOUT_STABLE is lower than EXIT_HIGH_VOLTAGE. 
 *          Otherwise Stay in HighVolt state.
 */
static VoltDetMState VoltDetM_HighVolt_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_HIGH;

    (void)Sta;

    if (VOLT_READOUT_STABLE < ENTER_CRIT_LOW_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_LOW_VOLTAGE))
    {
        ret_val = VOLT_STATE_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < EXIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_NORMAL;
    }
    else if (VOLT_READOUT_STABLE > ENTER_CRIT_HIGH_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_HIGH;
    }

    return ret_val;
}

/**
 * @brief HighVolt state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_HighVolt_Exit(void)
{
    //nop
}


/**
 * @brief CritHighVolt state Entry handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Set RTE VoltDetM Status to VOLTDETM_STATE_CRIT_HIGH
 */
static void VoltDetM_CritHighVolt_Entry(void)
{
    LOG_W("VOLTDET", "Volt state now at CRIT HIGH");
    Current_Volt_State = VOLTDETM_STATE_CRIT_HIGH;
}

/**
 * @brief CritHighVolt state OnDo handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do.
 */
static void VoltDetM_CritHighVolt_OnDo(void)
{
    //nop
}

/**
 * @brief CritHighVolt state Transition handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Transition to VOLT_STATE_HIGH if VOLT_READOUT_STABLE is lower than EXIT_CRIT_HIGH_VOLTAGE, 
 *          otherwise Stay in CritHighVolt state.
 */
static VoltDetMState VoltDetM_CritHighVolt_Transition(VoltDetMState Sta)
{
    VoltDetMState ret_val = VOLT_STATE_CRIT_HIGH;

    (void)Sta;

    if (VOLT_READOUT_STABLE < ENTER_CRIT_LOW_VOLTAGE)
    {
        ret_val = VOLT_STATE_CRIT_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_CRIT_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < ENTER_LOW_VOLTAGE))
    {
        ret_val = VOLT_STATE_LOW;
    }
    else if ((VOLT_READOUT_STABLE >= ENTER_LOW_VOLTAGE) && (VOLT_READOUT_STABLE < EXIT_HIGH_VOLTAGE))
    {
        ret_val = VOLT_STATE_NORMAL;
    }
    else if (VOLT_READOUT_STABLE < EXIT_CRIT_HIGH_VOLTAGE)
    {
        ret_val = VOLT_STATE_HIGH;
    }
    else
    {
        ret_val = VOLT_STATE_CRIT_HIGH;
    }

    return ret_val;
}

/**
 * @brief CritHighVolt state Exit handler function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Nothing to do. Still need this function to complete 
 *          the state machine logic.
 */
static void VoltDetM_CritHighVolt_Exit(void)
{
    //nop
}



/**
 * @brief VoltDetM module state management logic
 * 
 * @param None
 * 
 * @return None
 * 
 * @details This function is a simple realization of a finite state machine (FSM).
 * @details Use VoltDetMState_storage structure to store Previous, Current, and Next state.
 * @details Use VoltDetM_StateHandler structure to store every states' handler functions, 
 *          including Entry, OnDo, Transition, and Exit.
 * @details Logic : If Next state value is equal to Current state value, call current state's 
 *          OnDo handler funciton to do the work, then call Transition handler function 
 *          to determine which state should go next.
 *          If Next state value is not equal to Current state value, call current state's 
 *          Exit handler funciton to properly exit this state, then move Current state into 
 *          Last state, move Next state into Current state, and call newly entered state's 
 *          Entry handler function to prepare for transition into this state, then call 
 *          corresponding OnDo handler funciton and Transition handler function as mentioned before.
 * @details Called by VoltDetM_MainFunction()
 */
static void VoltDetM_Manage(void)
{
    if(VoltDetMState_storage.Next >= VOLT_STATE_MAX)
    {
        return;
    }

    if(VoltDetMState_storage.Current != VoltDetMState_storage.Next)
    {
        if (VoltDetMState_storage.Current < VOLT_STATE_MAX)
        {
            VoltDetM_StateHandler[VoltDetMState_storage.Current].Exit();
        }

        VoltDetMState_storage.Last = VoltDetMState_storage.Current;
        
        VoltDetMState_storage.Current = VoltDetMState_storage.Next;

        VoltDetM_StateHandler[VoltDetMState_storage.Current].Entry();
    }

    VoltDetM_StateHandler[VoltDetMState_storage.Current].OnDo();
    VoltDetMState_storage.Next = VoltDetM_StateHandler[VoltDetMState_storage.Current].Transition(VoltDetMState_storage.Current);
}

/**
 * @brief Set VoltDetM state machine state
 * 
 * @param Sta State need to be set to
 * 
 * @return None
 */
static void VoltDetM_SetState(VoltDetMState Sta)
{
    if (Sta < VOLT_STATE_MAX)
    {
        VoltDetMState_storage.Next = Sta;
    }
}

static uint16_t Get_Volt_ADC(void)
{
    uint16_t ADC_Value = 0U;
    HwIOAbs_GetAdcData(MCU_VBATT_CH, &ADC_Value);
    return ADC_Value;
}

static inline float32 Convert_Batt_ADC_To_Volt(uint16_t ADC_Value)
{
    return ((float32)ADC_Value * BATT_ADC_TO_VOLT_COEFF);
}

static void Update_Calculate_Volt_Stable(void)
{
    /* Median-filter parameters for Volt_history[] (array length = 10 samples)       */
    /* The 2 lowest and 2 highest sorted samples are discarded; the middle 6 are      */
    /* averaged. All derived constants follow from VOLT_HISTORY_SIZE – change only    */
    /* VOLT_HISTORY_SIZE and the rest update automatically.                           */
    #define VOLT_HISTORY_SIZE       10U                                        /* Total number of ADC samples in the history buffer      */
    #define VOLT_FILTER_SKIP        2U                                         /* Samples to discard at each end (lowest / highest)      */
    #define VOLT_FILTER_START_IDX   VOLT_FILTER_SKIP                           /* First index used in the average (= 2)                  */
    #define VOLT_FILTER_END_IDX     (VOLT_HISTORY_SIZE - VOLT_FILTER_SKIP)     /* One-past-last index used in the average (= 8)          */
    #define VOLT_FILTER_SAMPLE_CNT  (VOLT_FILTER_END_IDX - VOLT_FILTER_START_IDX) /* Number of samples averaged (= 6)                  */

    uint16 B_Volt_ADC = 0U;
    B_Volt_ADC = Get_Volt_ADC(); //Get voltage ADC readout

    if (Volt_history_cnt < VOLT_HISTORY_SIZE) //Buffer is not full
    {
        Volt_history[Volt_history_cnt] = B_Volt_ADC; //Put current readout into the buffer
        Volt_history_cnt++;
    }
    else  //Buffer is full
    {
        uint8  sort_i    = 0U;   /* Outer loop index for insertion sort          */
        uint8  sort_j    = 0U;   /* Inner (shift) index for insertion sort       */
        uint8  avg_k     = 0U;   /* Loop index for the averaging pass            */
        uint16 sort_key  = 0U;   /* Current element being inserted into position */
        uint32 calc_sum  = 0UL;  /* Running sum of the middle samples            */

        /* Insertion sort – sort Volt_history[] ascending in-place */
        for (sort_i = 1U; sort_i < VOLT_HISTORY_SIZE; sort_i++)
        {
            sort_key = Volt_history[sort_i];
            sort_j   = sort_i;
            while ((sort_j > 0U) && (Volt_history[sort_j - 1U] > sort_key))
            {
                Volt_history[sort_j] = Volt_history[sort_j - 1U];
                sort_j--;
            }
            Volt_history[sort_j] = sort_key;
        }

        /* Average the middle samples, discarding the 2 lowest and 2 highest */
        for (avg_k = VOLT_FILTER_START_IDX; avg_k < VOLT_FILTER_END_IDX; avg_k++)
        {
            calc_sum += (uint32)Volt_history[avg_k];
        }
        Volt_ADC_stable = (uint16)(calc_sum / VOLT_FILTER_SAMPLE_CNT);
        
        Volt_float_stable = Convert_Batt_ADC_To_Volt(Volt_ADC_stable);

        //Reset array pointer and put current readout into the buffer
        Volt_history_cnt = 0U;
        Volt_history[Volt_history_cnt] = B_Volt_ADC;
        Volt_history_cnt++;
    }
}

/*============================================================================
*                                    GLOBAL FUNCTIONS
==============================================================================*/

#ifdef ADC_VALUE_DEBUG
static uint16 test_volt_history[10] = {0};
static uint8 test_volt_history_cnt = 0U;

static void test_func(void)
{
    /* Convert ADC average value to the real rail voltage by divider ratio. */
    #define GET_REAL_COEFF(r1, r2)  ((float)(((1.8f / 4095.0f) / ((float)(r1) / ((float)(r1) + (float)(r2))))))

    enum
    {
        TEST_ADC_D5V,
        TEST_ADC_D3V3,
        TEST_ADC_D1V5,
        TEST_ADC_D0V9,
        TEST_ADC_D1V8,
        TEST_ADC_MAX
    };

    /* Keep 10 raw samples for each monitored rail. */
    static uint16 test_volt_storage[TEST_ADC_MAX][10] = {0};
    static uint8 test_volt_cnt[TEST_ADC_MAX] = {0};
    static uint32 test_volt_avg[TEST_ADC_MAX] = {0};

    /* Store the converted real voltage for debug observation. */
    static float test_volt_real[TEST_ADC_MAX] = {0.0};

    /* ADC channel mapping for each rail under test. */
    static const IdtAdcChannelType test_adc_channel[TEST_ADC_MAX] = {MCU_ADC_D5V_CH, MCU_ADC_D3V_CH, MCU_ADC_D1V5_CH, MCU_ADC_D0V9_CH, MCU_ADC_D1V8_CH};

    /* Per-rail ADC-to-voltage conversion factor. */
    static const float test_adc_real_factor[TEST_ADC_MAX] = 
    {
        GET_REAL_COEFF(10, 30), //D5V
        GET_REAL_COEFF(15, 30), //D3V3
        GET_REAL_COEFF(47, 20), //D1V5
        GET_REAL_COEFF(100, 10), //D0V9
        GET_REAL_COEFF(47, 30)  //D1V8
    };

    for (uint8 i = 0; i < TEST_ADC_MAX; i++)
    {
        if (test_volt_cnt[i] < 10)
        {
            /* Fill the sample window first. */
            HwIOAbs_GetAdcData(test_adc_channel[i], &test_volt_storage[i][test_volt_cnt[i]]);
            test_volt_cnt[i]++;
        }
        else
        {
            /* Restart the window and calculate the last 10-sample average. */
            test_volt_cnt[i] = 0U;
            test_volt_avg[i] = 0U;
            for (uint8 j = 0; j < 10; j++)
            {
                test_volt_avg[i] += test_volt_storage[i][j];
            }
            test_volt_avg[i] /= 10;
            /* Convert averaged ADC code to estimated real voltage. */
            test_volt_real[i] = (float)test_volt_avg[i] * test_adc_real_factor[i];
        }
    }
}

#endif

/**
 * @brief VoltDetM main function
 * 
 * @param None
 * 
 * @return None
 * 
 * @details Called by RTE every 20ms.
 * @details Update voltage conversion values via Volt_Mainfunction(). 
 *          Use median filtering to get a stable voltage readout.
 *          Transition to correct voltage status by calling VoltDetM_Manage().
 */
void VoltDetM_MainFunction(void)
{
    #ifdef ADC_VALUE_DEBUG
    uint16 Adc_data = 0U;
    
    HwAdc_GetAdcData(ADC0_Module,MCU_VBATT_ADC,&Adc_data);
    if (test_volt_history_cnt < 10)
    {
        test_volt_history[test_volt_history_cnt] = Adc_data;
        test_volt_history_cnt++;
    }
    else
    {
        test_volt_history_cnt = 0;
        test_volt_history[test_volt_history_cnt] = Adc_data;
        test_volt_history_cnt++;
    }

    test_func();
    
    #endif

    /* 
        Get current voltage ADC readout, put it in a buffer array.
        When buffer array reaches its end, use insertion sort to sort the array content,
        exclude biggest two numbers and smallest two numbers, then average the rest 6 numbers
        to get a stable readout. (Median filtering)
        This will make the voltage detection cycle effectively 200ms
    */
    Update_Calculate_Volt_Stable();

    //Excute voltage detection function after all voltage data are processed
    VoltDetM_Manage();

    #if 0
    IdtSocSMStaType SocSM_State = SOC_STATE_IDLE;
    uint16 VoltDet_ForceSync_Timer = 0U;
    static bool VoltDet_ForceSync_needed = false;

    Rte_Read_PiSocSMSta_Sta(&SocSM_State);

    /* TODO Replace with RTE interface after next davinci integration */
    SocSM_Get_ForceSyncTimer(&VoltDet_ForceSync_Timer);

    /* Send IPC to SoC when there is a state change */
    if (SocSM_State == SOC_STATE_NORMAL)
    {
        if (VoltDet_ForceSync_Timer > 52U)
        {
            if (VoltDet_ForceSync_needed != false)
            {
                Send_VoltDet_IPC_Msg();
                VoltDet_ForceSync_needed = false;
            }
        }
        else
        {
            VoltDet_ForceSync_needed = true;
        }

        if (Current_Volt_State != Prev_Volt_State)
        {
            Send_VoltDet_IPC_Msg();
            Prev_Volt_State = Current_Volt_State;
        }
    }
    #else
    if (Current_Volt_State != Prev_Volt_State)
    {
        Send_VoltDet_IPC_Msg();
        Prev_Volt_State = Current_Volt_State;
    }
    #endif //0
}

static void Send_VoltDet_IPC_Msg(void)
{
    if ((Current_Volt_State > VOLTDETM_STATE_INIT) && (Current_Volt_State <= VOLTDETM_STATE_CRIT_HIGH))
    {
        #if 1
        #pragma message ("Properly intergrate IPC send interface")
        #else
        Batt_Volt_State IPC_Msg_buffer = Batt_Volt_State_init_default;
        IPC_Msg_buffer.Cur_Batt_Volt_State = (uint8_t)Current_Volt_State;
        ipc_appl_send_with_variable_length(IPC_MSG_HANDLE_MTS_Batt_Volt_State, (uint8*)&IPC_Msg_buffer, (uint8)sizeof(IPC_Msg_buffer));
        #endif
    }
}

/**
 * @brief VoltDetM init function
 * 
 * @param None
 * 
 * @return None
 */
void VoltDetM_Init(void)
{
    VoltDetM_SetState(VOLT_STATE_INIT);
    memset(&Volt_history, 0, sizeof(Volt_history));
    Volt_history_cnt = 0U;
    Volt_ADC_stable = 0U;
    Volt_float_stable = 0.0F;
}

/**
 * @brief Provide other SWCs with current voltage status (called by RTE)
 * 
 * @param Sta Pointer to the address that status will be written to
 * 
 * @return None
 */
void VoltDet_GetVolSta(IdtVoltDetStaType *const Sta)
{
    *Sta = Current_Volt_State;
}

/**
 * @brief Provide other SWCs with current voltage status (called by RTE)
 * 
 * @param Sta Pointer to the address that status will be written to
 * 
 * @return None
 */
uint16 VoltDet_GetVolAD(void)
{
    return Volt_ADC_stable;
}

/**
 * @brief Get the current voltage float value
 * 
 * @return current voltage float value
 * 
 * @details This function returns the current voltage float value, which is calculated
 *          from the filtered stable ADC readout.
 * @details Note that 0.0F means invalid value, which can be used to indicate the voltage readout is not ready yet.
 */
float32 VoltDet_GetVolAD_Float(void)
{
    return Volt_float_stable;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
