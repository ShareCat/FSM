// coffee_machine.c - 使用状态机框架的咖啡机实现

#include "fsm.h"
#include "hardware.h"  // LED、电机、传感器等硬件接口

// 声明状态机实例和各个状态
static FSM_t coffee_fsm;

// 前向声明所有状态（因为状态之间会互相引用）
extern State_t state_idle;
extern State_t state_heating;
extern State_t state_brewing;
extern State_t state_done;
extern State_t state_error;

/*========== 待机状态 ==========*/
static void Idle_OnEntry(void)
{
    LED_Green_On();
    Display_Show("Ready");
}

static void Idle_OnRun(void)
{
    if (Button_IsPressed(BTN_START)) {
        FSM_TransitionTo(&coffee_fsm, &state_heating);
    }
}

static void Idle_OnExit(void)
{
    LED_Green_Off();
}

State_t state_idle = {
    .name = "IDLE",
    .OnEntry = Idle_OnEntry,
    .OnRun = Idle_OnRun,
    .OnExit = Idle_OnExit
};

/*========== 加热状态 ==========*/
static uint32_t heat_timer;

static void Heating_OnEntry(void)
{
    LED_Red_On();
    Heater_On();
    heat_timer = 0;
    Display_Show("Heating...");
}

static void Heating_OnRun(void)
{
    heat_timer++;

    // 检查温度
    if (Sensor_GetTemp() >= 95) {
        FSM_TransitionTo(&coffee_fsm, &state_brewing);
        return;
    }

    // 检查缺水
    if (Sensor_GetWaterLevel() < 10) {
        FSM_TransitionTo(&coffee_fsm, &state_error);
        return;
    }

    // 检查取消
    if (Button_IsPressed(BTN_CANCEL)) {
        FSM_TransitionTo(&coffee_fsm, &state_idle);
        return;
    }
}

static void Heating_OnExit(void)
{
    LED_Red_Off();   // 无论跳到哪里，都会执行这行
    Heater_Off();    // 离开加热状态，必须关加热管
}

State_t state_heating = {
    .name = "HEATING",
    .OnEntry = Heating_OnEntry,
    .OnRun = Heating_OnRun,
    .OnExit = Heating_OnExit
};

/*========== 冲泡状态 ==========*/
static uint32_t brew_timer;

static void Brewing_OnEntry(void)
{
    LED_Blue_On();
    Motor_Start();
    brew_timer = 0;
    Display_Show("Brewing...");
}

static void Brewing_OnRun(void)
{
    brew_timer++;

    if (brew_timer >= BREW_DURATION) {
        FSM_TransitionTo(&coffee_fsm, &state_done);
        return;
    }

    // 冲泡过程中也检查缺水
    if (Sensor_GetWaterLevel() < 5) {
        FSM_TransitionTo(&coffee_fsm, &state_error);
    }
}

static void Brewing_OnExit(void)
{
    LED_Blue_Off();
    Motor_Stop();
}

State_t state_brewing = {
    .name = "BREWING",
    .OnEntry = Brewing_OnEntry,
    .OnRun = Brewing_OnRun,
    .OnExit = Brewing_OnExit
};

/*========== 完成状态 ==========*/
static void Done_OnEntry(void)
{
    LED_Green_Blink();
    Buzzer_Beep(3);
    Display_Show("Done!");
}

static void Done_OnRun(void)
{
    if (Button_IsPressed(BTN_ANY)) {
        FSM_TransitionTo(&coffee_fsm, &state_idle);
    }
}

State_t state_done = {
    .name = "DONE",
    .OnEntry = Done_OnEntry,
    .OnRun = Done_OnRun,
    .OnExit = NULL  // 离开时不需要做什么
};

/*========== 错误状态 ==========*/
static void Error_OnEntry(void)
{
    LED_All_Blink();
    Buzzer_Alarm();
    Display_Show("ERROR!");
}

static void Error_OnRun(void)
{
    // 等待用户处理后复位
    if (Button_IsPressed(BTN_RESET) && Sensor_GetWaterLevel() > 50) {
        FSM_TransitionTo(&coffee_fsm, &state_idle);
    }
}

static void Error_OnExit(void)
{
    LED_All_Off();
    Buzzer_Off();
}

State_t state_error = {
    .name = "ERROR",
    .OnEntry = Error_OnEntry,
    .OnRun = Error_OnRun,
    .OnExit = Error_OnExit
};

/*========== 主函数 ==========*/
void CoffeeMachine_Init(void)
{
    FSM_Init(&coffee_fsm, &state_idle);
}

void CoffeeMachine_Task(void)
{
    FSM_Run(&coffee_fsm);
}
