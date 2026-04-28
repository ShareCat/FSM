// at_parser.c - 表驱动状态机示例

#include <stdint.h>
#include <stddef.h>

/*========== 状态和事件定义 ==========*/
typedefenum {
    ST_IDLE,
    ST_WAIT_OK,
    ST_WAIT_DATA,
    ST_ERROR,
    ST_MAX
} ATState_e;

typedefenum {
    EVT_SEND_CMD,       // 发送了指令
    EVT_RECV_OK,        // 收到 "OK"
    EVT_RECV_ERROR,     // 收到 "ERROR"
    EVT_RECV_DATA,      // 收到数据
    EVT_TIMEOUT,        // 超时
    EVT_RETRY,          // 重试
    EVT_MAX
} ATEvent_e;

/*========== 动作函数 ==========*/
static void Action_SendCmd(void)
{
    printf("[Action] Sending AT command...\n");
    // 实际发送指令到串口
}

static void Action_ProcessData(void)
{
    printf("[Action] Processing received data...\n");
    // 处理收到的数据
}

static void Action_HandleError(void)
{
    printf("[Action] Error occurred, preparing retry...\n");
    // 错误处理
}

static void Action_Reset(void)
{
    printf("[Action] Resetting state machine...\n");
    // 复位操作
}

/*========== 状态转移表 ==========*/
// 核心！整个状态机的逻辑都在这张表里
static const Transition_t transition_table[] = {
    // 当前状态      触发事件          下一状态        执行动作
    { ST_IDLE,      EVT_SEND_CMD,     ST_WAIT_OK,     Action_SendCmd     },

    { ST_WAIT_OK,   EVT_RECV_OK,      ST_WAIT_DATA,   NULL               },
    { ST_WAIT_OK,   EVT_RECV_ERROR,   ST_ERROR,       Action_HandleError },
    { ST_WAIT_OK,   EVT_TIMEOUT,      ST_ERROR,       Action_HandleError },

    { ST_WAIT_DATA, EVT_RECV_DATA,    ST_IDLE,        Action_ProcessData },
    { ST_WAIT_DATA, EVT_RECV_ERROR,   ST_ERROR,       Action_HandleError },
    { ST_WAIT_DATA, EVT_TIMEOUT,      ST_ERROR,       Action_HandleError },

    { ST_ERROR,     EVT_RETRY,        ST_IDLE,        Action_Reset       },
};

#define TRANSITION_COUNT (sizeof(transition_table) / sizeof(transition_table[0]))

/*========== 状态机引擎 ==========*/
static ATState_e current_state = ST_IDLE;

// 查表并执行状态转移
void AT_FSM_HandleEvent(ATEvent_e event)
{
    for (int i = 0; i < TRANSITION_COUNT; i++) {
        // 查找匹配的转移规则
        if (transition_table[i].current_state == current_state &&
            transition_table[i].event == event) {

            // 执行动作（如果有）
            if (transition_table[i].action != NULL) {
                transition_table[i].action();
            }

            // 切换状态
            printf("[FSM] %d -> %d (event: %d)\n",
                   current_state,
                   transition_table[i].next_state,
                   event);

            current_state = transition_table[i].next_state;
            return;
        }
    }

    // 没找到匹配的规则，说明这个事件在当前状态下被忽略
    printf("[FSM] Event %d ignored in state %d\n", event, current_state);
}

// 获取当前状态
ATState_e AT_FSM_GetState(void)
{
    return current_state;
}
