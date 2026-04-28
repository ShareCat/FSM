// fsm.c - 状态机框架实现

#include "fsm.h"
#include <stddef.h>  // for NULL

// 初始化状态机
void FSM_Init(FSM_t *fsm, State_t *init_state)
{
    fsm->current = init_state;
    fsm->next = NULL;

    // 执行初始状态的进入动作
    if (fsm->current && fsm->current->OnEntry) {
        fsm->current->OnEntry();
    }
}

// 状态机主循环（每次调用执行一轮）
void FSM_Run(FSM_t *fsm)
{
    // 1. 检查是否有待切换的状态
    if (fsm->next != NULL) {
        // 执行当前状态的退出动作
        if (fsm->current && fsm->current->OnExit) {
            fsm->current->OnExit();
        }

        // 切换到新状态
        fsm->current = fsm->next;
        fsm->next = NULL;

        // 执行新状态的进入动作
        if (fsm->current && fsm->current->OnEntry) {
            fsm->current->OnEntry();
        }
    }

    // 2. 执行当前状态的运行逻辑
    if (fsm->current && fsm->current->OnRun) {
        fsm->current->OnRun();
    }
}

// 请求切换到新状态（不会立即切换，而是标记）
void FSM_TransitionTo(FSM_t *fsm, State_t *next_state)
{
    if (next_state != fsm->current) {  // 避免切换到自己
        fsm->next = next_state;
    }
}
