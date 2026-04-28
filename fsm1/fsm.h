// fsm.h - 状态机框架头文件

#ifndef __FSM_H__
#define __FSM_H__

// 状态结构体：每个状态都是这个"模板"的实例
typedef struct {
    const char *name;       // 状态名称（调试用）
    void (*OnEntry)(void);  // 进入状态时执行
    void (*OnRun)(void);    // 状态运行时执行（每次循环调用）
    void (*OnExit)(void);   // 离开状态时执行
} State_t;

// 状态机结构体
typedef struct {
    State_t *current;       // 当前状态
    State_t *next;          // 下一个状态（用于延迟切换）
} FSM_t;

// 框架函数声明
void FSM_Init(FSM_t *fsm, State_t *init_state);
void FSM_Run(FSM_t *fsm);
void FSM_TransitionTo(FSM_t *fsm, State_t *next_state);

#endif
