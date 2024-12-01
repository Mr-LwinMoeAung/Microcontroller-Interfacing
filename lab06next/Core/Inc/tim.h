#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif




/* TIM1 initialization function */
void MX_TIM1_Init(void);

/* TIM2 initialization function */
void MX_TIM2_Init(void);

/* TIM2 interrupt handler */
void TIM2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
