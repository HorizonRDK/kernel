#ifndef __HOBOT_JPU_CONFIG_H_
#define __HOBOT_JPU_CONFIG_H_

/* definitions to be changed as customer  configuration */
/* if you want to have clock gating scheme frame by frame */
//#define JPU_SUPPORT_CLOCK_CONTROL
#define JPU_SUPPORT_ISR

#ifdef JPU_SUPPORT_ISR
#define JPU_IRQ_NUM                 	67+32	//15+32

/* if the driver want to disable and enable IRQ whenever interrupt asserted. */
#define JPU_IRQ_CONTROL
#endif

#define MAX_NUM_JPU_INSTANCE            64
#define MAX_NUM_JPU_CORE                1
#define MAX_INST_HANDLE_SIZE            48

#endif /* __HOBOT_JPU_CONFIG_H_ */
