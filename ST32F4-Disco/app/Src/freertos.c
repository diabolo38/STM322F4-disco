#if USE_FREE_RTOS

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"

void CDC_thread(void const * argument){
    int delay_ms=200;
    do{
        if(DemoEnterCondition ){
            DoCDC_Info();
        }
        osDelay(delay_ms);
    }while(1);
}

void freetos_init(){
  osThreadDef(CDCMsg_Thread, CDC_thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(CDCMsg_Thread), NULL);
}

#endif //USE_FREE_RTOS
