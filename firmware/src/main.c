/*
 * main.c
 *
 *  Created on: Aug 21, 2023
 *      Author: Amit Kumar - S223
 */

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "string.h"                     // for string operations

/**FreeRTOS Includes**/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"



/**Defines for BOOTLOADER**/
#define BTL_TRIGGER_PATTERN (0x5048434DUL)  //Bootloadaer trigger pattern.
#define BTL_TRIGGER_RAM_START  0x20000000U  //Bootloader RAM start address.


/**Defines for ESP**/
#define ESP_BUFFER_LENGTH   256             //Maximum buffer length of esp tx data

/**Defines for AC_DC_ENERGY METER**/
#define AC_DC_METER_BUFFER_LENGTH   256     //Maximum buffer length of ac_dc meter tx data

/**Defines for HMI**/
#define HMI_BUFFER_LENGTH   50              //Maximum buffer length of HMI tx data

/**Defines for MACHINE STATE**/
#define PHASE_CHECK_STATE   0               //phase checking state of system
#define NORMAL_STATE    1                   //normal state of system
#define SELF_TEST_STATE 2                   //self test state of system
#define RUNNING_STATE   3                   //running state of system
#define SINGLE_GUN_STATE    4               //Single gun charging state
#define DOUBLE_GUN_STATE    5               //Double gun charging state

/**Define for phase difference**/
#define PHASE_DIFFERENCE2   4              //correct phase sequence difference value 3~4
#define PHASE_DIFFERENCE1   3              //correct phase sequence difference value 3~4
#define PHASE_DIFFERENCE4   6              //correct phase sequence difference value 6~7
#define PHASE_DIFFERENCE3   7              //correct phase sequence difference value 6~7


/**Typedef structure QUEUE for SENDING data to ESP **/
typedef struct
{
uint8_t ESP_SEND_DATA[ESP_BUFFER_LENGTH];   //Variable to store data which needs to send to esp queue.
} ESP_SEND_Q;
ESP_SEND_Q   *esp_send_q;

/**Typedef structure QUEUE for SENDING data to AC_DC ENERGY Meter **/
typedef struct
{
uint8_t AC_DC_SEND_DATA[AC_DC_METER_BUFFER_LENGTH]; //Variable to store data which needs to send to ac_dc meter queue.
} AC_DC_METER_SEND_Q;
AC_DC_METER_SEND_Q   *ac_dc_meter_q;


/**Typedef structure QUEUE for SENDING data to HMI **/
typedef struct
{
uint8_t HMI_SEND_DATA[HMI_BUFFER_LENGTH];   //Variable to store data which needs to send to HMI queue.
} HMI_SEND_Q;
HMI_SEND_Q   *hmi_q;


/**PRIVATE GLOBAL VARIABLES**/
static int MACHINE_STATE = PHASE_CHECK_STATE;       //VARIABLE to store the state of System.
static uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;  //VARIABLE to store the bootloader trigger pattern.
long tick1,tick2=0;                                 //Variable to store Time at which phase1 & phase 2 interuppt occurs.




/**ESP RELATED TASKS HANDLES & QUEUES**/
TaskHandle_t ESP_RECEIVE_TaskHandle;                //ESP RECEIVE HANDLE
TaskHandle_t ESP_SEND_TaskHandle;                   //ESP SEND HANDLE
QueueHandle_t ESP_QUEUE=0;                          //ESP QUEUE VARIABLE


/**AC_DC_METER RELATED TASKS HANDLES & QUEUES**/
TaskHandle_t AC_DC_METER_RECEIVE_TaskHandle;        //AC_DC_METER RECEIVE HANDLE
TaskHandle_t AC_DC_METER_SEND_TaskHandle;           //AC_DC_METER SEND HANDLE
QueueHandle_t AC_DC_METER_QUEUE=0;                  //AC_DC_METER QUEUE VARIABLE


/**HMI RELATED TASKS HANDLES & QUEUES**/
TaskHandle_t HMI_RECEIVE_TaskHandle;                //HMI RECEIVE HANDLE
TaskHandle_t HMI_SEND_TaskHandle;                   //HMI SEND HANDLE
QueueHandle_t HMI_QUEUE=0;                          //HMI QUEUE VARIABLE

/**COMMON TASKS HANDLES & QUEUES**/
TaskHandle_t defaultTaskHandle;                     //default task HANDLE
TaskHandle_t TEMP_HUMI_TaskHandle;                  //HUMIDITY TEMPERATURE HANDLE
TaskHandle_t TEMP_ADC_TaskHandle;                   //TEMPERATURE HANDLE
TaskHandle_t GUN_TEMP_TaskHandle;                   //GUN TEMPERATURE HANDLE
TaskHandle_t SMOKE_SENSE_TaskHandle;                //SMOKE SENSOR HANDLE
TaskHandle_t MACHINE_STATEHandle;                   //MACHINE STATE HANDLE


/**PFP**/
/**PHASE DETECTION CALLBACK FUNCTIONS**/
static void PHASE1_NOTIFICATION(uintptr_t context); //Callback function for phase 1 detection.   
static void PHASE2_NOTIFICATION(uintptr_t context); //Callback function for phase 2 detection.

/**COMMON TASKS FUNCTION**/
void StartDefaultTask(void * argument);             //Default TASK Function
void StartTEMP_HUMITask(void * argument);           //Humidity Temperature TASK Function
void StartTEMP_ADCTask(void * argument);            //Temperature ADC TASK Function
void StartGUN_TEMPTask(void * argument);            //GUN Temperature TASK Function
void StartSMOKE_SENSETask(void * argument);         //SMOKE_SENSOR TASK Function
void StartMACHINE_STATETask(void * argument);       //MACHINE STATE TASK Function

/**ESP RELATED TASKS FUNCTIONS, VARIABLES & CALLBACKS**/
static uint8_t ESP_RECEIVING_SIZE   =   2;          //Variable to store cuurent receiving size of ESP
uint8_t ESP_RX_DATA[520] = {0};                     //Variable to store receiving data from esp
void StartESP_RECEIVETask(void * argument);         //ESP RECEIVE TASK Function
void StartESP_SENDTask(void * argument);            //ESP SEND TASK Function
void ESP_CALLBACK(uintptr_t context);               //ESP RECEIVE CALLBACK Function

/**HMI RELATED TASKS FUNCTIONS, VARIABLES & CALLBACKS**/
static uint8_t HMI_RECEIVING_SIZE   =   20;         //Variable to store current receiving size of HMI
uint8_t HMI_RX_DATA[50] = {0};                      //Variable to store receiving data of HMI
void StartHMI_RECEIVETask(void * argument);         //HMI RECEIVE TASK Function
void StartHMI_SENDTask(void * argument);            //HMI SEND TASK Function
void HMI_CALLBACK(uintptr_t context);               //HMI RECEIVE CALLBACK Function

/**AC_DC_METER RELATED TASKS FUNCTIONS, VARIABLES & CALLBACKS**/
static uint8_t AC_DC_METER_RECEIVING_SIZE = 100;    //Variable to store current receiving size of AC_DC_METRE
uint8_t AC_DC_METER_RX_DATA[260] = {0};             //Variable to store receiving data of AC_DC_METRE
void StartAC_DC_METER_RECEIVETask(void * argument); //AC_DC_METER RECEIVE TASK Function
void StartAC_DC_METER_SENDTask(void * argument);    //AC_DC_METER SEND TASK Function
void AC_DC_METER_CALLBACK(uintptr_t context);       //AC_DC_METER RECEIVE CALLBACK Function


int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    /**REGISTERING SERCOM CALLBACKS**/
    SERCOM7_USART_ReadCallbackRegister(ESP_CALLBACK, 0);    //Registered ESP Callback
    SERCOM2_USART_ReadCallbackRegister(AC_DC_METER_CALLBACK,0); //Registered AC_DC_METER Callback
    SERCOM6_USART_ReadCallbackRegister(HMI_CALLBACK, 0);    //Registered HMI Callback

    /**REGISTERING GPIO/EIC CALLBACKS**/
    EIC_CallbackRegister(EIC_PIN_10,PHASE1_NOTIFICATION, 0);//Registered Phase1 GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_11,PHASE2_NOTIFICATION, 0);//Registered Phase2 GPIO input interuppt Callback
    
    /**QUEUES CREATIONS**/
    ESP_QUEUE = xQueueCreate(1, sizeof(ESP_SEND_Q));        //ESP QUEUE CREATION
    HMI_QUEUE = xQueueCreate(1, sizeof(HMI_SEND_Q));        //HMI QUEUE CREATION
    AC_DC_METER_QUEUE = xQueueCreate(1,sizeof(AC_DC_METER_SEND_Q)); //AC_DC_METER QUEUE CREATION
    
    /**TASK CREATIONS**/
    xTaskCreate(StartDefaultTask,"StartDefaultTask",128,NULL,1,&defaultTaskHandle);                                         //DEFAULT TASK CREATION
    xTaskCreate(StartESP_RECEIVETask,"StartESP_RECEIVETask",512,NULL,1,&ESP_RECEIVE_TaskHandle);                            //ESP RECEIVE TASK CREATION
    xTaskCreate(StartESP_SENDTask,"StartESP_RECEIVETask",512,NULL,1,&ESP_SEND_TaskHandle);                                  //ESP SEND TASK CREATION
    xTaskCreate(StartAC_DC_METER_SENDTask,"StartAC_DC_METER_SENDTask",512,NULL,1,&AC_DC_METER_SEND_TaskHandle);             //AC_DC METER SEND TASK CREATION
    xTaskCreate(StartAC_DC_METER_RECEIVETask,"StartAC_DC_METER_RECEIVETask",512,NULL,1,&AC_DC_METER_RECEIVE_TaskHandle);    //AC_DC_METER RECEIVE TASK CREATION
    xTaskCreate(StartTEMP_HUMITask,"StartTEMP_HUMITask",128,NULL,1,&TEMP_HUMI_TaskHandle);                                  //HUMIDITY TEMP TASK CREATION
    xTaskCreate(StartTEMP_ADCTask,"StartTEMP_ADCTask",256,NULL,1,&TEMP_ADC_TaskHandle);                                     //TEMP ADC TASK CREATION
    xTaskCreate(StartGUN_TEMPTask,"StartGUN_TEMPTask",128,NULL,1,&GUN_TEMP_TaskHandle);                                     //GUN TEMP TASK CREATION
    xTaskCreate(StartHMI_RECEIVETask,"StartHMI_RECEIVETask",512,NULL,1,&HMI_RECEIVE_TaskHandle);                            //HMI RECEIVE TASK CREATION
    xTaskCreate(StartHMI_SENDTask,"StartHMI_RECEIVETask",512,NULL,1,&HMI_SEND_TaskHandle);                                  //HMIO SEND TASK CREATION
    xTaskCreate(StartSMOKE_SENSETask,"StartSMOKE_SENSETask",128,NULL,1,&SMOKE_SENSE_TaskHandle);                            //SMOKE SENSING TASK CREATION
    xTaskCreate(StartMACHINE_STATETask,"StartMACHINE_STATETask",128,NULL,1,&MACHINE_STATEHandle);                           //MACHINE STATE TASK CREATION
            
    /**ENABLING SERCOM Intruppt with their receiving size**/
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);                    //Enabling ESP receiving Intruppt
    SERCOM2_USART_Read(AC_DC_METER_RX_DATA, AC_DC_METER_RECEIVING_SIZE);    //Enabling AC_DC_METER receiving Intruppt
    SERCOM6_USART_Read(HMI_RX_DATA, HMI_RECEIVING_SIZE);                    //Enabling HMI receiving Intruppt
    
    /**SUSPENDING TASK BEFORE SATRTINg KERNEL**/
    vTaskSuspend(ESP_RECEIVE_TaskHandle);                                   //ESP RECEIVE TASK SUSPENDED
    vTaskSuspend(AC_DC_METER_RECEIVE_TaskHandle);                           //AC_DC_METER RECEIVE TASK SUSPENDED
    vTaskSuspend(ESP_SEND_TaskHandle);                                      //ESP_SEND TASK SUSPENDED
    vTaskSuspend(AC_DC_METER_SEND_TaskHandle);                              //AC_DC_METER TASK SUSPENDED
    vTaskSuspend(TEMP_HUMI_TaskHandle);                                     //TEMPERATURE HUMIDITY TASK SUSPENDED
    vTaskSuspend(TEMP_ADC_TaskHandle);                                      //TEMPERATURE ADC TASK SUSPENDED
    vTaskSuspend(GUN_TEMP_TaskHandle);                                      //GUN TEMPERATURE TASK SUSPENDED
    vTaskSuspend(HMI_RECEIVE_TaskHandle);                                   //HMI RECEIVE TASK SUSPENDED
    vTaskSuspend(HMI_SEND_TaskHandle);                                      //HMI SEND TASK SUSPENDED
    vTaskSuspend(SMOKE_SENSE_TaskHandle);                                   //SMOKE SENSING TASK SUSPENDED
    vTaskSuspend(MACHINE_STATEHandle);                                      //MACHINE STATE TASK SUSPENDED
    
    /**STARTING SCHEDULER**/
    vTaskStartScheduler();
    
    while ( true )
    {
        //Command never reaches HERE.
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/**CALLBACK FUNCTION FOR ESP RECEIVING**/
void ESP_CALLBACK(uintptr_t context)
{
    vTaskResume(ESP_RECEIVE_TaskHandle);                        //Resuming  ESP RECEIVE TASK
}

/**CALLBACK FUNCTION FOR AC_DC_METER RECEIVING**/
void AC_DC_METER_CALLBACK(uintptr_t context)
{
    vTaskResume(AC_DC_METER_RECEIVE_TaskHandle);                //Resuming AC_DC_METER RECEIVE TASK
}

/**CALLBACK FUNCTION FOR HMI RECEIVING**/
void HMI_CALLBACK(uintptr_t context)
{
    vTaskResume(HMI_RECEIVE_TaskHandle);                        //Resuming HMI RECEIVE TASK
}

/**CALLBACK FUNCTION FOR PHASE1 GPIO INTRUPPT DETECTION**/
static void PHASE1_NOTIFICATION(uintptr_t context)
{
    tick1 = xTaskGetTickCount(); 
//    printf("phase1  = %ld\r\n",tick1);
}

/**CALLBACK FUNCTION FOR PHASE2 GPIO INTRUPPT DETECTION**/
static void PHASE2_NOTIFICATION(uintptr_t context)
{   
    tick2 = xTaskGetTickCount(); 
//    printf("phase 2 = %ld\r\n",tick2);
}


/**Function for default TASK**/
void StartDefaultTask(void * argument)
{
    static int count =0;                                        //Variable to store task count
    uint32_t difference1 =0;                      //Variables to store difference between recorded time
    while(1)
    {
        if(count>3 && MACHINE_STATE == PHASE_CHECK_STATE)
        {
            difference1 = tick2-tick1;
            if(difference1 == PHASE_DIFFERENCE1 || difference1 == PHASE_DIFFERENCE2)    //Checking if phase difference is ok
            {
                //TODO : HMI + (also remaining checks)LED WHITE(ONLINE) / LED PINK (ONLINE) + ESP
                vTaskResume(MACHINE_STATEHandle);               //RESUMING MACHINE STATE TASK to change the state of MACHINE from phase checking state to other.
                printf("IN PHASE\r\n");                
            }
            else if(difference1 == PHASE_DIFFERENCE3 || difference1 == PHASE_DIFFERENCE4)   //checking if phase difference is not ok
            {
                //TODO : LED RED + HMI VIEW  +  ESP 
                printf("OUT PHASE\r\n");                
            }
            count = 0;                          //Resetting count variable
        }
        count++;                                //incrementing count variable
        if(count >10)                           //Resetting count variable
        {
            count =0;
        }
    vTaskDelay(1000);
    }
}

/**Function for STATE MACHINE TASK**/
void StartMACHINE_STATETask(void * argument)
{
    MACHINE_STATE = NORMAL_STATE;               //Changing state of machine from Phase detection to normal state
    EIC_InterruptDisable(EIC_PIN_10);           //Disabling Intruppt for phase 1 detection
    EIC_InterruptDisable(EIC_PIN_11);           //Disabling Intruppt for phase 2 detection
    while(1)
    {
        switch(MACHINE_STATE)                   //Maintaining machine state condition
        {
            case    NORMAL_STATE:               //Normal STATE
                break;
            case    SELF_TEST_STATE:            //Self test STATE
                break;
            case    RUNNING_STATE:              //Running STATE
                break;
        }
        vTaskDelay(1);
    }
}

/**Function for ESP RECEIVE TASK**/
void StartESP_RECEIVETask(void * argument)
{
    while(1)
    {

     if(ESP_RX_DATA[0] == 0x61)
    {
            ramStart[0] = BTL_TRIGGER_PATTERN;
            ramStart[1] = BTL_TRIGGER_PATTERN;
            ramStart[2] = BTL_TRIGGER_PATTERN;
            ramStart[3] = BTL_TRIGGER_PATTERN;
            //NVIC_SystemReset();
    }
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);
    vTaskSuspend(ESP_RECEIVE_TaskHandle);
    vTaskDelay(1000);
    }
}

/*ESP_SEND_Q    espmsg
 espmsg.ESP_SEND_DATA[x] = value
 xQueueSend(ESP_QUEUE,&espmsg,1);*/

/**Function for ESP SEND TASK**/
void StartESP_SENDTask(void * argument)
{
    ESP_SEND_Q espmsg;
    while(1)
    {
        if(xQueueReceive(ESP_QUEUE,&espmsg,0))
        {
            
        }
        vTaskDelay(1);
    }
}

/**Function for AC_DC_METER RECEIVE TASK**/
void StartAC_DC_METER_RECEIVETask(void * argument)
{
    while(1)
    {
        vTaskDelay(1);
    }
}

/**Function for AC_DC_METER SEND TASK**/
void StartAC_DC_METER_SENDTask(void * argument)
{
    AC_DC_METER_SEND_Q  ac_dc_msg;
    while(1)
    {
        if(xQueueReceive(AC_DC_METER_QUEUE,&ac_dc_msg,0))
        {
            
        }
        vTaskDelay(1);
    }
}

/**Function for HUMIDITY TEMPERATURE TASK**/
void StartTEMP_HUMITask(void *argument)
{
    while(1)
    {
        vTaskDelay(100);
    }
}


/**Function for TEMPERATURE ADC TASK**/
void StartTEMP_ADCTask(void *argument)
{
    while(1)
    {
        vTaskDelay(100);
    }
}


/**Function for GUN TEMPERATURE TASK**/
void StartGUN_TEMPTask(void * argument)
{
    while(1)
    {
        vTaskDelay(100);
    }    
}


/**Function for HMI RECEIVE TASK**/
void StartHMI_RECEIVETask(void * argument)
{
    while(1)
    {
    SERCOM6_USART_Read(HMI_RX_DATA, HMI_RECEIVING_SIZE);
    vTaskSuspend(HMI_RECEIVE_TaskHandle);
    vTaskDelay(1000);
    }
}


/**Function for HMI SEND TASK**/
void StartHMI_SENDTask(void * argument)
{
    HMI_SEND_Q HMImsg;
    while(1)
    {
        if(xQueueReceive(HMI_QUEUE,&HMImsg,0))
        {
            
        }
        vTaskDelay(1);
    }
}


/**Function for SMOKE SENSING TASK**/
void StartSMOKE_SENSETask(void * argument)
{
    while(1)
    {
        vTaskDelay(100);
    }    
}

