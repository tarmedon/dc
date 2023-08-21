

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"




#define BTL_TRIGGER_PATTERN (0x5048434DUL)
#define BTL_TRIGGER_RAM_START  0x20000000U

#define ESP_BUFFER_LENGTH   256
#define AC_DC_METER_BUFFER_LENGTH   256
#define HMI_BUFFER_LENGTH   50

#define PHASE_CHECK_STATE   0
#define NORMAL_STATE    1
#define SELF_TEST_STATE 2
#define RUNNING_STATE   3

typedef struct
{
uint8_t ESP_SEND_DATA[ESP_BUFFER_LENGTH];
} ESP_SEND_Q;
ESP_SEND_Q   *esp_send_q;

typedef struct
{
uint8_t AC_DC_SEND_DATA[AC_DC_METER_BUFFER_LENGTH];
} AC_DC_METER_SEND_Q;
AC_DC_METER_SEND_Q   *ac_dc_meter_q;

typedef struct
{
uint8_t HMI_SEND_DATA[HMI_BUFFER_LENGTH];
} HMI_SEND_Q;
HMI_SEND_Q   *hmi_q;



static int MACHINE_STATE = PHASE_CHECK_STATE;
static uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;




TaskHandle_t defaultTaskHandle;
TaskHandle_t ESP_RECEIVE_TaskHandle;
TaskHandle_t ESP_SEND_TaskHandle;
QueueHandle_t ESP_QUEUE=0;

TaskHandle_t AC_DC_METER_RECEIVE_TaskHandle;
TaskHandle_t AC_DC_METER_SEND_TaskHandle;
QueueHandle_t AC_DC_METER_QUEUE=0;

TaskHandle_t HMI_RECEIVE_TaskHandle;
TaskHandle_t HMI_SEND_TaskHandle;
QueueHandle_t HMI_QUEUE=0;

TaskHandle_t TEMP_HUMI_TaskHandle;
TaskHandle_t TEMP_ADC_TaskHandle;
TaskHandle_t GUN_TEMP_TaskHandle;
TaskHandle_t SMOKE_SENSE_TaskHandle;
TaskHandle_t MACHINE_STATEHandle;



static void PHASE1_NOTIFICATION(uintptr_t context);
static void PHASE2_NOTIFICATION(uintptr_t context);

void StartDefaultTask(void * argument);
void StartTEMP_HUMITask(void * argument);
void StartTEMP_ADCTask(void * argument);
void StartGUN_TEMPTask(void * argument);
void StartSMOKE_SENSETask(void * argument);
void StartMACHINE_STATETask(void * argument);

static uint8_t ESP_RECEIVING_SIZE   =   2;
uint8_t ESP_RX_DATA[520] = {0};
void StartESP_RECEIVETask(void * argument);
void StartESP_SENDTask(void * argument);
void ESP_CALLBACK(uintptr_t context);

static uint8_t HMI_RECEIVING_SIZE   =   20;
uint8_t HMI_RX_DATA[50] = {0};
void StartHMI_RECEIVETask(void * argument);
void StartHMI_SENDTask(void * argument);
void HMI_CALLBACK(uintptr_t context);

static uint8_t AC_DC_METER_RECEIVING_SIZE = 100;
uint8_t AC_DC_METER_RX_DATA[260] = {0};
void StartAC_DC_METER_RECEIVETask(void * argument);
void StartAC_DC_METER_SENDTask(void * argument);
void AC_DC_METER_CALLBACK(uintptr_t context);








int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    
    SERCOM7_USART_ReadCallbackRegister(ESP_CALLBACK, 0);
    SERCOM2_USART_ReadCallbackRegister(AC_DC_METER_CALLBACK,0);
    SERCOM6_USART_ReadCallbackRegister(HMI_CALLBACK, 0);

    EIC_CallbackRegister(EIC_PIN_10,PHASE1_NOTIFICATION, 0);
    EIC_CallbackRegister(EIC_PIN_11,PHASE2_NOTIFICATION, 0);
    ESP_QUEUE = xQueueCreate(1, sizeof(ESP_SEND_Q));
    HMI_QUEUE = xQueueCreate(1, sizeof(HMI_SEND_Q));
    AC_DC_METER_QUEUE = xQueueCreate(1,sizeof(AC_DC_METER_SEND_Q));
    
    xTaskCreate(StartDefaultTask,"StartDefaultTask",128,NULL,1,&defaultTaskHandle);
    xTaskCreate(StartESP_RECEIVETask,"StartESP_RECEIVETask",512,NULL,1,&ESP_RECEIVE_TaskHandle);
    xTaskCreate(StartESP_SENDTask,"StartESP_RECEIVETask",512,NULL,1,&ESP_SEND_TaskHandle);
    xTaskCreate(StartAC_DC_METER_SENDTask,"StartAC_DC_METER_SENDTask",512,NULL,1,&AC_DC_METER_SEND_TaskHandle);
    xTaskCreate(StartAC_DC_METER_RECEIVETask,"StartAC_DC_METER_RECEIVETask",512,NULL,1,&AC_DC_METER_RECEIVE_TaskHandle);
    xTaskCreate(StartTEMP_HUMITask,"StartTEMP_HUMITask",128,NULL,1,&TEMP_HUMI_TaskHandle);
    xTaskCreate(StartTEMP_ADCTask,"StartTEMP_ADCTask",256,NULL,1,&TEMP_ADC_TaskHandle);
    xTaskCreate(StartGUN_TEMPTask,"StartGUN_TEMPTask",128,NULL,1,&GUN_TEMP_TaskHandle);
    xTaskCreate(StartHMI_RECEIVETask,"StartHMI_RECEIVETask",512,NULL,1,&HMI_RECEIVE_TaskHandle);
    xTaskCreate(StartHMI_SENDTask,"StartHMI_RECEIVETask",512,NULL,1,&HMI_SEND_TaskHandle);
    xTaskCreate(StartSMOKE_SENSETask,"StartSMOKE_SENSETask",128,NULL,1,&SMOKE_SENSE_TaskHandle);
    xTaskCreate(StartMACHINE_STATETask,"StartMACHINE_STATETask",128,NULL,1,&MACHINE_STATEHandle);
            
    
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);
    SERCOM2_USART_Read(AC_DC_METER_RX_DATA, AC_DC_METER_RECEIVING_SIZE);
    SERCOM6_USART_Read(HMI_RX_DATA, HMI_RECEIVING_SIZE);
    
    vTaskSuspend(ESP_RECEIVE_TaskHandle);
    vTaskSuspend(AC_DC_METER_RECEIVE_TaskHandle);
    vTaskSuspend(ESP_SEND_TaskHandle);
    vTaskSuspend(AC_DC_METER_SEND_TaskHandle);
    vTaskSuspend(TEMP_HUMI_TaskHandle);
    vTaskSuspend(TEMP_ADC_TaskHandle);
    vTaskSuspend(GUN_TEMP_TaskHandle);
    vTaskSuspend(HMI_RECEIVE_TaskHandle);
    vTaskSuspend(HMI_SEND_TaskHandle);
    vTaskSuspend(SMOKE_SENSE_TaskHandle);
    vTaskSuspend(MACHINE_STATEHandle);
    vTaskStartScheduler();
    
    while ( true )
    {

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

void ESP_CALLBACK(uintptr_t context)
{
    vTaskResume(ESP_RECEIVE_TaskHandle);
}
void AC_DC_METER_CALLBACK(uintptr_t context)
{
    vTaskResume(AC_DC_METER_RECEIVE_TaskHandle);
}
void HMI_CALLBACK(uintptr_t context)
{
    vTaskResume(HMI_RECEIVE_TaskHandle);
}

static void PHASE1_NOTIFICATION(uintptr_t context)
{
    xTaskNotifyGive(defaultTaskHandle);
}

static void PHASE2_NOTIFICATION(uintptr_t context)
{
    xTaskNotifyGive(defaultTaskHandle);
}

void StartDefaultTask(void * argument)
{
    static int count =0;
    int previous_tick=0,current_tick=0,tick_difference;
    while(1)
    {
        
        count = ulTaskNotifyTake(pdTRUE, 0);
        if(count == 1 && MACHINE_STATE == PHASE_CHECK_STATE)
        {
            current_tick = xTaskGetTickCount();
            tick_difference = current_tick - previous_tick;
            previous_tick = current_tick;
            printf("%d",tick_difference);
            if(tick_difference == 1000)
            {
                vTaskResume(MACHINE_STATEHandle);
            }
        }  
    vTaskDelay(1);
    }
}

void StartMACHINE_STATETask(void * argument)
{
    MACHINE_STATE = SELF_TEST_STATE;
    while(1)
    {
        vTaskDelay(1);
    }
}

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
            NVIC_SystemReset();
    }
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);
    vTaskSuspend(ESP_RECEIVE_TaskHandle);
    vTaskDelay(1000);
    }
}

/*ESP_SEND_Q    espmsg
 espmsg.ESP_SEND_DATA[x] = value
 xQueueSend(ESP_QUEUE,&espmsg,1);*/

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

void StartAC_DC_METER_RECEIVETask(void * argument)
{
    while(1)
    {
        vTaskDelay(1);
    }
}
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

void StartTEMP_HUMITask(void *argument)
{
    while(1)
    {
        vTaskDelay(100);
    }
}

void StartTEMP_ADCTask(void *argument)
{
    while(1)
    {
        vTaskDelay(100);
    }
}

void StartGUN_TEMPTask(void * argument)
{
    while(1)
    {
        vTaskDelay(100);
    }    
}

void StartHMI_RECEIVETask(void * argument)
{
    while(1)
    {
    SERCOM6_USART_Read(HMI_RX_DATA, HMI_RECEIVING_SIZE);
    vTaskSuspend(HMI_RECEIVE_TaskHandle);
    vTaskDelay(1000);
    }
}

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

void StartSMOKE_SENSETask(void * argument)
{
    while(1)
    {
        vTaskDelay(100);
    }    
}

