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

/**Basic defines for CODE**/
#define OK                  0
#define NOT_OK              1

/**Defines for ESP**/
#define ESP_BUFFER_LENGTH   256             //Maximum buffer length of esp tx data

/**Defines for AC_DC_ENERGY METER**/
#define AC_DC_METER_BUFFER_LENGTH   256     //Maximum buffer length of ac_dc meter tx data

/**Defines for HMI**/
#define HMI_BUFFER_LENGTH   50              //Maximum buffer length of HMI tx data

/**Defines for RGB**/
#define RGB_BUFFER_LENGTH   1               //MAximum buffer length of RGB LED.
#define RED                 0x00            //Define to state & write duty cycle to timer for red color. 
#define GREEN               0x01            //Define to state & write duty cycle to timer for green color. 
#define BLUE                0x02            //Define to state & write duty cycle to timer for blue color. 
#define YELLOW              0x03            //Define to state & write duty cycle to timer for yellow color. 
#define CYAN                0x04            //Define to state & write duty cycle to timer for cyan color. 
#define MAGENTA             0x05            //Define to state & write duty cycle to timer for magenta color. 
#define ORANGE              0x06            //Define to state & write duty cycle to timer for orange color. 
#define YELLOW_GREEN        0x07            //Define to state & write duty cycle to timer for yellow-green color. 
#define CYAN_GREEN          0x08            //Define to state & write duty cycle to timer for cyan-green color. 
#define CYAN_BLUE           0x09            //Define to state & write duty cycle to timer for cyan-blue color. 
#define BLUE_MAGENTA        0x0A            //Define to state & write duty cycle to timer for blue-magenta color. 
#define RED_MAGENTA         0x0B            //Define to state & write duty cycle to timer for red-magenta color. 
#define GREEN_BLINK         0x0C            //Define to state & write duty cycle to timer for green blink color.


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

/**Defines for RFID**/
#define RFID_WAKEUP_RX_SIZE     15
#define RFID_FIRMWARE_RX_SIZE   19
#define RFID_ID_RX_SIZE         25
#define RFID_BUFFER_SIZE        50
#define RFID_INDEX9_DATA         0x0C
#define RFID_INDEX10_DATA        0xF4
#define RFID_INDEX11_DATA        0xD5
#define RFID_INDEX12_DATA        0x4B
#define RFID_INDEX13_DATA        0x01


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

/**Typedef structure QUEUE for SENDING data to HMI **/
typedef struct
{
uint8_t RGB_DATA[RGB_BUFFER_LENGTH];   //Variable to store data which needs to send to HMI queue.
} RGB_SEND_Q;
RGB_SEND_Q   *rgb_q;

typedef struct
{
uint8_t RGB1_DATA[RGB_BUFFER_LENGTH];   //Variable to store data which needs to send to HMI queue.
} RGB1_SEND_Q;
RGB1_SEND_Q   *rgb1_q;


/**PRIVATE GLOBAL VARIABLES**/
static int MACHINE_STATE = PHASE_CHECK_STATE;       //VARIABLE to store the state of System.
static uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;  //VARIABLE to store the bootloader trigger pattern.
long tick1,tick2=0;                                 //Variable to store Time at which phase1 & phase 2 interuppt occurs.
static uint8_t Emergency_button_t = OK;             //Flag for Emergency Button.
static uint8_t Imd_t = OK;                          //Flag for IMD.
static uint8_t Smoke_Detection_t = OK;              //Flag for Smoke Detection.
static uint8_t Limit_switch_t = OK;                 //Flag for limit switch.
static uint8_t spd_detection_t = OK;                //Flasg for SPD.
static uint8_t rfid_detection_t = OK;
static uint8_t RFID_DATA[RFID_BUFFER_SIZE] = {0};
static uint8_t CURRENT_RFID_RX_SIZE = RFID_WAKEUP_RX_SIZE;




/**ESP RELATED TASKS HANDLES & QUEUES**/
TaskHandle_t ESP_RECEIVE_TaskHandle;                //ESP RECEIVE HANDLE
TaskHandle_t ESP_SEND_TaskHandle;                   //ESP SEND HANDLE
QueueHandle_t ESP_QUEUE=0;                          //ESP QUEUE VARIABLE

/**RGB RELATED TASKS HANDLES & QUEUES**/
TaskHandle_t RGB_SEND_TaskHandle;                   //RGB SEND HANDLE
QueueHandle_t RGB_QUEUE=0;                          //RGB QUEUE VARIABLE
QueueHandle_t RGB1_QUEUE=0;                          //RGB1 QUEUE VARIABLE


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
TaskHandle_t MACHINE_STATEHandle;                   //MACHINE STATE HANDLE
TaskHandle_t POWERBANK_MONITOR_TaskHandle;
TaskHandle_t RFID_SENDHandle;
TaskHandle_t RFID_RECEIVEHandle;

/**Timers HANDLE**/
static TimerHandle_t mains_timer = NULL;

/**PFP**/


/**COMMON FUNCTIONS**/
void RED_t( uint16_t Compare_0_Value);                //Function to set duty cycle to TC0 timer for controlling RED color.
void GREEN_BLUE( uint16_t Compare_0_Value ,uint16_t Compare_1_Value);       //Function to set duty cycle to TC1 timer for controlling GREEN_BLUE color.
void RED_t1( uint16_t Compare_0_Value);                //Function to set duty cycle to TCC4 timer for controlling RED color.
void GREEN_BLUE1( uint16_t Compare_0_Value ,uint16_t Compare_1_Value);       //Function to set duty cycle to TC1 timer for controlling GREEN_BLUE color.

/**PHASE DETECTION CALLBACK FUNCTIONS**/
static void PHASE1_NOTIFICATION(uintptr_t context); //Callback function for phase 1 detection.   
static void PHASE2_NOTIFICATION(uintptr_t context); //Callback function for phase 2 detection.

/**GPIO CALLBACK FUNCTIONS**/
static void EMERGENCY_BTN(uintptr_t context);       //Callback function for Emergency Button.
static void IMD_NOTIFICATION(uintptr_t context);    //Callback function for IMD.
static void SMOKE_DETECTION(uintptr_t context);     //Callback function for smoke detection.
static void LIMIT_SWITCH(uintptr_t context);        //Callback function for Limit switch.
static void RGB1_AVAILABLITY_t(uintptr_t context);        //Callback function for RGB1.
static void RGB2_AVAILABLITY_t(uintptr_t context);        //Callback function for RGB2.
static void SPD_DETECTION(uintptr_t context);       //Callback for SPD Detection.  

/**TIMER CALLBACK FUNCTION**/
void MainsCallback(TimerHandle_t xTimer);

/**RFID RECEIVE CALLBACK FUNCTION**/
void RFID_CALLBACK(uintptr_t context);


/**COMMON TASKS FUNCTION**/
void StartDefaultTask(void * argument);             //Default TASK Function
void StartTEMP_HUMITask(void * argument);           //Humidity Temperature TASK Function
void StartTEMP_ADCTask(void * argument);            //Temperature ADC TASK Function
void StartGUN_TEMPTask(void * argument);            //GUN Temperature TASK Function
void StartMACHINE_STATETask(void * argument);       //MACHINE STATE TASK Function
void StartRGB_SENDTask(void * argument);
void StartPOWERBANK_MONITOR_Task(void * argument);
void StartRFID_SENDTask(void * argument);
void StartRFID_RECEIVETask(void * argument);

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

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    /**REGISTERING SERCOM CALLBACKS**/
    SERCOM7_USART_ReadCallbackRegister(ESP_CALLBACK, 0);    //Registered ESP Callback
    SERCOM2_USART_ReadCallbackRegister(AC_DC_METER_CALLBACK,0); //Registered AC_DC_METER Callback
    SERCOM6_USART_ReadCallbackRegister(HMI_CALLBACK, 0);    //Registered HMI Callback
    SERCOM1_USART_ReadCallbackRegister(RFID_CALLBACK,0);

    /**REGISTERING GPIO/EIC CALLBACKS**/
    EIC_CallbackRegister(EIC_PIN_10,PHASE1_NOTIFICATION, 0);//Registered Phase1 GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_11,PHASE2_NOTIFICATION, 0);//Registered Phase2 GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_1,EMERGENCY_BTN,0);        //Registered Emergency button GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_2,SPD_DETECTION,0);        //Registered SPD GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_7,IMD_NOTIFICATION,0);     //Registered IMD GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_8,SMOKE_DETECTION,0);      //Registered SMOKE detection GPIO input interuppt Callback
    EIC_CallbackRegister(EIC_PIN_9,LIMIT_SWITCH,0);         //Registered LIMIT Switch GPIO input intruppt callback
    EIC_CallbackRegister(EIC_PIN_5,RGB1_AVAILABLITY_t,0);   //Registered RGB1 SENSE GPIO input intruppt callback
    EIC_CallbackRegister(EIC_PIN_6,RGB2_AVAILABLITY_t,0);   //Registered RGB1 SENSE GPIO input intruppt callback    
    
    /**QUEUES CREATIONS**/
    ESP_QUEUE = xQueueCreate(1, sizeof(ESP_SEND_Q));        //ESP QUEUE CREATION
    HMI_QUEUE = xQueueCreate(1, sizeof(HMI_SEND_Q));        //HMI QUEUE CREATION
    AC_DC_METER_QUEUE = xQueueCreate(1,sizeof(AC_DC_METER_SEND_Q)); //AC_DC_METER QUEUE CREATION
    RGB_QUEUE = xQueueCreate(1,sizeof(RGB_SEND_Q));         //RGB QUEUE CREATION
    RGB1_QUEUE = xQueueCreate(1,sizeof(RGB1_SEND_Q));         //RGB QUEUE CREATION
    
    /**TIMERS CREATION**/
    mains_timer = xTimerCreate("mains_timer",1000,pdFALSE,(void *)0,MainsCallback);  
    
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
    xTaskCreate(StartMACHINE_STATETask,"StartMACHINE_STATETask",128,NULL,1,&MACHINE_STATEHandle);                           //MACHINE STATE TASK CREATION
    xTaskCreate(StartRGB_SENDTask,"StartRGB_SENDTask",512,NULL,1,&RGB_SEND_TaskHandle);                                     //RGB TASK CREATION
    xTaskCreate(StartPOWERBANK_MONITOR_Task,"StartPOWERBANK_MONITOR_Task",256,NULL,1,&POWERBANK_MONITOR_TaskHandle);
    xTaskCreate(StartRFID_RECEIVETask,"StartRFID_RECEIVETask",256,NULL,1,&RFID_RECEIVEHandle);
    xTaskCreate(StartRFID_SENDTask,"StartRFID_SENDTask",256,NULL,1,&RFID_SENDHandle);
             
    /**ENABLING SERCOM Intruppt with their receiving size**/
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);                    //Enabling ESP receiving Intruppt
    SERCOM2_USART_Read(AC_DC_METER_RX_DATA, AC_DC_METER_RECEIVING_SIZE);    //Enabling AC_DC_METER receiving Intruppt
    SERCOM6_USART_Read(HMI_RX_DATA, HMI_RECEIVING_SIZE);                    //Enabling HMI receiving Intruppt
    SERCOM1_USART_Read(RFID_DATA,RFID_WAKEUP_RX_SIZE);
    
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
    vTaskSuspend(MACHINE_STATEHandle);                                      //MACHINE STATE TASK SUSPENDED
    vTaskSuspend(POWERBANK_MONITOR_TaskHandle);
    vTaskSuspend(RFID_RECEIVEHandle);
    vTaskSuspend(RFID_SENDHandle);
    
    /**STARTING SCHEDULER**/
    vTaskStartScheduler();
    
    while ( true )
    {
        //Command never reaches HERE.
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

void MainsCallback(TimerHandle_t xTimer) 
{
    if(PHASE_VALUE_Get())
    {
        POWER_BANK_RELAY_Set();
        vTaskResume(POWERBANK_MONITOR_TaskHandle);
    }
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

/**CALLBACK FUNCTION FOR RFID**/
void RFID_CALLBACK(uintptr_t context)
{
    vTaskResume(RFID_RECEIVEHandle);
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
    xTimerStart(mains_timer, 1000);
//    printf("phase1  = %ld\r\n",tick1);
}

/**CALLBACK FUNCTION FOR PHASE2 GPIO INTRUPPT DETECTION**/
static void PHASE2_NOTIFICATION(uintptr_t context)
{   
    tick2 = xTaskGetTickCount(); 
    xTimerStart(mains_timer, 1000);
//    printf("phase 2 = %ld\r\n",tick2);
}

/**CALLBACK FUNCTION FOR EMERGENCY BUTTON**/
static void EMERGENCY_BTN(uintptr_t context)
{
    //TODO : Notify to esp, HMI & change state of SYSTEM by suspending requried task and changig machine state. Also change LED status
    printf("EMERGENCY BUTTON PRESSED\r\n");
    if(EMERGENCY_BUT_Get())                     //Condition to check if Emergency button is pressed HIGH or LOW
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = RED;
        rgb1msg.RGB1_DATA[0] = RED;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Emergency_button_t = NOT_OK;            //Setting emergency Button Flag to NOT_OK
        printf("BUTTON HIGH\r\n");
    }
    else
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = GREEN_BLINK;
        rgb1msg.RGB1_DATA[0] = GREEN_BLINK;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Emergency_button_t = OK;                 //Setting emergency Button Flag to OK
        printf("BUTTON LOW\r\n");
    }
}

/**CALLBACK FUNCTION FOR SPD**/
static void SPD_DETECTION(uintptr_t context)
{
    //TODO : Notify to esp, HMI & change state of SYSTEM by suspending requried task and changig machine state. Also change LED status
    printf("SPD DETECTED\r\n");
    if(SPD_RESPONSE_Get())                     //Condition to check if SPD is HIGH or LOW
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = GREEN_BLINK;
        rgb1msg.RGB1_DATA[0] = GREEN_BLINK;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        spd_detection_t = OK;            //Setting spd Flag to OK
        printf("SPD HIGH\r\n");
    }
    else
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = RED;
        rgb1msg.RGB1_DATA[0] = RED;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        spd_detection_t = NOT_OK;                 //Setting SPD Flag to NOT_OK
        printf("SPD LOW\r\n");
    }
}

/**CALLBACK FUNCTION FOR IMD**/
static void IMD_NOTIFICATION(uintptr_t context)
{
    //TODO : Notify to esp, HMI , PLC and change system state.
    printf("IMD DETECTED\r\n");
    if(IMD_RESPONSE_Get())                          //Condition for check voltage level IMD GPIO 
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = RED;
        rgb1msg.RGB1_DATA[0] = RED;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Imd_t = NOT_OK;                             //Setting IMD FLAG to NOT_OK.
        printf("IMD PIN HIGH\r\n");
    }
    else
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = GREEN_BLINK;
        rgb1msg.RGB1_DATA[0] = GREEN_BLINK;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Imd_t = OK;                                 //Setting IMD FLAG to OK.
        printf("IMD PIN LOW\r\n");
    }
    
}

/**CALLBACK FUNCTION FOR SMOKE DETECTION**/
static void SMOKE_DETECTION(uintptr_t context)
{
    //TODO : Notify to esp, HMI & change state of SYSTEM by suspending requried task and changig machine state. Also change LED status
    printf("SMOKE DETECTED\r\n");
    if(SMOKE_RESPONSE_Get())                        //Condition to check voltage level of SMOKE GPIO PIN.
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = RED;
        rgb1msg.RGB1_DATA[0] = RED;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Smoke_Detection_t = NOT_OK;                 //Setting SMOKE Detected flag to NOT_OK.
        printf("SMOKE PIN HIGH\r\n");
    }
    else
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = GREEN_BLINK;
        rgb1msg.RGB1_DATA[0] = GREEN_BLINK;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Smoke_Detection_t = OK;                     //Setting SMOKE Detected flag to OK.
        printf("SMOKE PIN LOW\r\n");
    }
}

/**CALLBACK FUNCTION FOR LIMIT SWITCH**/
static void LIMIT_SWITCH(uintptr_t context)
{
    //TODO : Notify to esp, HMI & change state of SYSTEM by suspending requried task and changig machine state. Also change LED status
    printf("LIMIT SWITCH PRESSED\r\n");
    if(LIMIT_BTN_Get())                         //Condition to check it if LIMIT switch pressed HIGH or lOW
    {
        Limit_switch_t = OK;                    //Setting Limit Switch Button to OK.
        printf("GATE CLOSED\r\n");
    }
    else
    {
        RGB_SEND_Q    rgbmsg;
        RGB1_SEND_Q    rgb1msg;
        rgbmsg.RGB_DATA[0] = RED;
        rgb1msg.RGB1_DATA[0] = RED;
        xQueueSend(RGB_QUEUE,&rgbmsg,1);
        xQueueSend(RGB1_QUEUE,&rgb1msg,1);
        Limit_switch_t = NOT_OK;                //Setting Limit Switch Button to NOT_OK.
        printf("GATE OPEN\r\n");
    }
}

static void RGB1_AVAILABLITY_t(uintptr_t context)
{
    if(RGB1_AVAILABLITY_Get())
    {
        printf("RGB1 AVAILABLE\r\n");
    }
    else
    {//TODO : NOTIFY ESP AND HMI
        printf("RGB1 NOT AVAILABLE\r\n");
    }
}

static void RGB2_AVAILABLITY_t(uintptr_t context)
{
    if(RGB2_AVAILABLITY_Get())
    {
        printf("RGB2 AVAILABLE\r\n");
    }
    else
    {//TODO : NOTIFY ESP AND HMI
        printf("RGB2 NOT AVAILABLE\r\n");
    }
}

/**Function for default TASK**/
void StartDefaultTask(void * argument)
{
    static int count =0;                          //Variable to store task count
    uint32_t difference1 =0;                      //Variables to store difference between recorded time
    RGB_SEND_Q    rgbmsg;
    RGB1_SEND_Q    rgb1msg;
    rgbmsg.RGB_DATA[0] = YELLOW;
    rgb1msg.RGB1_DATA[0] = YELLOW;
    xQueueSend(RGB_QUEUE,&rgbmsg,1);
    xQueueSend(RGB1_QUEUE,&rgb1msg,1);
    xTimerStart(mains_timer, 1000);
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
                rgbmsg.RGB_DATA[0] = RED;
                rgb1msg.RGB1_DATA[0] = RED;
                xQueueSend(RGB_QUEUE,&rgbmsg,1);
                xQueueSend(RGB1_QUEUE,&rgb1msg,1);
                printf("OUT PHASE\r\n");                
            }
            count = 0;                          //Resetting count variable
        }
        if(MACHINE_STATE == PHASE_CHECK_STATE)
        {
            count++;                            //incrementing count variable
        }
    vTaskDelay(1000);
    }
}

/**Function for STATE MACHINE TASK**/
void StartMACHINE_STATETask(void * argument)
{
    MACHINE_STATE = SELF_TEST_STATE;               //Changing state of machine from Phase detection to normal state
    RGB_SEND_Q    rgbmsg;
    RGB1_SEND_Q    rgb1msg;
    static uint8_t RGB_STATE = RED;
    while(1)
    {
        switch(MACHINE_STATE)                   //Maintaining machine state condition
        {
            case    NORMAL_STATE:               //Normal STATE
                break;
            case    SELF_TEST_STATE:            //Self test STATE
                    rgbmsg.RGB_DATA[0] = RGB_STATE;
                    rgb1msg.RGB1_DATA[0] = RGB_STATE;
                    xQueueSend(RGB_QUEUE,&rgbmsg,1);
                    xQueueSend(RGB1_QUEUE,&rgb1msg,1);
                    RGB_STATE = RGB_STATE+0x01;
                    if(RGB_STATE > RED_MAGENTA)
                    {
                        MACHINE_STATE = RUNNING_STATE;
                        vTaskResume(TEMP_ADC_TaskHandle);
                        vTaskResume(RFID_SENDHandle);
                    }
                    vTaskDelay(89);
                break;
            case    RUNNING_STATE:              //Running STATE
                    rgbmsg.RGB_DATA[0] = GREEN_BLINK;
                    rgb1msg.RGB1_DATA[1] = GREEN_BLINK;
                    xQueueSend(RGB_QUEUE,&rgbmsg,1);
                    xQueueSend(RGB1_QUEUE,&rgb1msg,1);
                    
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

     if(ESP_RX_DATA[0] == 0x61)                 //Checking condition if data received from esp is aa to goto to bootloader
    {
            ramStart[0] = BTL_TRIGGER_PATTERN;  //Storing bootloader trigger pattern in starting of ram address.
            ramStart[1] = BTL_TRIGGER_PATTERN;  //Storing bootloader trigger pattern in starting of ram address.
            ramStart[2] = BTL_TRIGGER_PATTERN;  //Storing bootloader trigger pattern in starting of ram address.
            ramStart[3] = BTL_TRIGGER_PATTERN;  //Storing bootloader trigger pattern in starting of ram address.
            //NVIC_SystemReset();
    }
    SERCOM7_USART_Read(ESP_RX_DATA, ESP_RECEIVING_SIZE);    //Enabling SERCOM7 read intruppt.
    vTaskSuspend(ESP_RECEIVE_TaskHandle);       //Suspending receiveing task.
    vTaskDelay(1);
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
void StartPOWERBANK_MONITOR_Task(void *argument)
{
    static float POWER_BANK_ADC_READING;
    static float POWER_BANK_ADC_VOLTS;
    static float POWER_BANK_ACTUAL_VOLT;
    ADC0_Enable();
    while(1)
    {
        ADC0_ChannelSelect( (ADC_POSINPUT) ADC_POSINPUT_AIN1, (ADC_NEGINPUT) ADC_NEGINPUT_GND );
        ADC0_ConversionStart();
        while(!ADC0_ConversionStatusGet());
        POWER_BANK_ADC_READING=ADC0_ConversionResultGet();
        POWER_BANK_ADC_VOLTS = ((3.3/1024)*POWER_BANK_ADC_READING);
        POWER_BANK_ACTUAL_VOLT = map(POWER_BANK_ADC_VOLTS,0.5,2.65,0,12);
        printf("%f\r\n",POWER_BANK_ACTUAL_VOLT);
        POWER_BANK_ADC_READING = 0;
        vTaskDelay(100);
    }
}


/**Function for TEMPERATURE ADC TASK**/
void StartTEMP_ADCTask(void *argument)
{
    static   float TEMPERATURE_ADC_RAW_NEW;
    static   float TEMPERATURE_VOLTAGE;
    float TEMPERATURE_SENSOR_DATA =0;
    ADC0_Enable();
    while(1)
    {
        ADC0_ChannelSelect( (ADC_POSINPUT) ADC_POSINPUT_AIN0, (ADC_NEGINPUT) ADC_NEGINPUT_GND );
        ADC0_ConversionStart();
        while(!ADC0_ConversionStatusGet());
        TEMPERATURE_ADC_RAW_NEW=ADC0_ConversionResultGet();

        // calculate temperature 
        TEMPERATURE_VOLTAGE = (TEMPERATURE_ADC_RAW_NEW - 7.65)/309.05;
        TEMPERATURE_SENSOR_DATA = ((TEMPERATURE_VOLTAGE - 2.62)/-13.75)*1000;
        printf("PCB_TEMPERATURE = %f\r\n",TEMPERATURE_SENSOR_DATA);
//        TEMPERATURE_VOLTAGE = ((3.3/1024)*TEMPERATURE_ADC_RAW_NEW);
//        TEMPERATURE_SENSOR_DATA = map(TEMPERATURE_VOLTAGE,0.5,2.65,150,0);
//        printf("%f\r\n",TEMPERATURE_SENSOR_DATA);
        TEMPERATURE_VOLTAGE = 0;
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


void StartRFID_SENDTask(void * argument)
{
    rfid_detection_t = NOT_OK;
    uint8_t RFID_WAKEUP[] = {0x55,0x55,0x00,0x00,0x00,0x00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00};
    uint8_t RFID_GET_FIRMWARE_V[] = {0x00,0x00,0xFF,0x02,0xFE,0xD4,0x02,0x2A};
    uint8_t RFID_READ_ID[] = {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x02,0x00,0xE0,0x00};
    

    while(1)
    {
            switch(CURRENT_RFID_RX_SIZE)
            {
                case RFID_WAKEUP_RX_SIZE:
                        SERCOM1_USART_Write(RFID_WAKEUP,sizeof(RFID_WAKEUP));
                        while(!(SERCOM1_USART_TransmitComplete()));
                    break;
                case RFID_FIRMWARE_RX_SIZE:
                        SERCOM1_USART_Write(RFID_GET_FIRMWARE_V,sizeof(RFID_GET_FIRMWARE_V));
                        while(!(SERCOM1_USART_TransmitComplete()));
                    break;
                case RFID_ID_RX_SIZE:
                        SERCOM1_USART_Write(RFID_READ_ID,sizeof(RFID_READ_ID));
                        while(!(SERCOM1_USART_TransmitComplete()));
                    break;
            }
            
        
        vTaskDelay(100);
    }
}

void StartRFID_RECEIVETask(void * argument)
{
    uint8_t WAKEUP_RESPONSE[] = {0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0xFF,0x02,0xFE,0xD5,0x15,0x16,0x00};
    uint8_t FIRMWARE_RESPONSE[] = {0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0xFF,0x06,0xFA,0xD5,0x03,0x32,0x01,0x06,0x07,0xE8,0x00};
    static uint8_t RFID_ID_RECEIVED[4] = {0};
    RGB_SEND_Q    rgbmsg;
    RGB1_SEND_Q    rgb1msg;
    while(1)
    {
        switch(CURRENT_RFID_RX_SIZE)
        {
            case RFID_WAKEUP_RX_SIZE:
                    for(int i =0;i<CURRENT_RFID_RX_SIZE;i++)
                    {
                        if(RFID_DATA[i] != WAKEUP_RESPONSE[i])
                        {
                            rfid_detection_t = NOT_OK;
                            rgbmsg.RGB_DATA[0] = RED;
                            rgb1msg.RGB1_DATA[0] = RED;
                            xQueueSend(RGB_QUEUE,&rgbmsg,1);
                            xQueueSend(RGB1_QUEUE,&rgb1msg,1);
                        }
                        else
                        {
                            rfid_detection_t = OK;
                            CURRENT_RFID_RX_SIZE = RFID_FIRMWARE_RX_SIZE;
                        }
                    }
                break;
                
            case RFID_FIRMWARE_RX_SIZE:
                    for(int i =0;i<CURRENT_RFID_RX_SIZE;i++)
                    {
                        if(RFID_DATA[i] != FIRMWARE_RESPONSE[i])
                        {
                            rfid_detection_t = NOT_OK;
                            rgbmsg.RGB_DATA[0] = RED;
                            rgb1msg.RGB1_DATA[0] = RED;
                            xQueueSend(RGB_QUEUE,&rgbmsg,1);
                            xQueueSend(RGB1_QUEUE,&rgb1msg,1);
                        }
                        else
                        {
                            rfid_detection_t = OK;
                            CURRENT_RFID_RX_SIZE = RFID_ID_RX_SIZE;
                        }
                    }
                break;
                
            case RFID_ID_RX_SIZE:
                    if((RFID_DATA[9] == RFID_INDEX9_DATA)&&(RFID_DATA[10] == RFID_INDEX10_DATA)&&(RFID_DATA[11] == RFID_INDEX11_DATA)&&(RFID_DATA[12] == RFID_INDEX12_DATA)&&(RFID_DATA[13] == RFID_INDEX13_DATA))
                    {
                        RFID_ID_RECEIVED[0] = RFID_DATA[19]; 
                        RFID_ID_RECEIVED[1] = RFID_DATA[20]; 
                        RFID_ID_RECEIVED[2] = RFID_DATA[21]; 
                        RFID_ID_RECEIVED[3] = RFID_DATA[22]; 
                        printf("ID's : %d %d %d %d\r\n",RFID_ID_RECEIVED[0],RFID_ID_RECEIVED[1],RFID_ID_RECEIVED[2],RFID_ID_RECEIVED[3]);
                    }
                    else
                    {
                        rfid_detection_t = NOT_OK;
                    }
                break;
        }
        memset(RFID_ID_RECEIVED,0,4);
        memset(RFID_DATA,0,sizeof(RFID_DATA));
        vTaskSuspend(RFID_RECEIVEHandle);
        vTaskDelay(1);
    }
}


/**Function for RGB SEND TASK**/
void StartRGB_SENDTask(void * argument)
{
    RGB_SEND_Q RGBmsg;
    RGB1_SEND_Q RGB1msg;
    static uint8_t state = OK;
    uint16_t loop = 65535;
    while(1)
    {
        if(xQueueReceive(RGB_QUEUE,&RGBmsg,0))
        {
        switch(RGBmsg.RGB_DATA[0])
        {
            case RED:
                    RED_t(65535);
                    GREEN_BLUE(0,0);
                break;
            case GREEN:
                    RED_t(0);
                    GREEN_BLUE(0,65535);
                break;
            case BLUE:
                    RED_t(0);
                    GREEN_BLUE(65535,0);
                break;
            case YELLOW:
                    RED_t(65535);
                    GREEN_BLUE(0,65535);
                break;
            case CYAN:
                    RED_t(0);
                    GREEN_BLUE(65535,65535);
                break;
            case MAGENTA:
                    RED_t(65535);
                    GREEN_BLUE(65535,0);
                break;
            case ORANGE:
                    RED_t(65535);
                    GREEN_BLUE(0,32767);
                break;
            case YELLOW_GREEN:
                    RED_t(32767);
                    GREEN_BLUE(0,65535);
                break;
            case CYAN_GREEN:
                    RED_t(0);
                    GREEN_BLUE(32767,65535);
                break;
            case CYAN_BLUE:
                    RED_t(0);
                    GREEN_BLUE(65535,32767);
                break;
            case BLUE_MAGENTA:
                    RED_t(32767);
                    GREEN_BLUE(65535,0);
                break;
            case RED_MAGENTA:
                    RED_t(65535);
                    GREEN_BLUE(32767,0);
                break;
            case GREEN_BLINK:
                switch(state)
                {
                    case OK:
                            RED_t(0);
                            GREEN_BLUE(0,loop);
                            loop = loop - 1000;
                            if(loop <= 30000 )
                            {
                            state = NOT_OK;
                            loop =30000;
                            }
                            vTaskDelay(80);
                        break;
                    case NOT_OK:
                            RED_t(0);
                            GREEN_BLUE(0,loop);
                            loop = loop+1000;
                            if(loop >=30000)
                            {
                            state = OK;
                            loop = 65535;
                            }
                            vTaskDelay(60);
                        break;
                }

                break;
        }
        }
        if(xQueueReceive(RGB1_QUEUE,&RGB1msg,0))
        {
        switch(RGB1msg.RGB1_DATA[0])
        {
            case RED:
                    RED_t1(65535);
                    GREEN_BLUE1(0,0);
                break;
            case GREEN:
                    RED_t1(0);
                    GREEN_BLUE1(65535,0);
                break;
            case BLUE:
                    RED_t1(0);
                    GREEN_BLUE1(0,65535);
                break;
            case YELLOW:
                    RED_t1(65535);
                    GREEN_BLUE1(65535,0);
                break;
            case CYAN:
                    RED_t1(0);
                    GREEN_BLUE1(65535,65535);
                break;
            case MAGENTA:
                    RED_t1(65535);
                    GREEN_BLUE1(0,65535);
                break;
            case ORANGE:
                    RED_t1(65535);
                    GREEN_BLUE1(32767,0);
                break;
            case YELLOW_GREEN:
                    RED_t1(32767);
                    GREEN_BLUE1(65535,0);
                break;
            case CYAN_GREEN:
                    RED_t1(0);
                    GREEN_BLUE1(65535,32767);
                break;
            case CYAN_BLUE:
                    RED_t1(0);
                    GREEN_BLUE1(32767,65535);
                break;
            case BLUE_MAGENTA:
                    RED_t1(32767);
                    GREEN_BLUE1(0,65535);
                break;
            case RED_MAGENTA:
                    RED_t1(65535);
                    GREEN_BLUE1(0,32767);
                break;
            case GREEN_BLINK:
                switch(state)
                {
                    case OK:
                            RED_t1(0);
                            GREEN_BLUE1(loop,0);
                            loop = loop - 1000;
                            if(loop <= 30000 )
                            {
                            state = NOT_OK;
                            loop =30000;
                            }
                            vTaskDelay(80);
                        break;
                    case NOT_OK:
                            RED_t1(0);
                            GREEN_BLUE1(loop,0);
                            loop = loop+1000;
                            if(loop >=30000)
                            {
                            state = OK;
                            loop = 65535;
                            }
                            vTaskDelay(60);
                        break;
                }

                break;
        }
        }
        vTaskDelay(1);
    }
}




void GREEN_BLUE( uint16_t Compare_0_Value ,uint16_t Compare_1_Value)
{
    TC1_CompareStop();

    /* Reset TC */
    TC1_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC1_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC1_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure waveform generation mode */
    TC1_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_NPWM;


    TC1_REGS->COUNT16.TC_CC[0] = Compare_0_Value;
    TC1_REGS->COUNT16.TC_CC[1] = Compare_1_Value;

    /* Clear all interrupt flags */
    TC1_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;


    while((TC1_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
    TC1_CompareStart();
}

void GREEN_BLUE1( uint16_t Compare_0_Value ,uint16_t Compare_1_Value)
{
    TC5_CompareStop();
    /* Reset TC */
    TC5_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC5_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC5_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure waveform generation mode */
    TC5_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_NPWM;


    TC5_REGS->COUNT16.TC_CC[0] = Compare_0_Value;
    TC5_REGS->COUNT16.TC_CC[1] = Compare_1_Value;

    /* Clear all interrupt flags */
    TC5_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;


    while((TC5_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
    TC5_CompareStart();
}


void RED_t( uint16_t Compare_0_Value)
{
     TC0_CompareStop();
   
       /* Reset TC */
    TC0_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC0_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC0_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure waveform generation mode */
    TC0_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_NPWM;


    TC0_REGS->COUNT16.TC_CC[0] = 65533U;
    TC0_REGS->COUNT16.TC_CC[1] = 24U;

    /* Clear all interrupt flags */
    TC0_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;


    while((TC0_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
     
     
     TC0_CompareStart();
}


void RED_t1( uint16_t Compare_0_Value)
{
    TCC4_PWMStop();
    /* Reset TCC */
    TCC4_REGS->TCC_CTRLA = TCC_CTRLA_SWRST_Msk;
    while ((TCC4_REGS->TCC_SYNCBUSY & TCC_SYNCBUSY_SWRST_Msk) != 0U)
    {
        /* Wait for sync */
    }
    /* Clock prescaler */
    TCC4_REGS->TCC_CTRLA = TCC_CTRLA_PRESCALER_DIV64 
                            | TCC_CTRLA_PRESCSYNC_PRESC ;

    TCC4_REGS->TCC_WAVE = TCC_WAVE_WAVEGEN_NPWM | TCC_WAVE_RAMP_RAMP1;


    /* Configure duty cycle values */
    TCC4_REGS->TCC_CC[0] = 500U;
    TCC4_REGS->TCC_CC[1] = 0U;
    TCC4_REGS->TCC_PER = 936U;



    while (TCC4_REGS->TCC_SYNCBUSY != 0U)
    {
        /* Wait for sync */
    }
    TCC4_PWMStart();
}
