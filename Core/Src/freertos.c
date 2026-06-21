/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "ModularCANLib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float Kp_pos;
  float Kp_vel;
  float Ki_vel;
  float alpha; // 参照モデルの極
  float gw;    // DOBのLPFの極
  float J;     // 慣性モーメント
  float D;     // 粘性摩擦係数
  float Ki;    // トルク定数
} ControllerParams;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum {
  Motor_Shoulder=0,
  Motor_Elbow,
  Motor_3508_left,
  Motor_3508_right,
  Motor_2006_xy,
  Motor_2006_z,
};

#define CONTROL_PERIOD_MS 1
#define CONTROL_PERIOD_S  0.001f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile float g_target_positions[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 各軸のDOBやフィルタの前回値を保持する構造体
typedef struct {
  float prev_v_ref_filtered;
  float prev_i_in;
  float integral_e;

  // DOB・フィルタ用の状態変数（追加）
  float dob_state;
  float i_in_filtered;
  float prev_i_in_filtered;
} ControllerState;

ControllerState ctrl_state[6] = {0};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 8192 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId canlib_taskHandle;
uint32_t canlib_taskBuffer[ 1024 ];
osStaticThreadDef_t canlib_taskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

float calc_axis_control(float target_pos, float current_pos, float current_vel,ControllerState* state, const ControllerParams* params);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCanlibTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 8192, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of canlib_task */
  osThreadStaticDef(canlib_task, StartCanlibTask, osPriorityAboveNormal, 0, 1024, canlib_taskBuffer, &canlib_taskControlBlock);
  canlib_taskHandle = osThreadCreate(osThread(canlib_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */

  // micro-ROS configuration
  rmw_uros_set_custom_transport(
    false,              //Framing disable here.
    "192.168.1.121",    //your Agent's ip address.
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rcl_subscription_t subscriber;
  std_msgs__msg__Float32MultiArray msg; // 受信メッセージ

  // 初期化オプションの生成とDomain IDのセット
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 30);

  // supportを初期化
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // ノードの作成
  rclc_node_init_default(&node, "stm32_scara_node", "", &support);

  // Subscriber作成 ("target_joint_positions"トピック)
  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_joint_positions");

  // エグゼキュータの作成
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // コールバック関数の定義
  void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * array_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    // 6軸分のデータが来ているかチェック
    if (array_msg->data.size >= 6) {
      g_target_positions[Motor_Shoulder]   = array_msg->data.data[0];
      g_target_positions[Motor_Elbow]      = array_msg->data.data[1];
      g_target_positions[Motor_3508_left]  = array_msg->data.data[2];
      g_target_positions[Motor_3508_right] = array_msg->data.data[3];
      g_target_positions[Motor_2006_xy]    = array_msg->data.data[4];
      g_target_positions[Motor_2006_z]     = array_msg->data.data[5];
    }
  }

  // Subscriberをエグゼキュータに追加
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  for(;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanlibTask */
/**
* @brief Function implementing the canlib_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanlibTask */
void StartCanlibTask(void const * argument)
{
  /* USER CODE BEGIN StartCanlibTask */

  // 1. 各モータの登録と初期化設定\
  // --- Shoulder (RobStride 02) ---
  // ModularCANLib_DeviceInfo_Type *dev_shoulder = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RobStride_02, Motor_Shoulder);
  // dev_shoulder->hcan = &hcan1; dev_shoulder->node_id = 1;
  // ModularCANLib_DeviceParam_RobStride_Type *param_shoulder = ModularCANLib_RobStride_GetDeviceParam(dev_shoulder);
  // param_shoulder->use_internal_offset = ROBSTRIDE_USE_OFFSET_POS_INITIAL;
  // param_shoulder->target.kp = 0.0f; // トルク制御のみ
  // param_shoulder->target.kd = 0.0f;

  // --- Elbow (RobStride 05 Edu) ---
  ModularCANLib_DeviceInfo_Type *dev_elbow = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RobStride_05_Edu, Motor_Elbow);
  dev_elbow->hcan = &hcan2; dev_elbow->node_id = 1;
  ModularCANLib_DeviceParam_RobStride_Type *param_elbow = ModularCANLib_RobStride_GetDeviceParam(dev_elbow);
  param_elbow->use_internal_offset = ROBSTRIDE_USE_OFFSET_POS_INITIAL;
  param_elbow->target.kp = 0.0f; // トルク制御のみ
  param_elbow->target.kd = 0.0f;

  // // --- 3508 Left (C620) ---
  // ModularCANLib_DeviceInfo_Type *dev_3508_l = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RoboMas_C620, Motor_3508_left);
  // dev_3508_l->hcan = &hcan2; dev_3508_l->node_id = 1;
  // ModularCANLib_RoboMas_GetDeviceParam(dev_3508_l)->ctrl_type = ROBOMAS_CTRL_CURRENT;
  //
  // // --- 3508 Right (C620) ---
  // ModularCANLib_DeviceInfo_Type *dev_3508_r = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RoboMas_C620, Motor_3508_right);
  // dev_3508_r->hcan = &hcan2; dev_3508_r->node_id = 2;
  // ModularCANLib_RoboMas_GetDeviceParam(dev_3508_r)->ctrl_type = ROBOMAS_CTRL_CURRENT;
  //
  // // --- 2006 XY (C610) ---
  // ModularCANLib_DeviceInfo_Type *dev_2006_xy = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RoboMas_C610, Motor_2006_xy);
  // dev_2006_xy->hcan = &hcan2; dev_2006_xy->node_id = 3;
  // ModularCANLib_RoboMas_GetDeviceParam(dev_2006_xy)->ctrl_type = ROBOMAS_CTRL_CURRENT;
  //
  // // --- 2006 Z (C610) ---
  // ModularCANLib_DeviceInfo_Type *dev_2006_z = ModularCANLib_DeviceInfo_Struct_Init(ModularCANLib_DeviceType_RoboMas_C610, Motor_2006_z);
  // dev_2006_z->hcan = &hcan2; dev_2006_z->node_id = 4;
  // ModularCANLib_RoboMas_GetDeviceParam(dev_2006_z)->ctrl_type = ROBOMAS_CTRL_CURRENT;

  // 全デバイス & CAN初期化
  ModularCANLib_AllDevice_And_CANSystem_Init();
  ModularCANLib_WaitForConnect();

  // 起動処理
  // Robstride_PresetParameters(dev_shoulder);
  Robstride_PresetParameters(dev_elbow);
  // Robstride_ControlEnable(dev_shoulder);
  Robstride_ControlEnable(dev_elbow);
  // RoboMas_ControlEnable(dev_3508_l);
  // RoboMas_ControlEnable(dev_3508_r);
  // RoboMas_ControlEnable(dev_2006_xy);
  // RoboMas_ControlEnable(dev_2006_z);

  // 2. 各軸の制御パラメータの初期化（※実機に合わせて値を調整してください）
  // ControllerParams p_shoulder = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.01f, .D = 0.001f, .Ki = 1.0f };
  ControllerParams p_elbow    = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.005f, .D = 0.001f, .Ki = 1.0f };
  // ControllerParams p_3508_l   = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.002f, .D = 0.001f, .Ki = 0.3f };
  // ControllerParams p_3508_r   = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.002f, .D = 0.001f, .Ki = 0.3f };
  // ControllerParams p_2006_xy  = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.001f, .D = 0.001f, .Ki = 0.18f };
  // ControllerParams p_2006_z   = { .Kp_pos = 10.0f, .Kp_vel = 0.5f, .Ki_vel = 0.1f, .alpha = 50.0f, .gw = 100.0f, .J = 0.001f, .D = 0.001f, .Ki = 0.18f };

  // 1kHz制御ループの準備
  TickType_t xLastWakeTime = osKernelSysTick();

  for(;;)
  {
    // 厳密に1ms周期で実行する
    osDelayUntil(&xLastWakeTime, CONTROL_PERIOD_MS);

    // ========================================================
    // 制御計算と指令値のセット
    // ========================================================
    float req_i; // 計算された目標電流値

    // --- Shoulder ---
    // req_i = calc_axis_control(g_target_positions[Motor_Shoulder], param_shoulder->feedback.position, param_shoulder->feedback.velocity, &ctrl_state[Motor_Shoulder], &p_shoulder);
    // param_shoulder->target.torque_ff = req_i * p_shoulder.Ki; // 電流からトルクに変換して代入

    // --- Elbow ---
    req_i = calc_axis_control(g_target_positions[Motor_Elbow], param_elbow->feedback.position, param_elbow->feedback.velocity, &ctrl_state[Motor_Elbow], &p_elbow);
    param_elbow->target.torque_ff = req_i * p_elbow.Ki;

    // // --- 3508 Left ---
    // req_i = calc_axis_control(g_target_positions[Motor_3508_left], ModularCANLib_RoboMas_GetDeviceParam(dev_3508_l)->feedback.position, ModularCANLib_RoboMas_GetDeviceParam(dev_3508_l)->feedback.velocity, &ctrl_state[Motor_3508_left], &p_3508_l);
    // RoboMas_SetTarget(dev_3508_l, req_i); // RoboMasは電流指令をそのまま渡す
    //
    // // --- 3508 Right ---
    // req_i = calc_axis_control(g_target_positions[Motor_3508_right], ModularCANLib_RoboMas_GetDeviceParam(dev_3508_r)->feedback.position, ModularCANLib_RoboMas_GetDeviceParam(dev_3508_r)->feedback.velocity, &ctrl_state[Motor_3508_right], &p_3508_r);
    // RoboMas_SetTarget(dev_3508_r, req_i);
    //
    // // --- 2006 XY ---
    // req_i = calc_axis_control(g_target_positions[Motor_2006_xy], ModularCANLib_RoboMas_GetDeviceParam(dev_2006_xy)->feedback.position, ModularCANLib_RoboMas_GetDeviceParam(dev_2006_xy)->feedback.velocity, &ctrl_state[Motor_2006_xy], &p_2006_xy);
    // RoboMas_SetTarget(dev_2006_xy, req_i);
    //
    // // --- 2006 Z ---
    // req_i = calc_axis_control(g_target_positions[Motor_2006_z], ModularCANLib_RoboMas_GetDeviceParam(dev_2006_z)->feedback.position, ModularCANLib_RoboMas_GetDeviceParam(dev_2006_z)->feedback.velocity, &ctrl_state[Motor_2006_z], &p_2006_z);
    // RoboMas_SetTarget(dev_2006_z, req_i);
  }
  /* USER CODE END StartCanlibTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
float calc_axis_control(float target_pos, float current_pos, float current_vel,
                        ControllerState* state, const ControllerParams* params)
{
  float dt = CONTROL_PERIOD_S;

  // 1. 位置のP制御で目標速度を生成
  float v_ref = params->Kp_pos * (target_pos - current_pos);

  // 2. 参照モデル F(s) = alpha / (s + alpha)
  float v_ref_f = (state->prev_v_ref_filtered + params->alpha * dt * v_ref) / (1.0f + params->alpha * dt);
  state->prev_v_ref_filtered = v_ref_f;

  // 3. 速度のPID制御 (FB電流)
  float e = v_ref_f - current_vel;
  state->integral_e += e * dt;
  float i_fb = params->Kp_vel * e + params->Ki_vel * state->integral_e;

  // 4. フィードフォワード制御 (FF電流)
  float i_ff = (params->D / params->Ki) * v_ref_f;

  // 5. DOB (外乱オブザーバ)
  float M = params->J / params->Ki;
  float D_ki = params->D / params->Ki;

  float dob_v_term = params->gw * (M * current_vel) - state->dob_state;
  state->dob_state += dt * params->gw * (dob_v_term + (D_ki * current_vel));

  state->i_in_filtered = (state->prev_i_in_filtered + params->gw * dt * state->prev_i_in) / (1.0f + params->gw * dt);
  state->prev_i_in_filtered = state->i_in_filtered;

  float i_dis_hat = state->i_in_filtered - dob_v_term;

  // 6. 制御入力の合成
  float i_model_in = i_ff + i_fb + i_dis_hat;

  // 次回ループのための保存
  state->prev_i_in = i_model_in;

  return i_model_in; // 計算された要求電流[A]
}
/* USER CODE END Application */

