#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_
/* Platform Includes -------------------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"

#include "dma.h"
#include "i2c.h"
#include "dcmi.h"
/* User Defined Macros -------------------------------------------------------------------- */

//#define USE_CAMERA

#define WRITE_ADDR 0x42
#define READ_ADDR  0x43
#define DCMI_DataRegister 0x28

#define CHUNK_SIZE 65535
#define FRAME_WIDTH   320
#define FRAME_HEIGHT  240
#define PIXEL_PER_WORD 2  // RGB565
#define FRAME_BUFFER_SIZE (FRAME_WIDTH * FRAME_HEIGHT * PIXEL_PER_WORD)

#define BIT_WAIT_FRAME (1<<0)
#define BIT_WAIT_UART (1<<1)
/* User Defined Variables (External) ------------------------------------------------------ */
extern volatile uint8_t frame_buffer_A[FRAME_BUFFER_SIZE];
extern volatile uint8_t frame_buffer_B[FRAME_BUFFER_SIZE];
extern volatile uint8_t* current_dcmi_ptr;
extern volatile uint8_t* uart_pending_ptr;

extern I2C_HandleTypeDef *hi2c_camera;
extern DCMI_HandleTypeDef *hdcmi_camera;
extern DMA_HandleTypeDef  *hdma_camera;

extern SemaphoreHandle_t binSyncUART ;
extern osThreadId_t Camera;
/* Functions ------------------------------------------------------------------------------ */
void camera_init(I2C_HandleTypeDef *hi2c2, DCMI_HandleTypeDef *hdcmi, DMA_HandleTypeDef  *hdma_dcmi);
void camera_config ();
void startCap (uint32_t capMode, uint32_t frame_buffer, uint32_t size);
void stop_Cap();
//void registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));

void Camera_Task(void* argument);
#endif /* INC_CAMERA_H_ */
