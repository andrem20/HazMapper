/* Platform Includes -------------------------------------------------------------------------- */
#include "usart.h"
/* User Includes ------------------------------------------------------------------------------ */
#include "camera.h"
#include "ov7670Reg.h"
#include "usart_support.h"


/* User Defined Variables (External) --------------------------------------------------------- */
I2C_HandleTypeDef *hi2c_camera;
DCMI_HandleTypeDef *hdcmi_camera;
DMA_HandleTypeDef  *hdma_camera;

/* Internally Used Functions  ---------------------------------------------------------------- */
static void ov7670_write (uint8_t regAddr, uint8_t data);
static void ov7670_read (uint8_t regAddr, uint8_t *data);

/* Functions ------------------------------------------------------------------------------ */
static void ov7670_write(uint8_t regAddr, uint8_t value) {
	uint8_t data[2] = { regAddr, value };
	HAL_I2C_Master_Transmit(hi2c_camera, WRITE_ADDR, data, 2, 100);

}


static void ov7670_read (uint8_t regAddr, uint8_t *data){
	HAL_I2C_Master_Transmit(hi2c_camera, WRITE_ADDR, &regAddr, 1, 100);
	HAL_I2C_Master_Receive(hi2c_camera, WRITE_ADDR, data, 1, 100);
}


void camera_init(I2C_HandleTypeDef *hi2c2, DCMI_HandleTypeDef *hdcmi, DMA_HandleTypeDef  *hdma_dcmi){
	hi2c_camera = hi2c2;
	hdcmi_camera = hdcmi;
	hdma_camera = hdma_dcmi;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(100);

	ov7670_write(0x12, 0x80);  // RESET

	uint8_t buffer[4];
	ov7670_read(0x0b, buffer);
	printf("[OV7670] dev id = %02X\n", buffer[0]);
}

void camera_config(){

	stop_Cap();

	ov7670_write(0x12, 0x80); // SCCB Register Reset
	for(int i = 0; OV7670_reg[i][0] != END; i++) {
		ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
		HAL_Delay(1);
	}
}

void startCap(uint32_t capMode, uint32_t frame_buffer, uint32_t size) {
	stop_Cap();

	// Inicializa os buffers
	current_dcmi_ptr = (uint8_t*)frame_buffer;
	uart_pending_ptr = NULL;
	xSemaphoreGive(binSyncUART);	// Semaforo começa cheio <=> UART pronta a enviar

	// Inicia captura contínua via DCMI + DMA
	HAL_DCMI_Start_DMA(hdcmi_camera, capMode, (uint32_t)current_dcmi_ptr, size / 4);
}

void stop_Cap(){
	HAL_DCMI_Stop(hdcmi_camera);
}

/* TASKS ---------------------------------------------------------------------------------- */
void Camera_Task(void* argument){
	uint32_t notify;
	camera_init(&hi2c2, &hdcmi, &hdma_dcmi);
	camera_config();
	startCap(DCMI_MODE_CONTINUOUS, (uint32_t)frame_buffer_A, FRAME_BUFFER_SIZE);
	for(;;){
		xTaskNotifyWait (0, BIT_WAIT_FRAME, &notify, portMAX_DELAY);	// Notification comes from DCMI callback
		if(notify & BIT_WAIT_FRAME){
			if(uart_pending_ptr != NULL){
				for (uint32_t offset = 0; offset < FRAME_BUFFER_SIZE; offset += CHUNK_SIZE) {
					uint16_t bytes_to_send = (FRAME_BUFFER_SIZE - offset) >= CHUNK_SIZE ? CHUNK_SIZE : (FRAME_BUFFER_SIZE - offset);
					xSemaphoreTake(binSyncUART, portMAX_DELAY);

					HAL_UART_Transmit_DMA(&huart3,(uint8_t*) (&uart_pending_ptr[offset]), bytes_to_send);
					xSemaphoreTake(binSyncUART, portMAX_DELAY);
				}
				uart_pending_ptr = NULL;
			}
		}
	}
}
