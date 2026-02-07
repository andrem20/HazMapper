/* Platform Includes BEGIN ------------------------------------------------------------ */
#include "i2c.h"
#include "string.h"
/* User Includes BEGIN ---------------------------------------------------------------- */
#include "sensors_i2c.h"

/* MACROS definition -----------------------------------------------------------------  */
/* LCD */
#define LCD_ADDRESS	0x27
#define LCD_RS		0x01 	// Register Select (Low: data received = instructions; High: data receivedd = display data
#define LCD_ENABLE	0x04

#define LCD_INIT_4BIT	(0x30 << 1)
#define LCD_CONFIG 		0x28	// 4 bits, 2 linhas, 5x8 font
#define LCD_DISPLAY_ON 	0x0C	// display ON, cursor OFF
#define LCD_ENTRY_MODE 	0x06
#define LCD_CLEAR 		0x01
#define LCD_BACKLIGHT	0x08

#define LCD_4BIT_SEND_MASK 0xF0
/* Internally Used Function ----------------------------------------------------------- */
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(uint8_t data);
static void LCD_Send4Bits(uint8_t data);

/* Functions -------------------------------------------------------------------------- */
void LCD_Init(void) {

    LCD_Send4Bits(LCD_INIT_4BIT);
  //  osDelay(5);
    LCD_Send4Bits(LCD_INIT_4BIT);
  //  osDelay(5);
    LCD_Send4Bits(LCD_INIT_4BIT);
   // osDelay(5);
    LCD_Send4Bits(LCD_INIT_4BIT);
    // 4 bit mode

    // Basic Config
    LCD_SendCommand(LCD_CONFIG);
    LCD_SendCommand(LCD_DISPLAY_ON);
    LCD_SendCommand(LCD_ENTRY_MODE);
    LCD_SendCommand(LCD_CLEAR);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendCommand(addr);
}

static void LCD_SendCommand(uint8_t cmd) {
    LCD_Send4Bits((cmd & LCD_4BIT_SEND_MASK) | LCD_BACKLIGHT);
    LCD_Send4Bits(((cmd << 4) & LCD_4BIT_SEND_MASK) | LCD_BACKLIGHT);
}

static void LCD_SendData(uint8_t data) {
    LCD_Send4Bits((data & 0xF0) | LCD_RS | LCD_BACKLIGHT);
    LCD_Send4Bits(((data << 4) & 0xF0) | LCD_RS | LCD_BACKLIGHT);
}

static void LCD_Send4Bits(uint8_t data) {
    uint8_t data_t;
    xSemaphoreTake(i2cmutex, portMAX_DELAY);
    data_t = data | LCD_ENABLE;
    HAL_I2C_Master_Transmit_IT(&hi2c1, LCD_INIT_4BIT, &data_t, 1);

    data_t = data & ~LCD_ENABLE;
    HAL_I2C_Master_Transmit_IT(&hi2c1, LCD_INIT_4BIT, &data_t, 1);
    xSemaphoreGive(i2cmutex);
}

/* TASK ------------------------------------------------------------------------------------ */
void LCD_Task(void* argument){
	void LCD_Init(void);
	char data [128];
	for(;;){
		xQueueReceive(LCD_DataQueue, &data, portMAX_DELAY);
		for(int i = 0; i < strlen(data); i++)
			LCD_SendData(i);
	}
}
