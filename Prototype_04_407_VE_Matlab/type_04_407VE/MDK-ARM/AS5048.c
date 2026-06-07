#include "AS5048.h"
#include "spi.h"

#define AS5048_DEADBAND_ENTER 3
#define AS5048_DEADBAND_EXIT 6

static int as5048_abs_int(int value)
{
	return (value < 0) ? -value : value;
}

static int as5048_median3(int a, int b, int c)
{
	if (a > b) {
		int tmp = a;
		a = b;
		b = tmp;
	}
	if (b > c) {
		int tmp = b;
		b = c;
		c = tmp;
	}
	if (a > b) {
		int tmp = a;
		a = b;
		b = tmp;
	}
	return b;
}

AS5048 AS5048s[AS5048_NUMBER];

void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	AS5->spi_number = spi;
	AS5->GPIOx = GPIOx;
	AS5->GPIO_Pin = GPIO_Pin;
	AS5->angle = 0;
	AS5->total_angle = 0;
	AS5->cirle = 0;
	AS5->last_angle = AS5->angle;
	AS5->delta_dis = 0;
	AS5->diff_hist[0] = 0;
	AS5->diff_hist[1] = 0;
	AS5->diff_hist[2] = 0;
	AS5->motion_state = 0;
	AS5->error_status = AS5048_ERR_NONE;
	AS5->diag_counter = 0;
	AS5->agc_value = 0;
}

uint16_t AS5048_ReadRaw(const int AS5048_ID, uint16_t registerAddress)
{
	uint8_t data[4] = {0, 0, 0, 0};
	uint8_t cmd[4] = {0, 0, 0, 0};
	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	cmd[3] = registerAddress & 0xFF;
	cmd[2] = (registerAddress >> 8) & 0xFF;
	cmd[1] = registerAddress & 0xFF;
	cmd[0] = (registerAddress >> 8) & 0xFF;

	HAL_GPIO_WritePin(AS5->GPIOx, AS5->GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(AS5->spi_number, cmd, data, 2, 1000);
	HAL_GPIO_WritePin(AS5->GPIOx, AS5->GPIO_Pin, GPIO_PIN_SET);

	return ((data[1] & 0xFF) << 8) | (data[0] & 0xFF);
}

uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress)
{
	return AS5048_ReadRaw(AS5048_ID, registerAddress) & ~0xC000;
}

void AS5048_getREGValue(const int AS5048_ID)
{
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	uint16_t raw = AS5048_ReadRaw(AS5048_ID, SPI_REG_DATA);

	if (raw & AS5048_EF_BIT) {
		// SPI communication error: set flag, keep last valid angle
		AS5->error_status |= AS5048_ERR_SPI_EF;
	} else {
		// Clear SPI error flag on successful read
		AS5->error_status &= ~AS5048_ERR_SPI_EF;
		AS5->angle = raw & 0x3FFF;
	}

	// Periodic diagnostic check
	AS5->diag_counter++;
	if (AS5->diag_counter >= AS5048_DIAG_CHECK_INTERVAL) {
		AS5->diag_counter = 0;
		AS5048_checkDiagnostics(AS5048_ID);
	}
}

void AS5048_checkDiagnostics(const int AS5048_ID)
{
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	uint16_t diag = AS5048_Read(AS5048_ID, SPI_REG_AGC);

	AS5->agc_value = diag & 0xFF;

	// Clear magnetic diagnostic flags, then set based on current state
	AS5->error_status &= ~(AS5048_ERR_MAG_LOW | AS5048_ERR_MAG_HIGH | AS5048_ERR_CORDIC_OVF);

	if (diag & AS5048_DIAG_COMP_LOW)
		AS5->error_status |= AS5048_ERR_MAG_LOW;
	if (diag & AS5048_DIAG_COMP_HIGH)
		AS5->error_status |= AS5048_ERR_MAG_HIGH;
	if (diag & AS5048_DIAG_CORDIC_OVF)
		AS5->error_status |= AS5048_ERR_CORDIC_OVF;
}

void AS5048_dataUpdate(const int AS5048_ID)
{
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	int diff = AS5->angle - AS5->last_angle;
	int filtered_diff;
	int abs_filtered_diff;

	if (diff > 16000) {
		diff -= 16384;
		AS5->cirle--;
	} else if (diff < -16000) {
		diff += 16384;
		AS5->cirle++;
	}

#if AS5048_RAW_CALIBRATION_MODE
	filtered_diff = diff;
#else

	AS5->diff_hist[0] = AS5->diff_hist[1];
	AS5->diff_hist[1] = AS5->diff_hist[2];
	AS5->diff_hist[2] = diff;
	filtered_diff = as5048_median3(AS5->diff_hist[0], AS5->diff_hist[1], AS5->diff_hist[2]);
	abs_filtered_diff = as5048_abs_int(filtered_diff);

	if (AS5->motion_state != 0U) {
		if (abs_filtered_diff <= AS5048_DEADBAND_ENTER) {
			AS5->motion_state = 0U;
			filtered_diff = 0;
		}
	} else {
		if (abs_filtered_diff < AS5048_DEADBAND_EXIT) {
			filtered_diff = 0;
		} else {
			AS5->motion_state = 1U;
		}
	}
#endif

	AS5->delta_dis = filtered_diff;
	AS5->total_angle += filtered_diff;
	AS5->last_angle = AS5->angle;
}
