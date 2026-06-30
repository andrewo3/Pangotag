#include "libs.h"

extern const uint8_t bmi270_config_file[];

struct bmi2_dev bmi270 = {0};

struct bmi2_fifo_frame fifo = {0};

extern void delay_us(uint32_t us, void *intf_ptr);

uint32_t startTime;
int16_t bmi2_data[MAX_FIFO_FRAMES * 3 * 2];
uint32_t payload_size;

uint8_t fifo_buffer[1024];

volatile uint8_t gps_char;
uint8_t nmea_recv[128];
uint8_t saved_rmc[128];
volatile uint16_t gps_idx = 0;
volatile uint8_t rmc_ready = 0;
uint32_t totalEnergyUsed = 0;
uint16_t lastAcc = 0;

int8_t bmi270_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    int res = HAL_I2C_Mem_Write(&hi2c2,
                              BMI2_I2C_PRIM_ADDR << 1,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t*)data,
                              len,
                              HAL_MAX_DELAY);
    if (res != HAL_OK) {
      return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

int8_t bmi270_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    int res = HAL_I2C_Mem_Read(&hi2c2,
                             BMI2_I2C_PRIM_ADDR << 1,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             len,
                             HAL_MAX_DELAY);
    if (res != HAL_OK) {
      return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

int initBMI270() {
    bmi270.read = bmi270_read;
    bmi270.write = bmi270_write;
    bmi270.delay_us = delay_us;
    bmi270.intf = BMI2_I2C_INTF;
    bmi270.config_file_ptr = nullptr;
    printf("bmi270 cfg @ %p: 0x%02x 0x%02x\n",bmi270_config_file, bmi270_config_file[0], bmi270_config_file[1]);
    uint8_t chip_id = 0x00;
    int res = HAL_I2C_Mem_Read(&hi2c2,
                    BMI2_I2C_PRIM_ADDR << 1,
                    BMI2_CHIP_ID_ADDR,
                    I2C_MEMADD_SIZE_8BIT,
                    &chip_id,
                    1,
                    HAL_MAX_DELAY);
    printf("Chip ID: 0x%02x, result success: %i\n",chip_id,res);

    int8_t rslt = bmi270_init(&bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("Failed to initialize BMI270: err %i\n",rslt);
        return rslt;
    }

    rslt = bmi2_soft_reset(&bmi270);
    bmi270.delay_us(2000,&bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("Failed to soft reset BMI270: err %i\n",rslt);
        return rslt;
    } 

    struct bmi2_sens_config config;
    config.type = BMI2_ACCEL;
    bmi2_get_sensor_config(&config,1,&bmi270);
    config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config.cfg.acc.range = BMI2_ACC_RANGE_4G;
    config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

    bmi2_set_sensor_config(&config, 1, &bmi270);

    config.type = BMI2_GYRO;
    bmi2_get_sensor_config(&config, 1, &bmi270);

    config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config.cfg.gyr.range = BMI2_GYR_RANGE_250;
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

    bmi2_set_sensor_config(&config, 1, &bmi270);

    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    rslt = bmi2_sensor_enable(sensor_list, 2, &bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("Failed to enable BMI270: err %i\n",rslt);
        return rslt;
    } 

    uint16_t fifo_config =
    BMI2_FIFO_ACC_EN |
    BMI2_FIFO_GYR_EN |
    BMI2_FIFO_HEADER_EN |
    BMI2_FIFO_TIME_EN;

    rslt = bmi2_set_fifo_config(fifo_config, BMI2_ENABLE, &bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("BMI270: Failed to set FIFO config: err %i\n",rslt);
        return rslt;
    } 

    rslt = bmi2_set_fifo_wm(FIFO_WM_THRESH, &bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("BMI270: Failed to enable FIFO watermark: err %i\n",rslt);
        return rslt;
    }

    struct bmi2_int_pin_config int_cfg;
    struct bmi2_int_pin_cfg pin_cfg;

    int_cfg.pin_type = BMI2_INT1;

    pin_cfg.output_en = BMI2_ENABLE;
    pin_cfg.od        = BMI2_INT_PUSH_PULL;
    pin_cfg.lvl       = BMI2_INT_ACTIVE_HIGH;
    pin_cfg.input_en  = BMI2_DISABLE;
    int_cfg.pin_cfg[0] = pin_cfg;

    rslt = bmi2_set_int_pin_config(&int_cfg, &bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("BMI270: Failed to enable INT1: err %i\n",rslt);
        return rslt;
    }

    rslt = bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT1, &bmi270);
    if (rslt != BMI2_OK) {
        // communication failed
        printf("BMI270: Failed to enable WM interrupts: err %i\n",rslt);
        return rslt;
    } 

    printf("BMI270 Initialization successful!\n");
    return rslt;
}

void initTMP() {
	uint8_t addr = 0x60;
	uint8_t out_bytes[5];
	uint8_t CTRL_REG1 = 0x26;
	uint8_t PT_DATA_CFG = 0x13;
	uint8_t STATUS = 0x00;

	//enable data flags
	Write_I2C_Reg(addr,PT_DATA_CFG,0x7,2);
	//0x3A to CTRL_REG1 to activate one reading
	Write_I2C_Reg(addr,CTRL_REG1,0x39,2);
}

//non-blocking tmp/prs sensor read
void readTMP() {
	uint8_t addr = 0x60;
	uint8_t out_bytes[5];
	uint8_t CTRL_REG1 = 0x26;
	uint8_t PT_DATA_CFG = 0x13;
	uint8_t STATUS = 0x00;
	//log_printf("Status: %02x\r\n",Read_I2C_Reg_NoStop(addr,CTRL_REG1));
    // if data not ready, return immediately
	if ((Read_I2C_Reg(addr,STATUS,2) & 0x0e) != 0x0e) {
		return;
	}
	// read data
	for (int i = 0; i < 5; i++) {
		out_bytes[i] = Read_I2C_Reg_NoStop(addr, i+1,2);
	}
    packet tmp_data = {0};

    tmp_data.type = PACKET_TMP;
    tmp_data.timestamp = HAL_GetTick() - startTime;
    tmp_data.len = 5;
    tmp_data.payload = out_bytes;
    int res = writePacket(tmp_data);
	if (res != FR_OK) {
        printf("Failed to write tmp sensor data: err %i\n", res);
    }
}

void readBMI270() {
    uint16_t int_status;
    bmi2_get_int_status(&int_status, &bmi270);
    //printf("FIFO Status: %i\n",int_status);

    uint16_t fifo_len;
    bmi2_get_fifo_length(&fifo_len, &bmi270);
    //printf("fifo length before = %u\n", fifo_len);

    int8_t rslt = bmi2_read_fifo_data(&fifo, &bmi270);
    if (rslt != BMI2_OK) {
        log_printf("Failed to read FIFO Frame: err %i\n",rslt);
        return;
    } else {
        //printf("Successful fifo frame read: %i frames\n",fifo.length);
        bmi2_get_fifo_length(&fifo_len, &bmi270);
        //printf("fifo length after = %u\n", fifo_len);

        struct bmi2_sens_axes_data accel_data[128];
        struct bmi2_sens_axes_data gyro_data[128];

        uint16_t acc_frames = MAX_FIFO_FRAMES;
        uint16_t gyr_frames = MAX_FIFO_FRAMES;

        rslt = bmi2_extract_accel(accel_data, &acc_frames, &fifo, &bmi270);
        if (rslt < BMI2_OK) {
            log_printf("Failed to extract acceleration frame: err %i\n",rslt);
        }
        rslt = bmi2_extract_gyro(gyro_data, &gyr_frames, &fifo, &bmi270);
        if (rslt < BMI2_OK) {
            log_printf("Failed to extract gyroscope frame: err %i\n",rslt);
        }
        //printf("%i, %i frames each - Frame 1: (%i,%i,%i)\n",acc_frames, gyr_frames, gyro_data[0].x,gyro_data[0].y,gyro_data[0].z);
        for (int frame = 0; frame < acc_frames; frame++) {
            bmi2_data[frame * 3] = (int16_t)accel_data[frame].x;
            bmi2_data[1 + frame * 3] = (int16_t)accel_data[frame].y;
            bmi2_data[2 + frame * 3] = (int16_t)accel_data[frame].z;
        }
        for (int frame = 0; frame < gyr_frames; frame++) {
            bmi2_data[acc_frames * 3 + frame * 3] = (int16_t)gyro_data[frame].x;
            bmi2_data[1 + acc_frames * 3 + frame * 3] = (int16_t)gyro_data[frame].y;
            bmi2_data[2 + acc_frames * 3 + frame * 3] = (int16_t)gyro_data[frame].z;
        }
        payload_size = 2 * (3 * acc_frames + 3 * gyr_frames);
        //printf("Decoded %i acc frames and %i gyro frames.\n",acc_frames, gyr_frames);
        packet imu_data = {0};
        imu_data.type = PACKET_IMU;
        imu_data.timestamp = HAL_GetTick() - startTime; // overflow after ~49 days
        imu_data.len = payload_size;
        imu_data.payload = (uint8_t*)bmi2_data;
        writePacket(imu_data);
        bmi2_get_int_status(&int_status, &bmi270);
        //printf("FIFO Status: %i\n",int_status);
    }
}

void GPS_Start()
{
    __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
    int status = HAL_UART_Receive_IT(&huart1, &gps_char, 1);
    printf("start success: %i\n",status);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        printf("%c",gps_char);
        if (gps_idx < sizeof(nmea_recv)-1)
        {
            nmea_recv[gps_idx++] = gps_char;
            //printf("NMEA: %s\n",nmea_recv);
            if (gps_char == '\n')
            {
                nmea_recv[gps_idx] = '\0';
                
                if (strncmp(nmea_recv, "$GNRMC", 6) == 0)
                {
                    memcpy(saved_rmc, nmea_recv, 128);
                    rmc_ready = 1;
                }

                gps_idx = 0;
            }
        }
        else
        {
            gps_idx = 0; // overflow recovery
        }
        __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
        HAL_UART_Receive_IT(&huart1, &gps_char, 1);
    }
}

void readGPS() {
    if (rmc_ready) {
        uint8_t rmc[128];
        memcpy(rmc, saved_rmc, 128);
        //printf("received message: %s", rmc);
        rmc_ready = 0;
        uint32_t len = strlen(rmc);
        packet gpsPacket = {0};
        gpsPacket.type = PACKET_GPS;
        gpsPacket.timestamp = HAL_GetTick() - startTime;
        gpsPacket.len = len;
        gpsPacket.payload = (uint8_t*)rmc;
        writePacket(gpsPacket);
    } else {
        return;
    }
}

void initBMIC() {
    uint8_t bmicAddr = 0b1100100;
    //printf("%02x 0b%02x\n",pow, control);
    //write 0 to control register - sets M (prescaler) to 1, giving highest precision
    HAL_StatusTypeDef status;

    uint8_t regs[8];

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        bmicAddr << 1,
        0x00,
        I2C_MEMADD_SIZE_8BIT,
        regs,
        8,
        HAL_MAX_DELAY);

    printf("Status=%d\n", status);

    for(int i = 0; i < 8; i++)
    {
        printf("%02X: %02X\n", i, regs[i]);
    }

    Write_I2C_Reg(bmicAddr, 1, 0b00001000, 1);

    uint8_t acr_buf[2];

    //initialize lastAcc
    status = HAL_I2C_Mem_Read(
        &hi2c1,
        bmicAddr << 1,    // STM32 HAL expects 8-bit address
        0x02,                 // ACR MSB register
        I2C_MEMADD_SIZE_8BIT,
        acr_buf,
        2,                    // Read MSB + LSB together
        50);
    if (status == HAL_OK) {
        lastAcc = (acr_buf[0] << 8) | acr_buf[1];
    }
}

void readBMIC() {
    uint8_t bmicAddr = 0b1100100;

    uint8_t acr_buf[2];
    uint16_t acc;

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        bmicAddr << 1,    // STM32 HAL expects 8-bit address
        0x02,                 // ACR MSB register
        I2C_MEMADD_SIZE_8BIT,
        acr_buf,
        2,                    // Read MSB + LSB together
        HAL_MAX_DELAY);
    if (status == HAL_OK) {
        acc = (acr_buf[0] << 8) | acr_buf[1];
        if (acc > lastAcc) { // misread (sensor was likely updating mid read)
            return;
        }
        uint16_t diff = lastAcc - acc;
        if (diff > 0x100) { // unrealistically large jump (also likely a misread)
            return;
        }
        //printf("read: 0x%04x\n",acc);
        if (acc < 0x300) {
            acc += 0x7fff;
            Write_I2C_Reg(bmicAddr, 2, (uint8_t)(acc >> 8), 1);
            Write_I2C_Reg(bmicAddr, 3, (uint8_t)(acc & 0xff), 1);
        }
        totalEnergyUsed += diff;
        float mah = 0.085f * 2.0f/128.0f * totalEnergyUsed * 0.5f;

        packet bmicPacket = {0};
        bmicPacket.type = PACKET_BMIC;
        bmicPacket.timestamp = HAL_GetTick() - startTime;
        bmicPacket.len = sizeof(uint32_t);
        bmicPacket.payload = (uint8_t*)&totalEnergyUsed;
        writePacket(bmicPacket);
        //printf("%i 0x%04x, total energy used: %.05f mAh\n", bmicPacket.len, acc, mah);
        lastAcc = acc;
        //printf("bmic status: 0x%02x\n", status);
    } else {
        printf("Failed to read BMIC: status %i\n", hi2c1.ErrorCode);
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(5);
        HAL_I2C_Init(&hi2c1);
        return;
    }
}

void setupAccSleep() {
	//setup accelerometer for generating interrupts on movement
	log_printf("LOG: Configuring accelerometer for sleep\r\n");
	uint8_t ACC_ADDR = 0b0011000;
	uint8_t x_reg = 0x28;
	uint8_t y_reg = 0x2A;
	uint8_t z_reg = 0x2C;

	uint8_t CTRL_REG1 = 0x20;
	uint8_t CTRL_REG2 = 0x21;
	uint8_t CTRL_REG3 = 0x22;
	uint8_t CTRL_REG4 = 0x23;
	uint8_t CTRL_REG5 = 0x24;
	uint8_t INT1_THS = 0x32;
	uint8_t INT1_DURATION = 0x33;
	uint8_t INT1_CFG = 0x30;
	uint8_t REFERENCE = 0x26;

	//set 100 Hz low-power mode
	Write_I2C_Reg(ACC_ADDR, CTRL_REG1, 0b01011111,1);
	// use high pass filter
	Write_I2C_Reg(ACC_ADDR, CTRL_REG2, 0b00001001,1);
	//enable interrupt 1
	Write_I2C_Reg(ACC_ADDR, CTRL_REG3, 0b01000000,1);
	// set +-2g range
	Write_I2C_Reg(ACC_ADDR, CTRL_REG4, 0b00000000,1);
	// latch interrupt 1
	Write_I2C_Reg(ACC_ADDR, CTRL_REG5, 0b00001000,1);
	// set 384 mg threshold on interrupt
	Write_I2C_Reg(ACC_ADDR, INT1_THS, 0b00011000,1);
	//set 20 ms duration before interrupt is recognized
	Write_I2C_Reg(ACC_ADDR, INT1_DURATION, 0b00000010,1);

	uint8_t ref = Read_I2C_Reg(ACC_ADDR, REFERENCE,1);

	//generate interrupt on high events of x, y, or z
	Write_I2C_Reg(ACC_ADDR, INT1_CFG, 0b00101010,1);
	log_printf("LOG: successfully configured accelerometer.\r\n");
}

void setupAccWake() {
	//setup accelerometer for generating interrupts on movement
	uint8_t ACC_ADDR = 0b0011000;
	uint8_t x_reg = 0x28;
	uint8_t y_reg = 0x2A;
	uint8_t z_reg = 0x2C;

	uint8_t CTRL_REG1 = 0x20;
	uint8_t CTRL_REG2 = 0x21;
	uint8_t CTRL_REG3 = 0x22;
	uint8_t CTRL_REG4 = 0x23;
	uint8_t CTRL_REG5 = 0x24;

	uint8_t INT1_CFG = 0x30;
	uint8_t INT1_SRC = 0x31;

	//read interrupt res to clear it
	uint8_t int_event = Read_I2C_Reg(ACC_ADDR, INT1_SRC, 1);

	//low-power 1 Hz
	Write_I2C_Reg(ACC_ADDR, CTRL_REG1, 0b00011111,1);
	//no filters
	Write_I2C_Reg(ACC_ADDR, CTRL_REG2, 0b00000000,1);
	//disable interrupts
	Write_I2C_Reg(ACC_ADDR, CTRL_REG3, 0b00000000,1);
	//+-4g range
	Write_I2C_Reg(ACC_ADDR, CTRL_REG4, 0b00001000,1);
	//no interrupt functions
	Write_I2C_Reg(ACC_ADDR, CTRL_REG5, 0b00000000,1);

	//disable interrupts on high events of x, y, or z
	Write_I2C_Reg(ACC_ADDR, INT1_CFG, 0b00000000,1);

}