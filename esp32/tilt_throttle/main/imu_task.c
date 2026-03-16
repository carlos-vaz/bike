#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "../../icm20948/icm20948.h"
#include "../../icm20948/icm20948_i2c.h"


#define TAG "imu_task"

#define GYR_WINDOW_ms (500)
#define GYRO_DATA_PROC_FREQ_hz (100)
#define SERIAL_OUTPUT_FREQ_hz (10)
#define MOTOR_CMD_FREQ_hz (50)
#define GYRO_DATA_BUFFER_LEN ((GYR_WINDOW_ms * GYRO_DATA_PROC_FREQ_hz) / 1000)

#define GYRO_X_DEADBAND_dps (10)
#define GYRO_FSS_RANGE_dps (500) // actual range... may differ from configured range
                                 // since ICM chip fails to write config FSS for now
#define MAX_DUTY_CYCLE_CMD_percent (100)

/* i2c bus configuration */
i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	//.sda_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SDA,
    .sda_io_num = (gpio_num_t) 19,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	//.scl_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SCL,
	.scl_io_num = (gpio_num_t) 18,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 400000,
	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

/* ICM 20948 configuration */
icm0948_config_i2c_t icm_config = {
	.i2c_port = I2C_NUM_0,
	.i2c_addr = ICM_20948_I2C_ADDR_AD1
};

// input : float in range [-32k to 32k]
// output: float (degrees per second)
int gyroToMotorCommand(float gx) {
    float gx_dps = gx * GYRO_FSS_RANGE_dps / (1<<16);

    // kill value if inside of deadband
    if(((gx_dps > 0) && (gx_dps < GYRO_X_DEADBAND_dps)) || \
       ((gx_dps < 0) && (gx_dps > -1*GYRO_X_DEADBAND_dps))) {
        gx_dps = 0.0;
    }

    return (2 * gx_dps * MAX_DUTY_CYCLE_CMD_percent / GYRO_FSS_RANGE_dps);
}

#define TEST_MOTOR_CMD_VALUE_CHANGE_FREQ_hz (1)
// Update duty cycle command (-30, 0, 30, 0, -30, 0, ...) 
int testMotorCommand(int iter) {
    static int cmd = 0;
    if(iter % (GYRO_DATA_PROC_FREQ_hz / TEST_MOTOR_CMD_VALUE_CHANGE_FREQ_hz) == 0) {
        cmd = 20;
        if((iter % (2*(GYRO_DATA_PROC_FREQ_hz / TEST_MOTOR_CMD_VALUE_CHANGE_FREQ_hz))) != 0) {
            cmd = 0;
        }
        if((iter % (4*(GYRO_DATA_PROC_FREQ_hz / TEST_MOTOR_CMD_VALUE_CHANGE_FREQ_hz))) == 0) {
            cmd = -20;
        }
    }
    return cmd;
}


void printGyroX(icm20948_agmt_t *agmt, float gX) {
    ESP_LOGI(TAG, "gxraw= %d\t\tgxfiltered=%.2f", agmt->gyr.axes.x, gX);
}


void print_agmt(icm20948_agmt_t agmt)
{
  	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] (WITH LPF) Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		agmt.tmp.val
	);
}


 // Outputs filtered (sliding window average) gyroX data. 
 // Change window size (milliseconds) using GYR_WINDOW_ms definition
 float processGyro(icm20948_agmt_t *agmt) {
    static int16_t gyroXBuffer[GYRO_DATA_BUFFER_LEN];
    static int gbi = 0; // buffer index
    static bool full = false; 
    static float avg = 0.0;

    if(full) {
        // expel old data point from average
        avg -= ((float)gyroXBuffer[gbi] / GYRO_DATA_BUFFER_LEN);
    }
    else {
        // not expelling, but changing datapoint weights
        avg = (avg*gbi)/(gbi+1);  // before update, size = gbi, newsize = gbi+1
    }

    gyroXBuffer[gbi++] = agmt->gyr.axes.x;
    full = (gbi==GYRO_DATA_BUFFER_LEN) ? true : full;
    gbi %= GYRO_DATA_BUFFER_LEN;

    // include new data point in average
    avg += ((float)agmt->gyr.axes.x / (full ? GYRO_DATA_BUFFER_LEN : gbi)); // after update, newsize = gbi

    return avg;
}

void imu_task(void *arg)
{
    icm20948_device_t icm;

	/* setup i2c */
	ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));
	
	/* setup ICM20948 device */
	icm20948_init_i2c(&icm, &icm_config);
		
	/* check ID */
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
	{
		ESP_LOGE(TAG, "check id failed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* check whoami */
	icm20948_status_e stat = ICM_20948_STAT_ERR;
	uint8_t whoami = 0x00;
	while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))
	{
		whoami = 0x00;
		stat = icm20948_get_who_am_i(&icm, &whoami);
		ESP_LOGE(TAG, "whoami does not match (0x %d). Halting...", whoami);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	icm20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	//icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_GYR);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	if(ICM_20948_STAT_OK != icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS)) {
        printf("UNABLE TO SET sample modes (continuous)\n");
    } else {
        printf("SUCCESSFULLY SET sample modes (continuous)\n");
    }


	// Set full scale ranges for both acc and gyr
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_2000; // (icm20948_gyro_config_1_fs_sel_e)
	if(ICM_20948_STAT_OK != icm20948_set_full_scale(&icm, sensors, myfss)) {
        printf("UNABLE TO SET GYRO full scale\n");
    } else {
        printf("successfuly set Gyro FSS scale\n");
    }


	// Set up DLPF configuration
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	//myDLPcfg.g = GYR_D361BW4_N376BW5;
    myDLPcfg.g = GYR_D5BW7_N8BW9;
	if(ICM_20948_STAT_OK != icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg)) {
        printf("UNABLE TO SET GYRO DLPF FILTER TYPE\n");
    } else {
        printf("successfuly set Gyro FILTER\n");
    }

	// Choose whether or not to use DLPF
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
	if(ICM_20948_STAT_OK != icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, true)) {
        printf("UNABLE TO SET GYRO FCHOICE\n");
    }
    else {
        printf("successfuly set Gyro FCHOICE\n");
    }

	// Now wake the sensor up
	icm20948_sleep(&icm, false);
	icm20948_low_power(&icm, false);

    // argument to this task is the queue for CORE0 (this) --> CORE1 communication
    QueueHandle_t queue = (QueueHandle_t)arg;
    icm20948_agmt_t agmt;
    float gxFiltered = 0;
    int motorCmd = 0;
    int iter = 0;

    while (1) {
		vTaskDelay((1000 / GYRO_DATA_PROC_FREQ_hz) / portTICK_PERIOD_MS);

        // Read and process Gyro data at frequency GYRO_DATA_PROC_FREQ_hz
        if (ICM_20948_STAT_OK == icm20948_get_agmt(&icm, &agmt)) {
            // process gyro data
            gxFiltered = processGyro(&agmt);
        } else {
            ESP_LOGE(TAG, "Unable to read from ICM20948");
        }

        // Send motor commands at frequency MOTOR_CMD_FREQ_hz
        if(iter % (GYRO_DATA_PROC_FREQ_hz / MOTOR_CMD_FREQ_hz) == 0) {
            motorCmd = gyroToMotorCommand(gxFiltered);
            //motorCmd = testMotorCommand(iter);

            //ESP_LOGI(TAG, "sending motor cmd = %d", motorCmd);
            if (xQueueGenericSend(queue, (void *)&motorCmd, portMAX_DELAY, queueSEND_TO_BACK) != pdTRUE) {
                ESP_LOGI(TAG, "Queue full\n");
            }
        }

        //  print output (for plotting / debugging) at SERIAL_OUTPUT_FREQ_hz
        if(iter % (GYRO_DATA_PROC_FREQ_hz / SERIAL_OUTPUT_FREQ_hz) == 0) {
            printGyroX(&agmt, gxFiltered);
        }

        iter++;
    }

    vTaskDelete(NULL);
}
