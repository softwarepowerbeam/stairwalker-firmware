/*
 * cpp_link.cpp
 *
 *  Created on: Sep 21, 2023
 *      Author: Brenda
 */



#include "cmsis_os.h"
#include "cpp_link.hpp"
#include "lwip/apps/lwiperf.h"
#include <cmath>
#include "freertos.h"

/********************************DEBUG MACROS**********************************/
#define DEBUG_CPP_LINK


/***********************************INCLUDES***********************************/
/*General Includes*/
#include "main.h"
#include "tim.h"
#include "usart.h"
//#include "spi.h"
#include "adc.h"

/*PB_HAL_STM32H7 Includes*/
//#include "ADC/adc_stm32h7.hpp"//TODO (high)CHeck why an error is presented when ADC peripheral is no added
#include "GPIO/gpio_stm32h7.hpp"
#include "UART/uart_stm32h7.hpp"
#include "UART/uart_stm32h7_dma.hpp"
#include "PWM/pwm_stm32h7.hpp"
#include "ADC/adc_stm32h7.hpp"

/*BoardSupport Includes*/
#include "DCMotorController/roboclaw.hpp"
#include "HBridge/h_bridge_tle7209.hpp"
#include "UDP/udp_client_stm32h7.hpp"
#include "IMU/bno085.hpp"
#include "HBridge/vnhxxxxxas.hpp"

/*PB_Drivers Includes*/
#include "LED/led_gpio.hpp"
#include "DCMotor/dc_motor.hpp"

/*UserInterface Includes*/
#include "Devices/DeviceLED.hpp"
#include "Devices/DeviceDCMotor.hpp"
#include "Devices/DeviceSwingChair.hpp"
#include "Devices/DeviceIMU.hpp"

#include "Logging.hpp"
#include "TrafficCop.hpp"
#include "Json.hpp"



/****************************HAL INITIALIZATIONS*******************************/


/*External Handlers*/



/*UDP Inits*/
const uint8_t HOST_ADDRESS[] = {192,168,33,1};
//const uint8_t HOST_ADDRESS[] = {192,168,33,105};
const uint16_t UDP_CMD_PORT{8};
const uint16_t UDP_LOG_PORT{49152};


/*
 * https://community.st.com/t5/stm32-mcus/how-to-create-a-project-for-stm32h7-with-ethernet-and-lwip-stack/ta-p/49308
 * https://github.com/stm32-hotspot/STM32H7-LwIP-Examples
 * https://www.reddit.com/r/embedded/comments/13rcrqf/stm32h723zg_creating_tcpip_with_lwip_but_cannot/
 * https://www.youtube.com/watch?v=6olMulM5SCg
 * https://controllerstech.com/stm32-ethernet-1-connection/
 * https://www.youtube.com/watch?v=MJfUiw8bZEI&list=PLfIJKC1ud8gjoY2McCCqkxXWuwiC2iSap&index=2
 */

/* Initialize UDP and Logs*/
UdpClientSTM32H7 logUDP;
UdpClientSTM32H7 trafficCopUDP;
TrafficCop traffic_cop(&trafficCopUDP);

void mainProcess(osThreadId *waiting_task, uint32_t tasks_to_wait)
{

}

void main_process(rtos_handlers_matrix_t *rtos_handler)
{

//	/* Initialize UDP and Logs*/
//	UdpClientSTM32H7 logUDP;
//	UdpClientSTM32H7 trafficCopUDP;
//	TrafficCop traffic_cop(&trafficCopUDP);

	Logging log;
	string task_name = "Default Task";
	osDelay(100);

	/* Initialize UDP and Logs*/
	logUDP.initialize(HOST_ADDRESS, UDP_LOG_PORT);
	trafficCopUDP.initialize(HOST_ADDRESS, UDP_CMD_PORT);

	// Create UDP Thread
	osDelay(100);
	sys_thread_new("udp_thread",
					traffic_cop.receiveThread,
					(void*) &traffic_cop,
					DEFAULT_THREAD_STACKSIZE,
					osPriorityNormal);
	osDelay(100);

	Logging::udpClient = &logUDP;
	log.Initialize(&task_name);
	log.Info("UDP Initialized");
	osDelay(100);



	/*Init LEDs*/
	GpioSTM32H7 red_led_pin(LED_RED_GPIO_Port, LED_RED_Pin, GpioIOConf::GPIO_OUTPUT);
	DeviceLED dev_red_led(&red_led_pin);

	red_led_pin.set();


	traffic_cop.addDevice(&dev_red_led, "red_led");



	for(uint32_t idx = 0; idx < rtos_handler->thread_vector.size; idx++)
	{
		xTaskNotify(*(rtos_handler->thread_vector.handler + (idx)), 0, eNoAction);
	}

	for(uint32_t idx = 0; idx < rtos_handler->thread_vector.size; idx++)
	{
		xTaskNotifyWait(0x00, /* Don't clear any notification bits on entry. */
							ULONG_MAX, /* Reset the notification value to 0 on exit. */
							NULL, /* Notification value is not required */
							portMAX_DELAY); /* Block indefinitely. */
	}



	traffic_cop.initialize();

	for (;;)
	{

#ifdef DEBUG_CPP_LINK

//		red_led_pin.set();
//		osDelay(500);
//		red_led_pin.clear();
		osDelay(500);


#else
		osDelay(1000);
#endif //DEBUG_CPP_LINK
	}

}


void debug_process(rtos_handlers_matrix_t *rtos_handler)
{
	xTaskNotify(*rtos_handler->thread_vector.handler, 0, eNoAction);


	while(true)
	{
		__NOP();
	}
}

extern uint8_t abBuffer[ALIGN_BASE2_CEIL(19, __SCB_DCACHE_LINE_SIZE)] __ALIGNED(__SCB_DCACHE_LINE_SIZE);
extern osPoolId  mpool;

float yaw = 0;
float pitch = 0;
float roll = 0;
bno085_rvc_datagram_t imu_data;

void seat_ctrl_control(rtos_handlers_matrix_t *rtos_handler)
{
	//______________________IMU Configuration______________________________
	//Configure configuration GPIOs
	GpioSTM32H7 bno085_intn(BNO085_INTN_GPIO_Port, BNO085_INTN_Pin,
													GpioIOConf::GPIO_INPUT);
	GpioSTM32H7 bno085_nrst(BNO085_NRST_GPIO_Port, BNO085_NRST_Pin,
													GpioIOConf::GPIO_OUTPUT);
	GpioSTM32H7 bno085_ps0(BNO085_PS0_GPIO_Port, BNO085_PS0_Pin,
													GpioIOConf::GPIO_OUTPUT);
	GpioSTM32H7 bno085_ps1(BNO085_PS1_GPIO_Port, BNO085_PS1_Pin,
													GpioIOConf::GPIO_OUTPUT);
	//Configure communication protocol
	UartSTM32H7DMA imu_bno085_uart_rvc(&huart4, UART_RVC_SIZE);

	//TODO: (High) implement the imu_bno085 library!!!!!!!!!!!!
	//Initialize IMU Device
	imu_bno085 imu_chair(&imu_bno085_uart_rvc, bno085_com_type_e::UART_RVC,
				&bno085_ps0, &bno085_ps1, &bno085_nrst, &bno085_intn);
	imu_chair.initialize();

	//Adding IMU as a device to the traffic cop
	DeviceIMU wheelchair_imu;
	traffic_cop.addDevice(&wheelchair_imu, "wheelchair_imu");

	//______________________Motor H-Bridge Configuration______________________________

	GpioSTM32H7 vnhxxxxas_ina(VNH7XXXXAS_INA_GPIO_Port, VNH7XXXXAS_INA_Pin,
													GpioIOConf::GPIO_OUTPUT);
	GpioSTM32H7 vnhxxxxas_inb(VNH7XXXXAS_INB_GPIO_Port, VNH7XXXXAS_INB_Pin,
													GpioIOConf::GPIO_OUTPUT);
	GpioSTM32H7 vnhxxxxas_sel0(VNH7XXXXAS_SEL0_GPIO_Port, VNH7XXXXAS_SEL0_Pin,
													GpioIOConf::GPIO_OUTPUT);
	PwmSTM32H7 h_bridge_pwm(&htim2, TIM_CHANNEL_4);
	PwmSTM32H7 h_bridge_pwm_bis(&htim14, TIM_CHANNEL_1);

	AdcChannelSTM32H7 hbridge_adc(&hadc1, adc_channel_e::CHANNEL_10);

	hbridge_vnhxxxxxas hbridge_bis(&h_bridge_pwm_bis, &vnhxxxxas_ina, &vnhxxxxas_inb,
										&vnhxxxxas_sel0, &hbridge_adc);
	hbridge_bis.initialize();

	hbridge_vnhxxxxxas hbridge(&h_bridge_pwm, &vnhxxxxas_ina, &vnhxxxxas_inb,
								&vnhxxxxas_sel0, &hbridge_adc);
	hbridge.initialize();

	DeviceSwingChairRaw SwingChair(&hbridge, &yaw);//TODO: (Very High) not implemented!!
	SwingChair.initialize();
	traffic_cop.addDevice(&SwingChair, "swing_chair");
	//______________________Current Sensor Configuration______________________________

	AdcChannelSTM32H7 motor_current_adc(&hadc3, adc_channel_e::CHANNEL_1);

	//________Notify that device configuration finished here________________________
	xTaskNotify(*rtos_handler->thread_vector.handler, 0, eNoAction);


	const float g_cnst = 9.807;  //!< (m/s^2) gravitational constant.
#ifdef ACTUAL_DIM
	const float length = 0.275;  //!< (m) distance .
	const float mass = 90;    //!< (Kg) Mass. This parameter might change from user to user (check robustness of this selection).
#else //ACTUAL_DIM
	const float length = 0.15;  //!< (m) distance .
	const float mass = 0.07;    //!< (Kg) Mass. This parameter might change from user to user (check robustness of this selection).
#endif //ACTUAL_DIM

	const float torque_const = g_cnst * length * mass;
	const float ml2_const = (length * length)* mass;

	uint8_t *imu_raw_data;
	bool is_close_loop = false;
	int32_t angle_ctrl_raw = 0;
	float kp = 0, ki = 0, kd = 0, k0 = 0, k1 = 0, k2 = 0;
	float angle_setpoint = 0, friction_coeff = 0, sinus_coeff = 1;
	const int32_t MAX_PWM = 2048;
	const float	MAX_CTRL = (float) (3.14159);//In Radians
	const float OUTPUT_SCALE_COEFF_RAD = (2047 / 3.14159);
	const float DEG2RAW = (float) MAX_PWM/90;
	const float PERIOD = 0.01;//UART-RVC send information with a 100Hz frequency
	float angular_velocity_rad = 0.0;
	float angular_velocity = 0.0;
	float angle_error[3];
	swing_chair_ctrl_type_t ctrl_type = swing_chair_ctrl_type_t::PID;
	float angle_rad = 0.0;
	float angle = 0.0;
	float angle_control = 0;
	float nonlinear_ctrl = 0;
	float angle_rad_old = 0;
	float angle_old = 0;
	float angle_setpoint_rad = 0;
	float friction = 0;
	float angle_ctrl_frict = 0;
	const float deg2rad = 3.14159265359/180;
	const float rad2deg = 180/3.14159265359;

	imu_data.pitch = 0;
	imu_data.yaw = 0;
	imu_data.roll = 0;

	angle_error[0] = 0.0;
	angle_error[1] = 0.0;
	angle_error[2] = 0.0;

	while(true)
	{

		is_close_loop = SwingChair.get_ctrl_mode();
		angle_setpoint = SwingChair.get_reference();

		if(is_close_loop == true)
		{
			//Since feedback is mandatory do nothing if IMU is not working
			//TODO: (high) add a timeout
			osEvent evt = osMessageGet(*(rtos_handler->queue_vector.handler), osWaitForever);

			if (evt.status == osEventMessage)
			{
				imu_raw_data = new uint8_t[UART_RVC_SIZE];

				uint8_t *message = (uint8_t*)evt.value.p;
				memcpy(imu_raw_data, message, UART_RVC_SIZE);
				osPoolFree(mpool, message);

				imu_chair.decode_rvc(imu_raw_data, &imu_data);

				delete [] imu_raw_data;

				yaw = ((float) imu_data.yaw) / 100;
				pitch = ((float) imu_data.pitch) / 100;
				roll = ((float) imu_data.roll) / 100;
				wheelchair_imu.set_pitch(pitch);
				wheelchair_imu.set_yaw(yaw);
				wheelchair_imu.set_roll(roll);

				kp = SwingChair.get_p_coeff();
				ki = SwingChair.get_i_coeff();
				kd = SwingChair.get_d_coeff();
				k0 = kp + ki + kd;
				k1 = -kp - (2 * kd);
				k2 = kd;

				ctrl_type = SwingChair.get_ctrl_type();
				friction_coeff = SwingChair.get_friction_coeff();
				sinus_coeff = SwingChair.get_sinus_coeff();


				angle_setpoint_rad = angle_setpoint * deg2rad;
//				angle_rad = pitch * deg2rad;
				angle_old = angle;
				angle = roll;
				angle_rad = angle * deg2rad;

				angle_error[2] = angle_error[1];
				angle_error[1] = angle_error[0];
				angle_error[0] = angle_setpoint_rad - angle_rad;
				angular_velocity = (angle - angle_old) / PERIOD;
				angular_velocity_rad = angular_velocity * deg2rad;

				switch(ctrl_type)
				{
					case swing_chair_ctrl_type_t::PID:
						angle_control = angle_control + (k0 * angle_error[0]) +
														(k1 * angle_error[1]) +
														(k2 * angle_error[2]);
						if(angle_control > MAX_CTRL)
						{
							angle_control = MAX_CTRL;
						}
						else if (angle_control < -MAX_CTRL)
						{
							angle_control = -MAX_CTRL;
						}
						angle_ctrl_raw = (int32_t) (OUTPUT_SCALE_COEFF_RAD * angle_control);

						break;
					case swing_chair_ctrl_type_t::PID_FRICTION:

						if(angular_velocity > 0.01)
						{
							friction = 0.15;
						}
						else if(angular_velocity < -0.01)
						{
							friction = -0.15;
						}
						else
						{
							friction = 0;
						}
						angle_control = angle_control + (k0 * angle_error[0]) +
														(k1 * angle_error[1]) +
														(k2 * angle_error[2]);

						angle_ctrl_frict = angle_control + friction;

						if(angle_ctrl_frict > MAX_CTRL)
						{
							angle_ctrl_frict = MAX_CTRL;
						}
						else if (angle_ctrl_frict < -MAX_CTRL)
						{
							angle_ctrl_frict = -MAX_CTRL;
						}
						angle_ctrl_raw = (int32_t) (OUTPUT_SCALE_COEFF_RAD * angle_ctrl_frict);

						break;

					case swing_chair_ctrl_type_t::FEEDBACK_LINEARIZATION_PID:
						angle_control = angle_control + (k0 * angle_error[0]) +
														(k1 * angle_error[1]) +
														(k2 * angle_error[2]);
						if(angle_control > MAX_CTRL)
						{
							angle_control = MAX_CTRL;
						}
						else if (angle_control < -MAX_CTRL)
						{
							angle_control = -MAX_CTRL;
						}

						//underactuated.mit.edu/pend.html
						//https://ctms.engin.umich.edu/CTMS/index.php?aux=Activities_Pendulum
						//https://www.sciencedirect.com/topics/engineering/friction-compensation
						//My model has a high Dry friction
						nonlinear_ctrl = (angle_control * ml2_const) +
								(sinus_coeff * torque_const * sin(angle_rad));

						angle_ctrl_raw = (int32_t) (OUTPUT_SCALE_COEFF_RAD * nonlinear_ctrl);

						break;
					case swing_chair_ctrl_type_t::FEEDBACK_LINEARI_FRICTION_PID:
						break;
					default:
						__NOP();
						break;
				}
			}
			else
			{
				wheelchair_imu.set_pitch(0);
				wheelchair_imu.set_yaw(0);
				wheelchair_imu.set_roll(0);
				angle_ctrl_raw = 0;
			}
		}
		else
		{
			osEvent evt = osMessageGet(*(rtos_handler->queue_vector.handler), 500);/*wait 500msec*/

			if (evt.status == osEventMessage)
			{
				uint8_t *message = (uint8_t*)evt.value.p;
				imu_raw_data = new uint8_t[UART_RVC_SIZE];
				memcpy(imu_raw_data, message, UART_RVC_SIZE);
				osPoolFree(mpool, message);
				imu_chair.decode_rvc(imu_raw_data, &imu_data);
				delete [] imu_raw_data;
			}
			else
			{
				wheelchair_imu.set_pitch(0);
				wheelchair_imu.set_yaw(0);
				wheelchair_imu.set_roll(0);
			}
			//Set open-loop control output
//			angle_ctrl_raw = angle_setpoint;
			angle_ctrl_raw = (int32_t) (DEG2RAW * angle_setpoint);

		}


		//
		if(angle_ctrl_raw > MAX_PWM)
		{
			angle_ctrl_raw = MAX_PWM;
		}
		else if (angle_ctrl_raw < -MAX_PWM)
		{
			angle_ctrl_raw = -MAX_PWM;
		}
		hbridge.set(angle_ctrl_raw);
		hbridge_bis.set(angle_ctrl_raw);

		SwingChair.update_angle(angle);
		SwingChair.update_angular_velocity(angular_velocity);
		SwingChair.update_control_output(angle_ctrl_raw);
		SwingChair.update_debug(angle_error[0] * rad2deg);
		uint32_t adc_value = 0;
		hbridge_adc.get(adc_value);
		SwingChair.update_current((float) adc_value);
		uint32_t adc_value_2;
		motor_current_adc.get(adc_value_2);
		SwingChair.update_debug1((float) adc_value_2);
//		SwingChair.update_debug2(1024);

	}




}
/*******************************CPP<->C LINKAGE*********************************/


#ifdef __cplusplus
extern "C"
{
#endif

uint8_t ready;
extern osMessageQId imu_queueHandle;




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	static uint64_t errors_accum = 0;


	uint8_t *message = (uint8_t*)osPoolAlloc(mpool);
	static uint8_t shift = 0;
	size_t nbReceived = sizeof(abBuffer);

	SCB_InvalidateDCache_by_Addr(abBuffer, nbReceived);

	if((abBuffer[0] == 0xAA) && (abBuffer[1] == 0xAA))
	{
		HAL_UART_Receive_DMA(&huart4,  abBuffer, 19);
		SCB_InvalidateDCache_by_Addr(abBuffer, nbReceived);
		memcpy(message, abBuffer, 19);
	}
	else
	{
		if(shift == 1)
		{
			HAL_UART_Receive_DMA(&huart4,  abBuffer, 1);
			SCB_InvalidateDCache_by_Addr(abBuffer, nbReceived);
			shift = 0;
		}
		else
		{

			HAL_UART_Receive_DMA(&huart4,  abBuffer, 19);
			SCB_InvalidateDCache_by_Addr(abBuffer, nbReceived);
			shift = 1;
		}
	}

//Check https://os.mbed.com/handbook/CMSIS-RTOS
	osMessagePut(imu_queueHandle, (uint32_t) message, 0);


}


uint8_t get_imu_from_queue(bno085_rvc_datagram_t *datagram)
{
	//TODO: (medium) implement
}


void cpp_main_process(osThreadId *waiting_tasks, uint32_t tasks_to_wait)
{
	mainProcess(waiting_tasks, tasks_to_wait);
}

void main_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler)
{
	main_process(rtos_handler);
}

void debug_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler)
{
	debug_process(rtos_handler);
}
void seat_ctrl_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler)
{
	seat_ctrl_control(rtos_handler);
}

#ifdef __cplusplus
}
#endif

