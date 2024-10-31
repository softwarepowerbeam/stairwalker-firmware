/*
 * cpp_link.hpp
 *
 *  Created on: Sep 21, 2023
 *      Author: Brenda
 */

#ifndef APPLICATION_INC_CPP_LINK_HPP_
#define APPLICATION_INC_CPP_LINK_HPP_

#ifdef __cplusplus
extern "C" {
#endif


typedef struct thread_vector_handlers
{
	osThreadId *handler;
	uint32_t size;
}thread_vector_t;

typedef struct semaphore_vector_handlers
{
	osSemaphoreId *handler;
	uint32_t size;
}semaphore_vector_t;

typedef struct queue_vector_handlers
{
	osMessageQId *handler;
	uint32_t size;
}queue_vector_t;

typedef struct rtos_handlers_matrix
{
	thread_vector_t thread_vector;
	semaphore_vector_t semaphore_vector;
	queue_vector_t queue_vector;

}rtos_handlers_matrix_t;




//void cpp_main_process(osThreadId waiting_task_1, osThreadId waiting_task_2);
void cpp_main_process(osThreadId *waiting_tasks, uint32_t tasks_to_wait);
void main_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);
void debug_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);
void seat_ctrl_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);

void main_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);
void chair_imu_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);
void debug_cpp_wrapper(rtos_handlers_matrix_t *rtos_handler);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_INC_CPP_LINK_HPP_ */
