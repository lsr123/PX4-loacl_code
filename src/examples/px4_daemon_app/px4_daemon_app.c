/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>



#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
//#include <ecl/attitude_fw/ecl_pitch_controller.h>
//#include <ecl/attitude_fw/ecl_roll_controller.h>
//#include <ecl/attitude_fw/ecl_wheel_controller.h>
//#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>



#include <sys/ipc.h>



#include <px4_config.h>
#include <px4_posix.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>





static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */


/**
 * daemon management function.
 */
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);
int set_uart_baudrate(const int fd, unsigned int baud);
int rw_uart(void);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 px4_daemon_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int px4_daemon_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	thread_running = true;

	while (!thread_should_exit) {

		//warnx("Hello daemon!\n");
		//sleep(10);
		rw_uart();
		
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}

int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud)
	{
		case 9600:          speed = B9600;       break;
		case 19200:        speed = B19200;     break;
		case 38400:        speed = B38400;     break;
		case 57600:        speed = B57600;     break;
		case 115200:      speed = B115200;   break;
		default:
		//warnx("ERR:  baudrate:  %d\n", baud);

		return -EINVAL;
	}


	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration*/
	tcgetattr(fd, &uart_config);
	/* clear ONLCR flag (which appends a CR for every LF)*/
	uart_config.c_oflag &= ~ONLCR;
	/*no parity, one stop bit*/
	uart_config.c_cflag &= ~(CSTOPB|PARENB);
	/*set baud rate*/
	if ((termios_state = cfsetispeed (&uart_config, speed)) < 0)
	{
		warnx("ERR: %d (cfsetispeed) \n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr (fd, TCSANOW,  &uart_config)) < 0)
	{
		warnx("ERR: %d (tcsetattr) \n", termios_state);
		return false;
	}

	return true;
}
int rw_uart(void)
{
	

	//char data[] = "D1234567812345678\t";
	char dataA[] = "function_A running\n";
	char dataB[] = "function_B running\n";
	char buffer[] = "";
	//char command[8] = ""; 
	char temp1[] = {0x01,0x01,0x00,0x00};  //stop
	char temp2[] = {0x01,0x01,0x01,0x00};  //start
	char temp3[] = {0x01,0x08,0x00,0x01}; //speed_01
	char temp4[] = {0x01,0x04,0x00,0x00}; //speed_up
	char temp5[] = {0x01,0x05,0x00,0x00}; //speed_down

	//int ss;
	//char write_dataF[4] = "FFFF";
	//char write_dataL[4] = "LLLL";
	int i = 0;
	//ss = sizeof(int);
	int uart_read = open("/dev/ttyS6", O_RDWR|O_NOCTTY);  //kai chuan kou 

	if (uart_read < 0)
	{
		err(1, "failed to open port: %s", "/dev/ttyS6");
		return -1;
	}
	
	
	if (false == set_uart_baudrate(uart_read, 9600))
	{
		printf("[YCM]set_uart_baudrate is failed \n");
		return -1; 
	}
	printf("[YSM] uart init is successful\n");
	
	// while(1)
	// 	{printf("OK" );}
	
	
	while(true)
	{
		sleep(1);
		//printf("size = %d\n",sizeof(data));
		//int aaa = write(uart_read, &data, sizeof(data));
		//printf("aaa = %d\n",aaa );
		//sleep(1);
		write(uart_read,&temp1,sizeof(temp1));  //stop
		sleep(1);
		write(uart_read,&temp2,sizeof(temp2)); //start
		sleep(1);
		write(uart_read,&temp3,sizeof(temp3));  //speed_01
		sleep(1);

		for(i = 0;i<10;i++)      //speed_up
		{
			//read(uart_read,&command[i],1);
			
			write(uart_read,&temp4,sizeof(temp4));
			sleep(1);
			//printf("%d\n",(size_t)temp1);
			//write(uart_read,&i,sizeof(i));

		}
		for(i = 0;i<10;i++)     //speed_down
		{
			//read(uart_read,&command[i],1);
			
			write(uart_read,&temp5,sizeof(temp5));
			sleep(1);
			//printf("%d\n",(size_t)temp1);
			//write(uart_read,&i,sizeof(i));

		}
		write(uart_read,&temp1,sizeof(temp1));


		//int bbb = read(uart_read,&buffer,1);
		if(buffer[0] == '0')
		{
			write(uart_read,&dataA,sizeof(dataA));
			//write(uart_read,&command,sizeof(command));
		}
		if(buffer[0] == 'B')
		{
			write(uart_read,&dataB,sizeof(dataB));
		}
		//printf("bbb = %d\n",bbb );
		//write(uart_read,&bbb,sizeof(bbb));



		//if(data == 'R')
		// //{
		// 	for(int i = 0; i < 4; ++i )
		// 	{
		// 		read(uart_read, &data, 1); // du shu ju
		// 		buffer[i] = data;
		// 		data = '0';
		// 	}
			
		// 	printf("%s\n",buffer );
			
			//write(uart_read, &write_dataF, (size_t)write_dataF);
		//if(data == 'L')
			//write(uart_read, &write_dataL, (size_t)write_dataL);
			//sleep(10);
			//break;

		//}

		//write(uart_read, &write_data, (size_t)write_data);
		//if(j>100)
		//	break;

			
		//printf("IN ");

	//}
	
	}
	//close(uart_read);

	//printf("OUT");
	return 0;
}
