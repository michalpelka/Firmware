/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>
__EXPORT int px4_simple_app_main(int argc, char *argv[]);
 
int px4_simple_app_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	//open GPIO device
	int pin =0;
	int gpio_fd = open(PX4FMU_DEVICE_PATH, 0);
	if (gpio_fd < 0)
	{
		printf("gpio_led: GPIO device \"%s\" open fail\n", PX4FMU_DEVICE_PATH);
		return -1;
	}
	pin = 1;

	while(true)
	{
		pin = 1;
		ioctl(gpio_fd, GPIO_SET_INPUT, pin);


		int out=ioctl(gpio_fd, GPIO_GET, pin);
		printf("%d, %d\n", pin, out);
		sleep(1);
	}
//	int state=0x01;
//	while (true) {
//		//ioctl (gpio_fd, GPIO_GET(), pin);
//		if (state == 0x01)
//		{
//			printf("On!\n");
//			state = 0x00;
//			ioctl(gpio_fd, GPIO_SET, 1);
//		}
//		else
//		if (state == 0x00)
//		{
//			printf("Off!\n");
//			state = 0x01;
//			ioctl(gpio_fd, GPIO_CLEAR, 1);
//		}
//
//
//		sleep(1);
//	}

	return 0;
}
