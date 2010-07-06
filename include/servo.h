/*--------------------------------------------------------------------------------+
 |   Ajeco ANDI-SERVO Motion controller driver                                    |
 |                                                                                |
 |   Copyright (c) 1999, Mark Dennehy                                             |
 |                                                                                |
 |   Permission is hereby granted, free of charge, to any person obtaining a copy |
 |   of this software and associated documentation files (the "Software"), to deal|
 |   in the Software without restriction, including without limitation the rights |
 |   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell    |
 |   copies of the Software, and to permit persons to whom the Software is        |
 |   furnished to do so, subject to the following conditions:                     |
 |                                                                                |
 |   The above copyright notice and this permission notice shall be included in   |
 |   all copies or substantial portions of the Software.                          |
 |                                                                                |
 |   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR   |
 |   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,     |
 |   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE  |
 |   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER       |
 |   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,|
 |   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN    |
 |   THE SOFTWARE.                                                                |
 +-------------------------------------------------------------------------------*/

#ifndef	SERVO_H
#define SERVO_H

#include <linux/module.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/uaccess.h>

#include <andi_servo.h>
#include <andi.h>
#include <Lk.h>

/*---------------------------------------------------------------------+
 |    Defines                                                          |
 +--------------------------------------------------------------------*/

#define TRACE TRUE

#define LIMIT (PAGE_SIZE - 80)

#define SERVO_MAJOR 120
#define SERVO_ADDR 0x300
#define SERVO_NAME "andi_servo"
#define SERVO_NAME_LENGTH 10

/* Minor device numbers */
#define BOARD 0
#define CHANNEL_0 1
#define CHANNEL_1 2
#define FILTER_0 3
#define FILTER_1 4
#define TRAJECTORY_0 5
#define TRAJECTORY_1 6

/*---------------------------------------------------------------------+
 |    Prototypes                                                       |
 +--------------------------------------------------------------------*/

int init_module(void);
void cleanup_module(void);

int procfile_board_read(char *buffer, char **buffer_location, off_t offset,
						int buffer_length, int *eof, void *data);
int procfile_channel0_read(char *buffer, char **buffer_location, off_t offset,
						   int buffer_length, int *eof, void *data);
int procfile_channel1_read(char *buffer, char **buffer_location, off_t offset,
						   int buffer_length, int *eof, void *data);
int procfile_trajectory0_read(char *buffer, char **buffer_location,
							  off_t offset, int buffer_length, int *eof,
							  void *data);
int procfile_trajectory1_read(char *buffer, char **buffer_location,
							  off_t offset, int buffer_length, int *eof,
							  void *data);
int procfile_filter0_read(char *buffer, char **buffer_location, off_t offset,
						  int buffer_length, int *eof, void *data);
int procfile_filter1_read(char *buffer, char **buffer_location, off_t offset,
						  int buffer_length, int *eof, void *data);

int servo_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param);
static int servo_open(struct inode *inode, struct file *file);
static int servo_close(struct inode *inode, struct file *file);
static ssize_t servo_read(struct file *file, char *buffer, size_t length,
						  loff_t * offset);
static ssize_t servo_write(struct file *file, const char *buffer, size_t length,
						   loff_t * offset);

static int servo_read_board(char *buffer, size_t length, loff_t * offset);
static int servo_read_channel0(char *buffer, size_t length, loff_t * offset);
static int servo_read_channel1(char *buffer, size_t length, loff_t * offset);
static int servo_read_filter0(char *buffer, size_t length, loff_t * offset);
static int servo_read_filter1(char *buffer, size_t length, loff_t * offset);
static int servo_read_trajectory0(char *buffer, size_t length, loff_t * offset);
static int servo_read_trajectory1(char *buffer, size_t length, loff_t * offset);
static int servo_write_board(const char *buffer, size_t length,
							 loff_t * offset);
static int servo_write_channel0(const char *buffer, size_t length,
								loff_t * offset);
static int servo_write_channel1(const char *buffer, size_t length,
								loff_t * offset);
static int servo_write_filter0(const char *buffer, size_t length,
							   loff_t * offset);
static int servo_write_filter1(const char *buffer, size_t length,
							   loff_t * offset);
static int servo_write_trajectory0(const char *buffer, size_t length,
								   loff_t * offset);
static int servo_write_trajectory1(const char *buffer, size_t length,
								   loff_t * offset);
#endif
