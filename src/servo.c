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

#ifndef __KERNEL__
#	define __KERNEL__
#endif

#ifndef MODULE
#	define EXPORT_NO_SYMBOLS
#	define MODULE
#endif

#include <servo.h>

/*static char kernel_version[] = UTS_RELEASE;*/

/*---------------------------------------------------------------------+
 |    ANDI-SERVO data                                                  |
 +--------------------------------------------------------------------*/

static struct LM629_Filter filter0 = {
	dterm:2,
	kp:2,
	ki:0,
	kd:50,
	il:0
};

static struct LM629_Filter filter1 = {
	dterm:2,
	kp:2,
	ki:0,
	kd:50,
	il:0
};

static struct LM629_Filter new_filter0 = {
	dterm:2,
	kp:2,
	ki:0,
	kd:50,
	il:0
};

static struct LM629_Filter new_filter1 = {
	dterm:2,
	kp:2,
	ki:0,
	kd:50,
	il:0
};

static struct LM629_Trajectory trajectory0 = {
	stop_smooth:TRUE,
	load_acc:TRUE,
	load_vel:TRUE,
	load_pos:TRUE,
	acc:160000L,
	velocity:200000L,
	position:0L
};

static struct LM629_Trajectory trajectory1 = {
	stop_smooth:TRUE,
	load_acc:TRUE,
	load_vel:TRUE,
	load_pos:TRUE,
	acc:160000L,
	velocity:200000L,
	position:0L
};

static struct LM629_Trajectory new_trajectory0 = {
	stop_smooth:TRUE,
	load_acc:TRUE,
	load_vel:TRUE,
	load_pos:TRUE,
	acc:160000L,
	velocity:200000L,
	position:0L
};

static struct LM629_Trajectory new_trajectory1 = {
	stop_smooth:TRUE,
	load_acc:TRUE,
	load_vel:TRUE,
	load_pos:TRUE,
	acc:160000L,
	velocity:200000L,
	position:0L
};

static struct LM629 channel0 = {
	Filter:&filter0,
	NewFilter:&new_filter0,
	Trajectory:&trajectory0,
	NewTrajectory:&new_trajectory0,
	filter_updated:FALSE,
	trajectory_started:FALSE,
	pwm_brake:FALSE,
	position_error:0
};

static struct LM629 channel1 = {
	Filter:&filter1,
	NewFilter:&new_filter1,
	Trajectory:&trajectory1,
	NewTrajectory:&new_trajectory1,
	filter_updated:FALSE,
	trajectory_started:FALSE,
	pwm_brake:FALSE,
	position_error:0
};

static struct andi_servo servo = {
	Channel0:&channel0,
	Channel1:&channel1,
	FaultLED:FALSE
};

/*---------------------------------------------------------------------+
 |    /proc/andi-servo file data structures                            |
 +--------------------------------------------------------------------*/

struct proc_dir_entry servo_proc_dir = {
	namelen:SERVO_NAME_LENGTH,
	name:SERVO_NAME,
	mode:S_IFDIR | S_IRUGO | S_IXUGO | S_IWUSR,
	uid:0,
	gid:0,
	nlink:1
};

struct proc_dir_entry servo_board_proc_file = {
	namelen:5,
	name:"board",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_board_read
};

struct proc_dir_entry servo_channel0_proc_file = {
	namelen:8,
	name:"channel0",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_channel0_read
};

struct proc_dir_entry servo_channel1_proc_file = {
	namelen:8,
	name:"channel1",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_channel1_read
};

struct proc_dir_entry servo_trajectory0_proc_file = {
	namelen:11,
	name:"trajectory0",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_trajectory0_read
};

struct proc_dir_entry servo_trajectory1_proc_file = {
	namelen:11,
	name:"trajectory1",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_trajectory1_read
};

struct proc_dir_entry servo_filter0_proc_file = {
	namelen:7,
	name:"filter0",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_filter0_read
};

struct proc_dir_entry servo_filter1_proc_file = {
	namelen:7,
	name:"filter1",
	mode:S_IFREG | S_IRUGO,
	uid:0,
	gid:0,
	nlink:1,
	read_proc:procfile_filter1_read
};

/*---------------------------------------------------------------------+
 |    file operations structure                                        |
 +--------------------------------------------------------------------*/

struct file_operations servo_fops = {
	read:servo_read,
	write:servo_write,
	ioctl:servo_ioctl,
	open:servo_open,
	release:servo_close
};

/*---------------------------------------------------------------------+
 |    ioctl() function                                                 |
 +--------------------------------------------------------------------*/

int servo_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param)
{
	LG(TRACE,
	   "int servo_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)\n");

	switch (MINOR(inode->i_rdev))
	{
	case BOARD:
		switch (ioctl_num)
		{
		case SERVO_HARD_RESET:
		case SERVO_SET_BRAKES:
		case SERVO_SET_LED:
		case SERVO_SET_IRQ_ENABLE:
		case SERVO_GET_BRAKES:
		case SERVO_GET_LED:
		case SERVO_GET_IRQ_ENABLE:
		case SERVO_GET_IRQ_CAUSE:
		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	case CHANNEL_0:
	case CHANNEL_1:
		switch (ioctl_num)
		{
		case SERVO_SOFT_RESET:
		case SERVO_SMOOTH_STOP:
		case SERVO_ABRUPT_STOP:
		case SERVO_MOTOR_OFF:
		case SERVO_SET_HOME_POSITION:
		case SERVO_SET_BRAKE:
		case SERVO_POSITION_MODE:
		case SERVO_FEEDBACK_MODE:
		case SERVO_SET_BREAKPOINT:
		case SERVO_SET_ACCELERATION:
		case SERVO_SET_POSITION_ERROR_THRESHOLD:
		case SERVO_SET_IRQ_MASK:
		case SERVO_GET_STATUS:
		case SERVO_GET_SIGNALS:
		case SERVO_GET_BRAKE:
		case SERVO_GET_BREAKPOINT:
		case SERVO_GET_ACCELERATION:
		case SERVO_GET_POSITION_ERROR_THRESHOLD:
		case SERVO_GET_IRQ_MASK:
		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	case FILTER_0:
		switch (ioctl_num)
		{
		case SERVO_UPDATE_FILTER:
			return update_filter(&servo, 1);
		case SERVO_CHECK_FILTER_UPDATED:
			*(int *) ioctl_param = servo.Channel0->filter_updated;
			return 0;

		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	case FILTER_1:
		switch (ioctl_num)
		{
		case SERVO_UPDATE_FILTER:
			return update_filter(&servo, 1);
		case SERVO_CHECK_FILTER_UPDATED:
			*(int *) ioctl_param = servo.Channel1->filter_updated;
			return 0;

		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	case TRAJECTORY_0:
		switch (ioctl_num)
		{
		case SERVO_START_TRAJECTORY:
			return start_trajectory(&servo, 0);
		case SERVO_CHECK_TRAJECTORY_STARTED:
			*(int *) ioctl_param = servo.Channel0->trajectory_started;
			return 0;
		case SERVO_CHECK_TRAJECTORY_COMPLETE:
			*(int *) ioctl_param = servo.Channel0->trajectory_complete;
			return 0;

		case SERVO_REGISTER_FOR_NOTIFICATION:
		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	case TRAJECTORY_1:
		switch (ioctl_num)
		{
		case SERVO_START_TRAJECTORY:
			return start_trajectory(&servo, 1);
		case SERVO_CHECK_TRAJECTORY_STARTED:
			*(int *) ioctl_param = servo.Channel1->trajectory_started;
			return 0;
		case SERVO_CHECK_TRAJECTORY_COMPLETE:
			*(int *) ioctl_param = servo.Channel1->trajectory_complete;
			return 0;

		case SERVO_REGISTER_FOR_NOTIFICATION:
		default:
			L("Unknown ioctl_number %u\n", ioctl_num);
			return -ENOSYS;
		};
		break;

	default:
		L("Unknown minor device number : %d\n", MINOR(inode->i_rdev));
		return -ENXIO;
	};

	return 0;
}

/*---------------------------------------------------------------------+
 |    open() function                                                  |
 +--------------------------------------------------------------------*/

static int servo_open(struct inode *inode, struct file *file)
{
	LG(TRACE,
	   "static int servo_open(struct inode *inode, struct file *file)\n");

	MOD_INC_USE_COUNT;

	return 0;
}

/*---------------------------------------------------------------------+
 |    close() function                                                 |
 +--------------------------------------------------------------------*/

static int servo_close(struct inode *inode, struct file *file)
{
	LG(TRACE,
	   "static int servo_close(struct inode *inode, struct file *file)\n");

	MOD_DEC_USE_COUNT;

	return 0;
}

/*---------------------------------------------------------------------+
 |    read() function                                                  |
 +--------------------------------------------------------------------*/

static ssize_t servo_read(struct file *file, char *buffer, size_t length,
						  loff_t * offset)
{
	LG(TRACE,
	   "static ssize_t servo_read(struct file *file, char *buffer, size_t length, loff_t * offset)\n");

	/*
	 * if (offset != file->f_pos)
	 *         return -ESPIPE;
	 */

	switch (MINOR(file->f_dentry->d_inode->i_rdev))
	{
	case BOARD:
		return servo_read_board(buffer, length, offset);

	case CHANNEL_0:
		return servo_read_channel0(buffer, length, offset);

	case CHANNEL_1:
		return servo_read_channel1(buffer, length, offset);

	case FILTER_0:
		return servo_read_filter0(buffer, length, offset);

	case FILTER_1:
		return servo_read_filter1(buffer, length, offset);

	case TRAJECTORY_0:
		return servo_read_trajectory0(buffer, length, offset);

	case TRAJECTORY_1:
		return servo_read_trajectory1(buffer, length, offset);

	default:
		L("Unknown minor device number : %d\n",
		  MINOR(file->f_dentry->d_inode->i_rdev));
		return -ENXIO;
	};

	return 0;
}

/*---------------------------------------------------------------------+
 |    write() function                                                 |
 +--------------------------------------------------------------------*/

static ssize_t servo_write(struct file *file, const char *buffer, size_t length,
						   loff_t * offset)
{
	LG(TRACE,
	   "static ssize_t servo_write(struct file *file, const char *buffer, size_t length, loff_t * offset)\n");

	switch (MINOR(file->f_dentry->d_inode->i_rdev))
	{
	case BOARD:
		return servo_write_board(buffer, length, offset);

	case CHANNEL_0:
		return servo_write_channel0(buffer, length, offset);

	case CHANNEL_1:
		return servo_write_channel1(buffer, length, offset);

	case FILTER_0:
		return servo_write_filter0(buffer, length, offset);

	case FILTER_1:
		return servo_write_filter1(buffer, length, offset);

	case TRAJECTORY_0:
		return servo_write_trajectory0(buffer, length, offset);

	case TRAJECTORY_1:
		return servo_write_trajectory1(buffer, length, offset);

	default:
		L("Unknown minor device number : %d\n",
		  MINOR(file->f_dentry->d_inode->i_rdev));
		return -ENXIO;
	};

	return 0;
}

/*---------------------------------------------------------------------+
 |    int init_module(void)                                            |
 +--------------------------------------------------------------------*/

int init_module(void)
{
	int retval;

	LG(TRACE, "int init_module(void)\n");

/*
 * port ranges: the device can reside between
 * 0x280 and 0x3F0, in step of 0x10. It uses 8 ports.
 * Default is 0x300
 */
	servo.base_address = SERVO_ADDR;

	if (check_region(servo.base_address, 8))
	{
		L("Could not allocate I/O region.\n");
		return EBUSY;
	}
	else
		request_region(servo.base_address, 8, SERVO_NAME);

	retval = init_board(&servo);

	if (retval)
	{
		L("Error initialising ANDI-SERVO board\n");
		goto init_board_failure;
		return EIO;
	}

/*
 * /proc files. Used for status reports only - no input routines. (Yet).
 * Handy for a quick view of the system without interfering with any
 * running controllers.
 * /proc/andi
 * 			/board
 * 			/channel0
 * 			/channel1
 * 			/trajectory0
 * 			/trajectory1
 * 			/filter0
 * 			/filter1
 * 			
 */

	L("Registering procfiles\n");

	retval = proc_register(&proc_root, &servo_proc_dir);
	if (retval < 0)
	{
		L("Error in registering /proc/%s directory : %d\n", SERVO_NAME, retval);
		goto proc_dir_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_board_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/board file : %d\n", SERVO_NAME,
		  retval);
		goto proc_board_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_channel0_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/channel0 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_channel0_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_channel1_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/channel1 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_channel1_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_trajectory0_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/trajectory0 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_trajectory0_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_trajectory1_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/trajectory1 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_trajectory1_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_filter0_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/filter0 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_filter0_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	retval = proc_register(&servo_proc_dir, &servo_filter1_proc_file);
	if (retval < 0)
	{
		L("Error in registering /proc/%s/filter1 file : %d\n", SERVO_NAME,
		  retval);
		goto proc_filter1_register_failure;	/* Yes, a goto. I know, I know ... */
	}

/*
 * This is the actual device itself. It has several minor devices. 
 * These are the actual control nodes. In /dev there should be a
 * subdirectory containing them, but that's optional.
 * /dev/andi_servo
 *                 /board
 *                 /channel0
 *                 /channel1
 *                 /trajectory0
 *                 /trajectory1
 *                 /filter0
 *                 /filter1
 *
 */

	L("Registering chrdev\n");

	retval = register_chrdev(SERVO_MAJOR, SERVO_NAME, &servo_fops);
	if (retval < 0)
	{
		L("Error in registering device : %d\n", retval);
		goto chrdev_register_failure;	/* Yes, a goto. I know, I know ... */
	}

	L("Success\n");
	return 0;

/*-----------------------------------------------------------------------+
 |Ich. Unfortunately, this is one of the only times goto has a legitimate|
 |use as the kernal doesn't track register failures during init_module().|
 +----------------------------------------------------------------------*/

  chrdev_register_failure:
	unregister_chrdev(SERVO_MAJOR, SERVO_NAME);
  proc_filter1_register_failure:
	proc_unregister(&servo_proc_dir, servo_filter1_proc_file.low_ino);
  proc_filter0_register_failure:
	proc_unregister(&servo_proc_dir, servo_filter0_proc_file.low_ino);
  proc_trajectory1_register_failure:
	proc_unregister(&servo_proc_dir, servo_trajectory1_proc_file.low_ino);
  proc_trajectory0_register_failure:
	proc_unregister(&servo_proc_dir, servo_trajectory0_proc_file.low_ino);
  proc_channel1_register_failure:
	proc_unregister(&servo_proc_dir, servo_channel1_proc_file.low_ino);
  proc_channel0_register_failure:
	proc_unregister(&servo_proc_dir, servo_channel0_proc_file.low_ino);
  proc_board_register_failure:
	proc_unregister(&servo_proc_dir, servo_board_proc_file.low_ino);
  proc_dir_register_failure:
	proc_unregister(&proc_root, servo_proc_dir.low_ino);
  init_board_failure:
	release_region(servo.base_address, 8);
	return retval;

}

/*---------------------------------------------------------------------+
 |    void cleanup_module(void)                                        |
 +--------------------------------------------------------------------*/

void cleanup_module(void)
{
	int retval;

	LG(TRACE, "void cleanup_module(void)\n");

	release_region(servo.base_address, 8);

	retval = proc_unregister(&servo_proc_dir, servo_board_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/board file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = proc_unregister(&servo_proc_dir, servo_channel0_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/channel0 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = proc_unregister(&servo_proc_dir, servo_channel1_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/channel1 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval =
		proc_unregister(&servo_proc_dir, servo_trajectory0_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/trajectory0 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval =
		proc_unregister(&servo_proc_dir, servo_trajectory1_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/trajectory1 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = proc_unregister(&servo_proc_dir, servo_filter0_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/filter0 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = proc_unregister(&servo_proc_dir, servo_filter1_proc_file.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s/filter1 file: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = proc_unregister(&proc_root, servo_proc_dir.low_ino);
	if (retval < 0)
	{
		L("Error in unregistering /proc/%s directory: %d\n", SERVO_NAME,
		  retval);
		return;
	}

	retval = unregister_chrdev(SERVO_MAJOR, SERVO_NAME);
	if (retval < 0)
	{
		L("Error in unregistering device : %d\n", retval);
		return;
	}

}

/*---------------------------------------------------------------------+
 |    These are the functions called by the driver infrastructure.     |
 +--------------------------------------------------------------------*/

/*---------------------------------------------------------------------------+
 |int procfile_board_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                     |
 +--------------------------------------------------------------------------*/

int procfile_board_read(char *buffer, char **buffer_location, off_t offset,
						int buffer_length, int *eof, void *data)
{
	int len, retval, status0, status1, signals0, signals1, heading;
	long encoder0, encoder1;

	LG(TRACE,
	   "int procfile_board_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	heading = 0;
	len = 0;
	retval = 0;
	status1 = 0;
	signals1 = 0;
	encoder1 = 0L;
	status0 = 0;
	signals0 = 0;
	encoder0 = 0L;

	if (offset > 0)
		return 0;

	retval = get_status(&servo, 0, &status0);
	if (retval < 0)
	{
		L("Error with get_status : %d\n", retval);
		return retval;
	}

	retval = get_status(&servo, 1, &status1);
	if (retval < 0)
	{
		L("Error with get_status : %d\n", retval);
		return retval;
	}

	retval = get_signals(&servo, 0, &signals0);
	if (retval < 0)
	{
		L("Error with get_signals : %d\n", retval);
		return retval;
	}

	retval = get_signals(&servo, 1, &signals1);
	if (retval < 0)
	{
		L("Error with get_signals : %d\n", retval);
		return retval;
	}

	retval = get_real_position(&servo, 0, &encoder0);
	if (retval < 0)
	{
		L("Error with get_real_position : %d\n", retval);
		return retval;
	}

	retval = get_real_position(&servo, 1, &encoder1);
	if (retval < 0)
	{
		L("Error with get_real_position : %d\n", retval);
		return retval;
	}

	len = sprintf(buffer,
				  "Ajeco ANDI-SERVO Motion controller driver. $Revision: 2.59 $\n\n");
	len += sprintf(buffer + len, "Channel 0 Status  : %02x\n", status0);
	len += print_status(status0, buffer + len);
	len += sprintf(buffer + len, "\n");

	len += sprintf(buffer + len, "Channel 0 Signals : %04x\n", signals0);
	len += print_signals(signals0, buffer + len);
	len += sprintf(buffer + len, "\n");

	len += sprintf(buffer + len, "Channel 1 Status  : %02x\n", status1);
	len += print_status(status1, buffer + len);
	len += sprintf(buffer + len, "\n");

	len += sprintf(buffer + len, "Channel 1 Signals : %04x\n", signals1);
	len += print_signals(signals1, buffer + len);
	len += sprintf(buffer + len, "\n");

	len += sprintf(buffer + len, "Channel 0 Encoder Count : %08lx\n", encoder0);
	len += sprintf(buffer + len, "Channel 1 Encoder Count : %08lx\n", encoder1);
	len += sprintf(buffer + len, "\n");

	return len;
}

/*------------------------------------------------------------------------------+
 |int procfile_channel0_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                        |
 +-----------------------------------------------------------------------------*/

int procfile_channel0_read(char *buffer, char **buffer_location, off_t offset,
						   int buffer_length, int *eof, void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_channel0_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer,
				  "Ajeco ANDI-SERVO Motion controller driver. $Revision: 2.59 $\n\n");

	return len;
}

/*------------------------------------------------------------------------------+
 |int procfile_channel1_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                        |
 +-----------------------------------------------------------------------------*/

int procfile_channel1_read(char *buffer, char **buffer_location, off_t offset,
						   int buffer_length, int *eof, void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_channel1_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer,
				  "Ajeco ANDI-SERVO Motion controller driver. $Revision: 2.59 $\n\n");

	return len;
}

/*---------------------------------------------------------------------------------+
 |int procfile_trajectory0_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                           |
 +--------------------------------------------------------------------------------*/

int procfile_trajectory0_read(char *buffer, char **buffer_location,
							  off_t offset, int buffer_length, int *eof,
							  void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_trajectory0_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer, "Channel 0 Trajectory :\n");
	len += print_trajectory(servo.Channel0->Trajectory, buffer + len);

	return len;
}

/*---------------------------------------------------------------------------------+
 |int procfile_trajectory1_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                                       |
 +--------------------------------------------------------------------------------*/

int procfile_trajectory1_read(char *buffer, char **buffer_location,
							  off_t offset, int buffer_length, int *eof,
							  void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_trajectory1_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer, "Channel 1 Trajectory :\n");
	len += print_trajectory(servo.Channel1->Trajectory, buffer + len);

	return len;
}

/*-----------------------------------------------------------------------------+
 |int procfile_filter0_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                                   |
 +----------------------------------------------------------------------------*/

int procfile_filter0_read(char *buffer, char **buffer_location, off_t offset,
						  int buffer_length, int *eof, void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_filter0_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer, "Channel 0 Filter :\n");
	len += print_filter(servo.Channel0->Filter, buffer + len);

	return len;
}

/*-----------------------------------------------------------------------------+
 |int procfile_filter1_read(char *buffer, char **buffer_location, off_t offset,|
 |              int buffer_length, int *eof, void *data)                       |
 +----------------------------------------------------------------------------*/

int procfile_filter1_read(char *buffer, char **buffer_location, off_t offset,
						  int buffer_length, int *eof, void *data)
{
	int len;

	LG(TRACE,
	   "int procfile_filter1_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)\n");

	if (offset > 0)
		return 0;

	len = sprintf(buffer, "Channel 1 Filter :\n");
	len += print_filter(servo.Channel1->Filter, buffer + len);

	return len;
}

/*------------------------------------------------------------------------+
 |static int servo_read_board(char* buffer, size_t length, loff_t *offset)|
 +-----------------------------------------------------------------------*/
static int servo_read_board(char *buffer, size_t length, loff_t * offset)
{

	LG(TRACE,
	   "static int servo_read_board(char* buffer, size_t length, loff_t *offset)\n");

	return -ENOSYS;
}

/*---------------------------------------------------------------------------+
 |static int servo_read_channel0(char* buffer, size_t length, loff_t *offset)|
 +--------------------------------------------------------------------------*/
static int servo_read_channel0(char *buffer, size_t length, loff_t * offset)
{
	LG(TRACE,
	   "static int servo_read_channel0(char* buffer, size_t length, loff_t *offset)\n");

	return -ENOSYS;
}

/*---------------------------------------------------------------------------+
 |static int servo_read_channel1(char* buffer, size_t length, loff_t *offset)|
 +--------------------------------------------------------------------------*/
static int servo_read_channel1(char *buffer, size_t length, loff_t * offset)
{
	LG(TRACE,
	   "static int servo_read_channel1(char* buffer, size_t length, loff_t *offset)\n");

	return 0;
}

/*--------------------------------------------------------------------------+
 |static int servo_read_filter0(char* buffer, size_t length, loff_t *offset)|
 +-------------------------------------------------------------------------*/
static int servo_read_filter0(char *buffer, size_t length, loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_read_filter0(char* buffer, size_t length, loff_t *offset)\n");

	retval = sprintf(buffer, "Channel 0 Filter :\n");
	retval += print_filter(servo.Channel0->Filter, buffer + retval);

	return retval;
}

/*--------------------------------------------------------------------------+
 |static int servo_read_filter1(char* buffer, size_t length, loff_t *offset)|
 +-------------------------------------------------------------------------*/
static int servo_read_filter1(char *buffer, size_t length, loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_read_filter1(char* buffer, size_t length, loff_t *offset)\n");

	retval = sprintf(buffer, "Channel 1 Filter :\n");
	retval += print_filter(servo.Channel1->Filter, buffer + retval);

	return retval;
}

/*------------------------------------------------------------------------------+
 |static int servo_read_trajectory0(char* buffer, size_t length, loff_t *offset)|
 +-----------------------------------------------------------------------------*/
static int servo_read_trajectory0(char *buffer, size_t length, loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_read_trajectory0(char* buffer, size_t length, loff_t *offset)\n");

	retval = sprintf(buffer, "Channel 0 Trajectory :\n");
	retval += print_trajectory(servo.Channel0->Trajectory, buffer + retval);

	return retval;
}

/*------------------------------------------------------------------------------+
 |static int servo_read_trajectory1(char* buffer, size_t length, loff_t *offset)|
 +-----------------------------------------------------------------------------*/
static int servo_read_trajectory1(char *buffer, size_t length, loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_read_trajectory1(char* buffer, size_t length, loff_t *offset)\n");

	retval = sprintf(buffer, "Channel 1 Trajectory :\n");
	retval += print_trajectory(servo.Channel1->Trajectory, buffer + retval);

	return retval;
}

/*-------------------------------------------------------------------------------+
 |static int servo_write_board(const char* buffer, size_t length, loff_t *offset)|
 +------------------------------------------------------------------------------*/
static int servo_write_board(const char *buffer, size_t length, loff_t * offset)
{
	LG(TRACE,
	   "static int servo_write_board(const char* buffer, size_t length, loff_t *offset)\n");

	return -ENOSYS;
}

/*----------------------------------------------------------------------------------+
 |static int servo_write_channel0(const char* buffer, size_t length, loff_t *offset)|
 +---------------------------------------------------------------------------------*/
static int servo_write_channel0(const char *buffer, size_t length,
								loff_t * offset)
{
	LG(TRACE,
	   "static int servo_write_channel0(const char* buffer, size_t length, loff_t *offset)\n");

	return -ENOSYS;
}

/*----------------------------------------------------------------------------------+
 |static int servo_write_channel1(const char* buffer, size_t length, loff_t *offset)|
 +---------------------------------------------------------------------------------*/
static int servo_write_channel1(const char *buffer, size_t length,
								loff_t * offset)
{
	LG(TRACE,
	   "static int servo_write_channel1(const char* buffer, size_t length, loff_t *offset)\n");

	return -ENOSYS;
}

/*---------------------------------------------------------------------------------+
 |static int servo_write_filter0(const char* buffer, size_t length, loff_t *offset)|
 +--------------------------------------------------------------------------------*/
static int servo_write_filter0(const char *buffer, size_t length,
							   loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_write_filter0(const char* buffer, size_t length, loff_t *offset)\n");

	retval =
		copy_from_user(servo.Channel0->NewFilter,
					   (struct LM629_Filter *) buffer,
					   sizeof (struct LM629_Filter));

	if (retval)
		return -EFAULT;

	if (!load_filter(&servo, 0))
		return -EIO;

	memcpy(servo.Channel0->Filter, servo.Channel0->NewFilter,
		   sizeof (struct LM629_Filter));

	return sizeof (struct LM629_Filter);
}

/*---------------------------------------------------------------------------------+
 |static int servo_write_filter1(const char* buffer, size_t length, loff_t *offset)|
 +--------------------------------------------------------------------------------*/
static int servo_write_filter1(const char *buffer, size_t length,
							   loff_t * offset)
{
	int retval;

	LG(TRACE,
	   "static int servo_write_filter1(const char* buffer, size_t length, loff_t *offset)\n");

	retval =
		copy_from_user(servo.Channel1->NewFilter,
					   (struct LM629_Filter *) buffer,
					   sizeof (struct LM629_Filter));

	if (retval)
		return -EFAULT;

	if (!load_filter(&servo, 1))
		return -EIO;

	memcpy(servo.Channel1->Filter, servo.Channel1->NewFilter,
		   sizeof (struct LM629_Filter));

	return sizeof (struct LM629_Filter);
}

/*-------------------------------------------------------------------------------------+
 |static int servo_write_trajectory0(const char* buffer, size_t length, loff_t *offset)|
 +------------------------------------------------------------------------------------*/
static int servo_write_trajectory0(const char *buffer, size_t length,
								   loff_t * offset)
{
	LG(TRACE,
	   "static int servo_write_trajectory0(const char* buffer, size_t length, loff_t *offset)\n");

	if (copy_from_user
		(servo.Channel0->NewTrajectory, (struct LM629_Trajectory *) buffer,
		 sizeof (struct LM629_Trajectory)))
		return -EFAULT;

	if (!load_trajectory(&servo, 1))
		return -EIO;

	memcpy(servo.Channel0->Trajectory, servo.Channel0->NewTrajectory,
		   sizeof (struct LM629_Trajectory));

	return sizeof (struct LM629_Trajectory);
}

/*-------------------------------------------------------------------------------------+
 |static int servo_write_trajectory1(const char* buffer, size_t length, loff_t *offset)|
 +------------------------------------------------------------------------------------*/
static int servo_write_trajectory1(const char *buffer, size_t length,
								   loff_t * offset)
{
	LG(TRACE,
	   "static int servo_write_trajectory1(const char* buffer, size_t length, loff_t *offset)\n");

	if (copy_from_user
		(servo.Channel1->NewTrajectory, (struct LM629_Trajectory *) buffer,
		 sizeof (struct LM629_Trajectory)))
		return -EFAULT;

	if (!load_trajectory(&servo, 1))
		return -EIO;

	memcpy(servo.Channel1->Trajectory, servo.Channel1->NewTrajectory,
		   sizeof (struct LM629_Trajectory));

	return sizeof (struct LM629_Trajectory);
}
