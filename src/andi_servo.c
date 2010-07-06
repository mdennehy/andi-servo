/*--------------------------------------------------------------------------------+
 |   Ajeco ANDI-SERVO Motion controller driver                                    |
 |                                                                                |
 |   Copyright (c) 1999 Mark Dennehy                                              |
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

#include <andi_servo.h>

#define TRACE TRUE
#define BUSY_RETRY_LIMIT 30

#ifndef __KERNEL__
#	define __KERNEL__
#endif

#ifndef MODULE
#	define EXPORT_NO_SYMBOLS
#	define MODULE
#endif

#define __NO_VERSION__

static int command, data;
static char buffer[512];

/*---------------------------------------------------------------------+
 | int check_busy_bit(struct andi_servo *board, int channel)           |
 |                                                                     |
 | Checks the LM629 busy-bit. Used for handshaking. If the LM629 is not|
 | busy, it returns 0 (false). Tries the busy-bit 3 times.             |
 +--------------------------------------------------------------------*/
int check_busy_bit(struct andi_servo *board, int channel)
{
	int i;

	LG(TRACE, "check_busy_bit(struct andi_servo *board, int channel)\n");

	if (channel)
		command = board->base_address + COMMAND_1;
	else
		command = board->base_address + COMMAND_0;

	LG(TRACE, "check_busy_bit: Channel selected, attempting inb()\n");

	for (i = 0; i < BUSY_RETRY_LIMIT; i++)
		if (!(inb(command) & BUSY_BIT))
		{
			LG(TRACE, "check_busy_bit: success\n");
			return 0;
		}
		else
			LG(TRACE, "check_busy_bit: failure, retrying\n");

	LG(TRACE, "check_busy_bit: failure, aborting\n");

	return -EBUSY;
}

/*---------------------------------------------------------------------+
 |  int soft_reset(struct andi_servo *board, int channel)              |
 |                                                                     |
 |  Software Reset operation. As per LM629 programming guide, page 4.  |
 |  Returns 0 if there aren't any problems, and an errorcode otherwise.|
 +--------------------------------------------------------------------*/
int soft_reset(struct andi_servo *board, int channel)
{
	int retval;
	LG(TRACE, "int soft_reset(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	OUT(RESET, command);

	mdelay(2);

	CHECK_BUSY;

	OUT(RSTI, command);

	CHECK_BUSY;

	OUT(0x00, data);
	OUT(0x00, data);

	CHECK_BUSY;

	return 0;
}

/*---------------------------------------------------------------------+
 |    int define_home(struct andi_servo *board, int channel)           |
 +--------------------------------------------------------------------*/
int define_home(struct andi_servo *board, int channel)
{
	LG(TRACE, "int define_home(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*-----------------------------------------------------------------------+
 |int set_position_error_threshold(struct andi_servo *board, int channel,|
 |                             int position_error_threshold,             |
 |                             BOOLEAN stop_on_error)                    |
 +----------------------------------------------------------------------*/
int set_position_error_threshold(struct andi_servo *board, int channel,
								 int position_error_threshold,
								 BOOLEAN stop_on_error)
{
	LG(TRACE,
	   "int set_position_error_threshold(struct andi_servo *board, int channel, int position_error_threshold, BOOLEAN stop_on_error)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------------+
 |int set_breakpoint(struct andi_servo *board, int channel, BOOLEAN relative)|
 +--------------------------------------------------------------------------*/
int set_breakpoint(struct andi_servo *board, int channel, BOOLEAN relative)
{
	LG(TRACE,
	   "int set_breakpoint(struct andi_servo *board, int channel, BOOLEAN relative)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int load_filter(struct andi_servo *board, int channel,           |
 |                    struct LM629_Filter *filter)                     |
 +--------------------------------------------------------------------*/
int load_filter(struct andi_servo *board, int channel)
{
	struct LM629_Filter *filter;
	int commandword;
	int retval;

	LG(TRACE, "int load_filter(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
		filter = board->Channel1->NewFilter;
		board->Channel1->filter_updated = FALSE;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
		filter = board->Channel0->NewFilter;
		board->Channel0->filter_updated = FALSE;
	}

	print_filter(filter, buffer);
	L("Load this filter :\n%s\n", buffer);

	OUT(LFIL, command);

	CHECK_BUSY;

	OUT(((filter->dterm - 1) && 0x00FF), data);

	commandword = 0;
	if (filter->kp)
		commandword |= LOAD_Kp;
	if (filter->ki)
		commandword |= LOAD_Ki;
	if (filter->kd)
		commandword |= LOAD_Kd;
	if (filter->il)
		commandword |= LOAD_Il;

	OUT(commandword, data);

	if (filter->kp)
	{
		CHECK_BUSY;
		OUT(((filter->kp & 0xFF00) >> 8), data);
		OUT((filter->kp & 0x00FF), data);
	}

	if (filter->ki)
	{
		CHECK_BUSY;
		OUT(((filter->ki & 0xFF00) >> 8), data);
		OUT((filter->ki & 0x00FF), data);
	}

	if (filter->kd)
	{
		CHECK_BUSY;
		OUT(((filter->kd & 0xFF00) >> 8), data);
		OUT((filter->kd & 0x00FF), data);
	}

	if (filter->il)
	{
		CHECK_BUSY;
		OUT(((filter->il & 0xFF00) >> 8), data);
		OUT((filter->il & 0x00FF), data);
	}

	CHECK_BUSY;

	return 0;
}

/*---------------------------------------------------------------------+
 |    int update_filter(struct andi_servo *board, int channel)         |
 +--------------------------------------------------------------------*/
int update_filter(struct andi_servo *board, int channel)
{
	int retval;

	LG(TRACE, "int update_filter(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	OUT(UDF, command);

	CHECK_BUSY;

	if (channel)
	{
		board->Channel1->Filter = board->Channel1->NewFilter;
		board->Channel1->filter_updated = TRUE;
	}
	else
	{
		board->Channel0->Filter = board->Channel0->NewFilter;
		board->Channel0->filter_updated = TRUE;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int load_trajectory(struct andi_servo *board, int channel,       |
 |                    struct LM629_Trajectory *trajectory)             |
 +--------------------------------------------------------------------*/
int load_trajectory(struct andi_servo *board, int channel)
{
	int commandword;
	int retval;
	struct LM629_Trajectory *trajectory;

	LG(TRACE,
	   "int load_trajectory(struct andi_servo *board, int channel, struct LM629_Trajectory *trajectory)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
		trajectory = board->Channel1->NewTrajectory;
		board->Channel1->trajectory_started = FALSE;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
		trajectory = board->Channel0->NewTrajectory;
		board->Channel0->trajectory_started = FALSE;
	}

	print_trajectory(trajectory, buffer);

	L("Load this trajectory :\n%s\n", buffer);

	OUT(LFIL, command);

	CHECK_BUSY;

	commandword = 0;

	if (trajectory->forward_dir)
		commandword |= FORWARD_DIRECTION;
	if (trajectory->velocity_mode)
		commandword |= VELOCITY_MODE;
	if (trajectory->stop_smooth)
		commandword |= SMOOTH_STOP;
	if (trajectory->stop_abrupt)
		commandword |= ABRUPT_STOP;
	if (trajectory->motor_off)
		commandword |= TURN_MOTOR_OFF;
	if (trajectory->load_acc)
		commandword |= LOAD_ACCELERATION;
	if (trajectory->load_vel)
		commandword |= LOAD_VELOCITY;
	if (trajectory->load_pos)
		commandword |= LOAD_POSITION;
	if (trajectory->acc_relative)
		commandword |= ACCELERATION_RELATIVE;
	if (trajectory->vel_relative)
		commandword |= VELOCITY_RELATIVE;
	if (trajectory->pos_relative)
		commandword |= POSITION_RELATIVE;

	OUT(((commandword & 0xFF00) >> 8), data);
	OUT((commandword & 0x00FF), data);

	CHECK_BUSY;

	if (trajectory->load_acc)
	{
		OUT(((trajectory->acc & 0xFF000000) >> 24), data);
		OUT(((trajectory->acc & 0x00FF0000) >> 16), data);

		CHECK_BUSY;

		OUT(((trajectory->acc & 0x0000FF00) >> 8), data);
		OUT((trajectory->acc & 0x000000FF), data);

		CHECK_BUSY;
	}

	if (trajectory->load_vel)
	{
		OUT(((trajectory->velocity & 0xFF000000) >> 24), data);
		OUT(((trajectory->velocity & 0x00FF0000) >> 16), data);

		CHECK_BUSY;

		OUT(((trajectory->velocity & 0x0000FF00) >> 8), data);
		OUT((trajectory->velocity & 0x000000FF), data);

		CHECK_BUSY;
	}

	if (trajectory->load_pos)
	{
		OUT(((trajectory->position & 0xFF000000) >> 24), data);
		OUT(((trajectory->position & 0x00FF0000) >> 16), data);

		CHECK_BUSY;

		OUT(((trajectory->position & 0x0000FF00) >> 8), data);
		OUT((trajectory->position & 0x000000FF), data);

		CHECK_BUSY;
	}

	CHECK_BUSY;

	return 0;
}

/*---------------------------------------------------------------------+
 |    int start_trajectory(struct andi_servo *board, int channel)      |
 +--------------------------------------------------------------------*/
int start_trajectory(struct andi_servo *board, int channel)
{
	int retval;
	LG(TRACE, "int start_trajectory(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	OUT(STT, command);

	CHECK_BUSY;

	if (channel)
	{
		board->Channel1->Trajectory = board->Channel1->NewTrajectory;
		board->Channel1->trajectory_started = TRUE;
	}
	else
	{
		board->Channel0->Trajectory = board->Channel0->NewTrajectory;
		board->Channel0->trajectory_started = TRUE;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |  int get_status(struct andi_servo *board, int channel, int *status) |
 +--------------------------------------------------------------------*/
int get_status(struct andi_servo *board, int channel, int *status)
{
	LG(TRACE,
	   "int get_status(struct andi_servo *board, int channel, int *status)\n");

	if (channel)
		*status = inb(board->base_address + COMMAND_1);
	else
		*status = inb(board->base_address + COMMAND_0);

	L("status : %02x\n", *status);

	return 0;
}

/*---------------------------------------------------------------------+
 | int get_signals(struct andi_servo *board, int channel, int *signals)|
 +--------------------------------------------------------------------*/
int get_signals(struct andi_servo *board, int channel, int *signals)
{

	int retval;
	LG(TRACE,
	   "int get_signals(struct andi_servo *board, int channel, int *signals)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	CHECK_BUSY;

	OUT(RDSIGS, command);

	CHECK_BUSY;

	*signals = inb(data);
	*signals <<= 8;
	*signals |= inb(data);

	L("signals = %04x\n", *signals);

	CHECK_BUSY;

	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_index_position(struct andi_servo *board, int channel,    |
 |                           int *index_position)                      |
 +--------------------------------------------------------------------*/
int get_index_position(struct andi_servo *board, int channel,
					   int *index_position)
{
	LG(TRACE,
	   "int get_index_position(struct andi_servo *board, int channel, int *index_position)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int set_index_position(struct andi_servo *board, int channel)    |
 +--------------------------------------------------------------------*/
int set_index_position(struct andi_servo *board, int channel)
{
	LG(TRACE,
	   "int set_index_position(struct andi_servo *board, int channel)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_desired_position(struct andi_servo *board, int channel,  |
 |                         int *desired_position)                      |
 +--------------------------------------------------------------------*/
int get_desired_position(struct andi_servo *board, int channel,
						 long *desired_position)
{
	LG(TRACE,
	   "int get_desired_position(struct andi_servo *board, int channel, int *desired_position)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_real_position(struct andi_servo *board, int channel,     |
 |                          int *real_position)                        |
 +--------------------------------------------------------------------*/
int get_real_position(struct andi_servo *board, int channel,
					  long *real_position)
{
	int retval;
	LG(TRACE,
	   "int get_real_position(struct andi_servo *board, int channel, int *real_position)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	CHECK_BUSY;

	OUT(RDRP, command);

	CHECK_BUSY;

	*real_position = (long) inb(data);
	*real_position <<= 8;
	*real_position |= (long) inb(data);
	*real_position <<= 8;

	CHECK_BUSY;

	*real_position |= (long) inb(data);
	*real_position <<= 8;
	*real_position |= (long) inb(data);

	L("real position = %08lx\n", *real_position);

	CHECK_BUSY;

	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_desired_velocity(struct andi_servo *board, int channel,  |
 |                         int *desired_velocity)                      |
 +--------------------------------------------------------------------*/
int get_desired_velocity(struct andi_servo *board, int channel,
						 long *desired_velocity)
{
	LG(TRACE,
	   "int get_desired_velocity(struct andi_servo *board, int channel, int *desired_velocity)\n");
	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}
	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_real_velocity(struct andi_servo *board, int channel,     |
 |                          int *real_velocity)                        |
 +--------------------------------------------------------------------*/
int get_real_velocity(struct andi_servo *board, int channel, int *real_velocity)
{
	LG(TRACE,
	   "int get_real_velocity(struct andi_servo *board, int channel, int *real_velocity)\n");
	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*----------------------------------------------------------------------+
 |int set_irq_mask(struct andi_servo *board, int channel, int *irq_mask)|
 +---------------------------------------------------------------------*/
int set_irq_mask(struct andi_servo *board, int channel, int *irq_mask)
{
	LG(TRACE,
	   "int set_irq_mask(struct andi_servo *board, int channel, int *irq_mask)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*---------------------------------------------------------------------+
 |    int hard_reset(struct andi_servo *board, int channel)            |
 +--------------------------------------------------------------------*/
int hard_reset(struct andi_servo *board, int channel)
{
	int retval;
	LG(TRACE, "int hard_reset(struct andi_servo *board, int channel)\n");

	OUT(0x00, board->base_address + IRQENABLE);
	inb(board->base_address + CLEARIRQ);

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
		OUT(0x02, board->base_address + HARD_RESET);
		mdelay(200);
		OUT(0x00, board->base_address + HARD_RESET);
		mdelay(200);
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
		OUT(0x01, board->base_address + HARD_RESET);
		mdelay(200);
		OUT(0x00, board->base_address + HARD_RESET);
		mdelay(200);
	}

	retval = inb(command);

	if ((retval != 0xC4) && (retval != 0x84))
	{
		L("H/W reset failed, status code = %02x\nRetrying...\n", retval);
		hard_reset(board, channel);
		/*return -retval; */
	}

	OUT(RSTI, command);

	CHECK_BUSY;

	OUT(0x00, data);
	OUT(0x00, data);

	CHECK_BUSY;

	retval = inb(command);

	if ((retval != 0xC0) && (retval != 0x80))
	{
		L("H/W reset failed, status code = %02x\nRetrying...\n", retval);
		hard_reset(board, channel);
		/*return -retval; */
	}

	CHECK_BUSY;

	return 0;
}

/*-----------------------------------------------------------------------+
 |int set_PWM_brake(struct andi_servo *board, int channel, BOOLEAN brake)|
 +----------------------------------------------------------------------*/
int set_PWM_brake(struct andi_servo *board, int channel, BOOLEAN brake)
{
	LG(TRACE,
	   "int set_PWM_brake(struct andi_servo *board, int channel, BOOLEAN brake)\n");

	if (channel)
	{
		command = board->base_address + COMMAND_1;
		data = board->base_address + DATA_1;
	}
	else
	{
		command = board->base_address + COMMAND_0;
		data = board->base_address + DATA_0;
	}

	return 0;
}

/*-----------------------------------------------------------------------+
 |int get_position_error_threshold(struct andi_servo *board, int channel,|
 |                                 int *position_error_threshold)        |
 +----------------------------------------------------------------------*/
int get_position_error_threshold(struct andi_servo *board, int channel,
								 int *position_error_threshold)
{
	LG(TRACE,
	   "int get_position_error_threshold(struct andi_servo *board, int channel, int *position_error_threshold)\n");
	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_filter(struct andi_servo *board, int channel,            |
 |                   struct LM629_Filter *filter)                      |
 +--------------------------------------------------------------------*/
int get_filter(struct andi_servo *board, int channel,
			   struct LM629_Filter *filter)
{
	LG(TRACE,
	   "int get_filter(struct andi_servo *board, int channel, struct LM629_Filter *filter)\n");
	return 0;
}

/*---------------------------------------------------------------------+
 |    int get_trajectory(struct andi_servo *board, int channel,        |
 |                   struct LM629_Trajectory *trajectory)              |
 +--------------------------------------------------------------------*/
int get_trajectory(struct andi_servo *board, int channel,
				   struct LM629_Trajectory *trajectory)
{
	LG(TRACE,
	   "int get_trajectory(struct andi_servo *board, int channel, struct LM629_Trajectory *trajectory)\n");
	return 0;
}

/*----------------------------------------------------------------------+
 |int get_irq_mask(struct andi_servo *board, int channel, int *irq_mask)|
 +---------------------------------------------------------------------*/
int get_irq_mask(struct andi_servo *board, int channel, int *irq_mask)
{
	LG(TRACE,
	   "int get_irq_mask(struct andi_servo *board, int channel, int *irq_mask)\n");
	return 0;
}

/*-------------------------------------------------------------------------+
 |int get_PWM_brake(struct andi_servo *board, int channel, BOOLEAN * brake)|
 +------------------------------------------------------------------------*/
int get_PWM_brake(struct andi_servo *board, int channel, BOOLEAN * brake)
{
	LG(TRACE,
	   "int get_PWM_brake(struct andi_servo *board, int channel, BOOLEAN * brake)\n");
	return 0;
}

/*---------------------------------------------------------------------+
 |    int init_board(struct andi_servo *board)                         |
 +--------------------------------------------------------------------*/
int init_board(struct andi_servo *board)
{
	int retval;
	LG(TRACE, "int init_board(struct andi_servo *board)\n");

	retval = hard_reset(board, 0);
	if (retval < 0)
		return retval;

	retval = hard_reset(board, 1);
	if (retval < 0)
		return retval;

	retval = load_filter(board, 0);
	if (retval < 0)
		return retval;
	retval = load_filter(board, 1);
	if (retval < 0)
		return retval;

	retval = update_filter(board, 0);
	if (retval < 0)
		return retval;
	retval = update_filter(board, 1);
	if (retval < 0)
		return retval;

	retval = load_trajectory(board, 0);
	if (retval < 0)
		return retval;

	retval = load_trajectory(board, 1);
	if (retval < 0)
		return retval;

	retval = start_trajectory(board, 0);
	if (retval < 0)
		return retval;

	retval = start_trajectory(board, 1);
	if (retval < 0)
		return retval;

	return 0;
}

/*---------------------------------------------------------------------+
 |    char * print_filter(struct LM629_Filter *filter)                 |
 +--------------------------------------------------------------------*/
int print_filter(struct LM629_Filter *filter, char *buffer)
{
	int len;

	LG(TRACE, "char * print_filter(struct LM629_Filter *filter) ");

	len = sprintf(buffer, "LM629 PID Filter\n");
	len += sprintf(buffer + len, "\tDterm : %d\n", filter->dterm);
	len += sprintf(buffer + len, "\tKp    : %d\n", filter->kp);
	len += sprintf(buffer + len, "\tKi    : %d\n", filter->ki);
	len += sprintf(buffer + len, "\tKd    : %d\n", filter->kd);
	len += sprintf(buffer + len, "\tIl    : %d\n", filter->il);

	LG(TRACE, "%i characters stored in buffer\n", len);

	return len;
}

/*-----------------------------------------------------------------------+
 |int print_trajectory(struct LM629_Trajectory *trajectory, char *buffer)|
 +----------------------------------------------------------------------*/
int print_trajectory(struct LM629_Trajectory *trajectory, char *buffer)
{
	int len;

	LG(TRACE,
	   "int print_trajectory(struct LM629_Trajectory *trajectory, char *buffer) ");

	len = sprintf(buffer, "LM629 Trajectory\n");
	len += sprintf(buffer + len, "\tforward_dir   : %s\n",
				   trajectory->forward_dir ? "True" : "False");
	len += sprintf(buffer + len, "\tvelocity_mode : %s\n",
				   trajectory->velocity_mode ? "True" : "False");
	len += sprintf(buffer + len, "\tstop_smooth   : %s\n",
				   trajectory->stop_smooth ? "True" : "False");
	len += sprintf(buffer + len, "\tstop_abrupt   : %s\n",
				   trajectory->stop_abrupt ? "True" : "False");
	len += sprintf(buffer + len, "\tmotor_off     : %s\n",
				   trajectory->motor_off ? "True" : "False");
	len += sprintf(buffer + len, "\tload_acc      : %s\n",
				   trajectory->load_acc ? "True" : "False");
	len += sprintf(buffer + len, "\tload_vel      : %s\n",
				   trajectory->load_vel ? "True" : "False");
	len += sprintf(buffer + len, "\tload_pos      : %s\n",
				   trajectory->load_pos ? "True" : "False");
	len += sprintf(buffer + len, "\tacc_relative  : %s\n",
				   trajectory->acc_relative ? "True" : "False");
	len += sprintf(buffer + len, "\tvel_relative  : %s\n",
				   trajectory->vel_relative ? "True" : "False");
	len += sprintf(buffer + len, "\tpos_relative  : %s\n",
				   trajectory->pos_relative ? "True" : "False");

	len += sprintf(buffer + len, "\tacc           : %ld\n", trajectory->acc);
	len +=
		sprintf(buffer + len, "\tvelocity      : %ld\n", trajectory->velocity);
	len +=
		sprintf(buffer + len, "\tposition      : %ld\n", trajectory->position);

	LG(TRACE, "%i characters stored in buffer\n", len);
	return len;
}

/*---------------------------------------------------------------------+
 |    int print_status(int status, char *buffer)                       |
 +--------------------------------------------------------------------*/
int print_status(int status, char *buffer)
{
	int len;

	LG(TRACE, "int print_status(int status, char *buffer) ");

	len = sprintf(buffer, "LM629 Status\n");

	len += sprintf(buffer + len, "\tBusy                : %s\n",
				   (status & BUSY_BIT) ? "True" : "False");
	len += sprintf(buffer + len, "\tCommand Error       : %s\n",
				   (status & COMMAND_ERROR) ? "True" : "False");
	len += sprintf(buffer + len, "\tTrajectory Complete : %s\n",
				   (status & TRAJECTORY_COMPLETE) ? "True" : "False");
	len += sprintf(buffer + len, "\tIndex Pulse         : %s\n",
				   (status & INDEX_PULSE) ? "True" : "False");
	len += sprintf(buffer + len, "\tWraparound          : %s\n",
				   (status & WRAP_AROUND) ? "True" : "False");
	len += sprintf(buffer + len, "\tPosition Error      : %s\n",
				   (status & POSITION_ERROR) ? "True" : "False");
	len += sprintf(buffer + len, "\tBreakpoint reached  : %s\n",
				   (status & BREAKPOINT_REACHED) ? "True" : "False");
	len += sprintf(buffer + len, "\tMotor Off           : %s\n",
				   (status & MOTOR_OFF) ? "True" : "False");

	LG(TRACE, "%i characters stored in buffer\n", len);
	return len;
}

/*---------------------------------------------------------------------+
 |    int print_signals(int signals, char *buffer)                       |
 +--------------------------------------------------------------------*/
int print_signals(int signals, char *buffer)
{
	int len;

	LG(TRACE, "int print_signals(int signals, char *buffer) ");

	len = sprintf(buffer, "LM629 Signals\n");

	len += sprintf(buffer + len, "\tBusy                : %s\n",
				   (signals & BUSY_BIT) ? "True" : "False");
	len += sprintf(buffer + len, "\tCommand Error       : %s\n",
				   (signals & COMMAND_ERROR) ? "True" : "False");
	len += sprintf(buffer + len, "\tTrajectory Complete : %s\n",
				   (signals & TRAJECTORY_COMPLETE) ? "True" : "False");
	len += sprintf(buffer + len, "\tIndex Pulse         : %s\n",
				   (signals & INDEX_PULSE) ? "True" : "False");
	len += sprintf(buffer + len, "\tWraparound          : %s\n",
				   (signals & WRAP_AROUND) ? "True" : "False");
	len += sprintf(buffer + len, "\tPosition Error      : %s\n",
				   (signals & POSITION_ERROR) ? "True" : "False");
	len += sprintf(buffer + len, "\tBreakpoint reached  : %s\n",
				   (signals & BREAKPOINT_REACHED) ? "True" : "False");
	len += sprintf(buffer + len, "\tMotor Off           : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tEight Bit Mode      : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tTurn off on pos.err.: %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tOn Target           : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tVelocity Mode       : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tForward Direction   : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tFilter Loaded       : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tAcceleration Loaded : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");
	len += sprintf(buffer + len, "\tHost Interrupt      : %s\n",
				   (signals & MOTOR_OFF) ? "True" : "False");

	LG(TRACE, "%i characters stored in buffer\n", len);
	return len;
}
