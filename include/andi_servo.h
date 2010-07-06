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

#ifndef ANDISERVO_H
#define ANDISERVO_H

#include <asm/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <Lk.h>
#include <andi.h>

/*---------------------------------------------------------------------+
 |    I/O Port offsets for ANDI-SERVO board                            |
 +--------------------------------------------------------------------*/

#define COMMAND_0	0x0
#define DATA_0		0x1
#define COMMAND_1	0x2
#define DATA_1		0x3
#define PWM_BRAKES	0x4
#define HARD_RESET	0x5
#define	LED			0x6
#define IRQENABLE	0x6
#define IRQCAUSE	0x6
#define CLEARIRQ	0x7

/*---------------------------------------------------------------------+
 |   Bitmasks for ANDI-SERVO board                                     |
 +--------------------------------------------------------------------*/

/* IRQ Enable/LED Masks */
#define LED_MASK	0x01
#define IRQ_MASK	0x02

/* Interrupt Cause Masks */
#define CHANNEL1_LM629_IRQ	0x08
#define CHANNEL0_LM629_IRQ	0x04
#define CHANNEL1_THERMAL_IRQ	0x02
#define CHANNEL0_THERMAL_IRQ	0x01

/*---------------------------------------------------------------------+
 |    Bitmasks for LM629                                               |
 +--------------------------------------------------------------------*/

/* Interrupt Masks */
#define BREAKPOINT_INTERRUPT			0x40
#define POSITION_ERROR_INTERRUPT		0x20
#define WRAP_AROUND_INTERRUPT			0x10
#define INDEX_PULSE_INTERRUPT			0x08
#define TRAJECTORY_COMPLETE_INTERRUPT	0x04
#define	COMMAND_ERROR_INTERRUPT			0x02

/* Filter Control Word Masks */
#define LOAD_Kp		0x08
#define LOAD_Ki		0x04
#define	LOAD_Kd		0x02
#define LOAD_Il		0x01

/* Trajectory Control Word Masks */
#define FORWARD_DIRECTION	0x1000
#define VELOCITY_MODE		0x0800
#define	SMOOTH_STOP			0x0400
#define ABRUPT_STOP			0x0200
#define TURN_MOTOR_OFF		0x0100
#define LOAD_ACCELERATION	0x0020
#define ACCELERATION_RELATIVE	0x0010
#define LOAD_VELOCITY		0x0008
#define VELOCITY_RELATIVE	0x0004
#define LOAD_POSITION		0x0002
#define POSITION_RELATIVE	0x0001

/* Status Byte Masks */
#define MOTOR_OFF		0x80
#define BREAKPOINT_REACHED	0x40
#define POSITION_ERROR	0x20
#define WRAP_AROUND		0x10
#define INDEX_PULSE		0x08
#define TRAJECTORY_COMPLETE	0x04
#define COMMAND_ERROR	0x02
#define BUSY_BIT		0x01

/* Signals Register Masks */
#define HOST_INTERRUPT			0x8000
#define ACCELERATION_LOADED		0x4000
#define FILTER_LOADED			0x2000
#define FORWARD_DIRECTION		0x1000
#define VELOCITY_MODE			0x0800
#define ON_TARGET				0x0400
#define TURN_OFF_ON_POS_ERROR	0x0200
#define EIGHT_BIT_MODE			0x0100

/* 0x0080 to 0x0002 are as in Status Byte */
#define ACQUIRE_NEXT_INDEX		0x0001

/*---------------------------------------------------------------------+
 |    LM629 command Mnemonics                                          |
 +--------------------------------------------------------------------*/

#define RESET   0x0				/* Soft RESET command */
#define DFH     0x2				/* DeFine Home */
#define SIP     0x3				/* Set Index Position */
#define LPEI    0x1b			/* Interrupt on excessive position error */
#define LPES    0x1a			/* Stop on excessive error */
#define SBPA    0x20			/* Set BreakPoint, Absolute */
#define SBPR    0x21			/* Set BreakPoint, Relative */
#define MSKI    0x1c			/* MaSK Interrupts */
#define RSTI    0x1d			/* ReSeT Interrupts */
#define LFIL    0x1e			/* Load PID FILter parameters */
#define UDF     0x4				/* UpDate PID Filter */
#define LTRJ    0x1f			/* Load TRaJectory parameters */
#define STT		0x1				/* StarT Trajectory */
#define RDSIGS  0xc				/* ReaD SIGnalS register */
#define RDIP    0x9				/* ReaD Index Position */
#define RDDP    0x8				/* ReaD Desired Position */
#define RDRP    0xa				/* ReaD Real Position */
#define RDDV    0x7				/* ReaD Desired Velocity */
#define RDRV    0xb				/* ReaD Real Velocity */
#define RDSUM   0xd				/* ReaD integration SUM */

/*---------------------------------------------------------------------+
 |    Interrupt enable flags for s_set_irqmask()                       |
 +--------------------------------------------------------------------*/

#define I_ENA_BP        0x40	/* Ena interrupt on breakpoint          */
#define I_ENA_POSERR    0x20	/* Ena interrupt on excessive pos error */
#define I_ENA_WRAP      0x10	/* Ena interrupt on encoder wrap around */
#define I_ENA_INDEX     0x8		/* Ena interrupt on encoder index pulse */
#define I_ENA_DONE      0x4		/* Ena interrupt on completed trajetory */
#define I_ENA_CMDERR    0x2		/* Ena interrupt on command error       */
#define I_DISA_ALL      0x0		/* Disable all interrupt sources        */
#define I_CLR_ALL       0x0		/* Clear all interrupts                 */
#define I_ENA_ALL       0x7e	/* Enable all interrupt sources         */

/*---------------------------------------------------------------------+
 |    Error Codes                                                      |
 +--------------------------------------------------------------------*/

/*--------------------------------------------------------------------+
 |    Macros			                                              |
 +--------------------------------------------------------------------*/

#define OUT(data,port) \
	L("outb (%02x,%04x)\n",(int)data,(int)port); \
	outb(data,port);

#define CHECK_BUSY \
	retval = check_busy_bit(board, channel); \
	if (retval < 0) \
		return retval;

/*---------------------------------------------------------------------+
 |    Prototypes                                                       |
 +--------------------------------------------------------------------*/

/* Chip functions */
int check_busy_bit(struct andi_servo *board, int channel);
int soft_reset(struct andi_servo *board, int channel);
int define_home(struct andi_servo *board, int channel);
int set_position_error_threshold(struct andi_servo *board, int channel,
								 int position_error_threshold,
								 BOOLEAN stop_on_error);
int set_breakpoint(struct andi_servo *board, int channel, BOOLEAN relative);
int load_filter(struct andi_servo *board, int channel);
int update_filter(struct andi_servo *board, int channel);
int load_trajectory(struct andi_servo *board, int channel);
int start_trajectory(struct andi_servo *board, int channel);
int get_status(struct andi_servo *board, int channel, int *status);
int get_signals(struct andi_servo *board, int channel, int *signals);
int get_index_position(struct andi_servo *board, int channel,
					   int *index_position);
int set_index_position(struct andi_servo *board, int channel);
int get_desired_position(struct andi_servo *board, int channel,
						 long *desired_position);
int get_real_position(struct andi_servo *board, int channel,
					  long *real_position);
int get_desired_velocity(struct andi_servo *board, int channel,
						 long *desired_velocity);
int get_real_velocity(struct andi_servo *board, int channel,
					  int *real_velocity);
int set_irq_mask(struct andi_servo *board, int channel, int *irq_mask);

/* Board functions */
int hard_reset(struct andi_servo *board, int channel);
int set_PWM_brake(struct andi_servo *board, int channel, BOOLEAN brake);

/* Chip state model functions */
int get_position_error_threshold(struct andi_servo *board, int channel,
								 int *position_error_threshold);
int get_filter(struct andi_servo *board, int channel,
			   struct LM629_Filter *filter);
int get_trajectory(struct andi_servo *board, int channel,
				   struct LM629_Trajectory *trajectory);
int get_irq_mask(struct andi_servo *board, int channel, int *irq_mask);
int get_PWM_brake(struct andi_servo *board, int channel, BOOLEAN * brake);

/* Misc. functions */
int init_board(struct andi_servo *board);

int print_filter(struct LM629_Filter *filter, char *buffer);
int print_trajectory(struct LM629_Trajectory *trajectory, char *buffer);
int print_status(int status, char *buffer);
int print_signals(int signals, char *buffer);

#endif
