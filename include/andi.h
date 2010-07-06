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

#ifndef ANDI_H
#define ANDI_H

#include <asm/io.h>

#ifndef BOOLEAN
#define BOOLEAN int
#define TRUE 1
#define FALSE 0
#endif

/*---------------------------------------------------------------------+
 |    Structure definition for PID filter adjustment                   |
 +--------------------------------------------------------------------*/

struct LM629_Filter
{
	int dterm;					/* Derivative sampling interval factor  */
	int kp;						/* Proportional parameter               */
	int ki;						/* Integrating parameter                */
	int kd;						/* Derivating parameter                 */
	int il;						/* Integration limit                    */
};

/*---------------------------------------------------------------------+
 |    Structure definition containing all trajectory parameters        |
 +--------------------------------------------------------------------*/

struct LM629_Trajectory
{
	BOOLEAN forward_dir;		/* TRUE|FALSE (velocity mode only)      */
	BOOLEAN velocity_mode;		/* TRUE|FALSE (velocity|position mode)  */
	BOOLEAN stop_smooth;		/* TRUE|FALSE (smooth|no stop)          */
	BOOLEAN stop_abrupt;		/* TRUE|FALSE (abrupt|no stop)          */
	BOOLEAN motor_off;			/* TRUE|FALSE (motor off|on)            */
	BOOLEAN load_acc;			/* TRUE|FALSE (load acceleration|don't) */
	BOOLEAN load_vel;			/* TRUE|FALSE (load velocity|don't)     */
	BOOLEAN load_pos;			/* TRUE|FALSE (load position|don't)     */
	BOOLEAN acc_relative;		/* TRUE|FALSE (relative acc|absolute)   */
	BOOLEAN vel_relative;		/* TRUE|FALSE (relative vel|absolute)   */
	BOOLEAN pos_relative;		/* TRUE|FALSE (relative pos|absolute)   */
	long acc;					/* Acceleration, range: 0..MAXRANGE     */
	long velocity;				/* Position, range: -MAXRANGE..MAXRANGE */
	long position;				/* Position, range: -MAXRANGE..MAXRANGE */
};

/*---------------------------------------------------------------------+
 |    Structure definition for Motion controller channel               |
 +--------------------------------------------------------------------*/

struct LM629
{
	struct LM629_Filter *Filter;
	struct LM629_Filter *NewFilter;
	struct LM629_Trajectory *Trajectory;
	struct LM629_Trajectory *NewTrajectory;
	BOOLEAN filter_updated;
	BOOLEAN trajectory_started;
	BOOLEAN trajectory_complete;
	BOOLEAN pwm_brake;
	int position_error;
};

/*---------------------------------------------------------------------+
 |    Structure definition for ANDI-SERVO board                        |
 +--------------------------------------------------------------------*/

struct andi_servo
{
	struct LM629 *Channel0;
	struct LM629 *Channel1;
	BOOLEAN FaultLED;
	int base_address;
};

/*---------------------------------------------------------------------+
 |    ioctl() numbers                                                  |
 +--------------------------------------------------------------------*/

#ifndef SERVO_MAJOR
#define SERVO_MAJOR 120
#endif

/* board ioctls */

#define SERVO_HARD_RESET						_IO(SERVO_MAJOR,0)

#define SERVO_SET_BRAKES						_IOW(SERVO_MAJOR,2,int)
#define SERVO_SET_LED							_IOW(SERVO_MAJOR,3,int)
#define SERVO_SET_IRQ_ENABLE					_IOW(SERVO_MAJOR,4,int)

#define SERVO_GET_BRAKES						_IOR(SERVO_MAJOR,5,int)
#define SERVO_GET_LED							_IOR(SERVO_MAJOR,6,int)
#define SERVO_GET_IRQ_ENABLE					_IOR(SERVO_MAJOR,7,int)
#define SERVO_GET_IRQ_CAUSE						_IOR(SERVO_MAJOR,8,int)

/* channel ioctls */

#define	SERVO_SOFT_RESET						_IO(SERVO_MAJOR,1)
#define SERVO_SMOOTH_STOP						_IO(SERVO_MAJOR,9)
#define SERVO_ABRUPT_STOP						_IO(SERVO_MAJOR,10)
#define SERVO_MOTOR_OFF							_IO(SERVO_MAJOR,11)
#define SERVO_SET_HOME_POSITION					_IO(SERVO_MAJOR,12)

#define SERVO_SET_BRAKE							_IOW(SERVO_MAJOR,13,int)
#define SERVO_POSITION_MODE						_IOW(SERVO_MAJOR,14,int)
#define SERVO_FEEDBACK_MODE						_IOW(SERVO_MAJOR,15,int)
#define SERVO_SET_BREAKPOINT					_IOW(SERVO_MAJOR,16,int)
#define SERVO_SET_ACCELERATION					_IOW(SERVO_MAJOR,17,int)
#define SERVO_SET_POSITION_ERROR_THRESHOLD		_IOW(SERVO_MAJOR,18,int)
#define SERVO_SET_IRQ_MASK						_IOW(SERVO_MAJOR,19,int)

#define SERVO_GET_STATUS						_IOR(SERVO_MAJOR,20,int)
#define SERVO_GET_SIGNALS						_IOR(SERVO_MAJOR,21,int)
#define SERVO_GET_BRAKE							_IOR(SERVO_MAJOR,22,int)
#define SERVO_GET_BREAKPOINT					_IOR(SERVO_MAJOR,23,long *)
#define SERVO_GET_ACCELERATION					_IOR(SERVO_MAJOR,24,int)
#define SERVO_GET_POSITION_ERROR_THRESHOLD		_IOR(SERVO_MAJOR,25,int)
#define SERVO_GET_IRQ_MASK						_IOR(SERVO_MAJOR,26,int)

/* filter ioctls */

#define SERVO_UPDATE_FILTER						_IO(SERVO_MAJOR,27)

#define SERVO_CHECK_FILTER_UPDATED				_IOR(SERVO_MAJOR,28,int)

/* trajectory ioctls */

#define SERVO_START_TRAJECTORY					_IO(SERVO_MAJOR,29)

#define SERVO_REGISTER_FOR_NOTIFICATION			_IOW(SERVO_MAJOR,30,int)

#define SERVO_CHECK_TRAJECTORY_STARTED			_IOR(SERVO_MAJOR,31,int)
#define SERVO_CHECK_TRAJECTORY_COMPLETE			_IOR(SERVO_MAJOR,32,int)

#endif
