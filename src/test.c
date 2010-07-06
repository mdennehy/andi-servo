/*--------------------------------------------------------------------------------+
 |   Ajeco ANDI-SERVO Motion controller driver Test Program                       |
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

#include <stdio.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <andi.h>

#define FILENAME "/dev/andi_servo/filter0"

/*#define FILENAME "testfile" */

int atoi(char *buf);

int main(int argc, char **argv)
{
	FILE *servo;
	struct LM629_Filter filter = { 0, 0, 0, 0, 0 };

	int i = -1;

	servo = fopen(FILENAME, "r+b");
	printf("Opened file\n");
	fflush(stdout);

	filter.dterm = atoi(argv[1]);
	filter.kp = atoi(argv[2]);
	filter.ki = atoi(argv[3]);
	filter.kd = atoi(argv[4]);
	filter.il = atoi(argv[5]);

	printf("test %d,%d,%d,%d,%d\n", filter.dterm, filter.kp, filter.ki,
		   filter.kd, filter.il);
	fflush(stdout);

	i = fwrite((void *) &filter, sizeof (struct LM629_Filter), 1, servo);

	printf("Wrote %d bytes.\n", i);
	if (!i)
		perror("Error : ");
	fflush(stdout);

	printf("Updating filter.\n");
	fflush(stdout);
	i = ioctl(servo->_fileno, SERVO_UPDATE_FILTER);
	if (!i)
		perror("Error : ");
	fflush(stdout);

	fclose(servo);
	return 0;
}
