/*
 * Tests usage correctness of CMSIS math functions
 * used by KFs.
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "testing.h"


char buf[MAX_CRIT_MSG_BYTES] = {0};


void drop_math_class(const char *msg, const char *file,
										 int line, fu32 code)
{
	log_err("%s:%d: %s: %d", file, line, msg, code);
#ifndef NDEBUG
	sprintf(buf, "%s:%d: %s: %d", file, line, msg, code);
	__BKPT(0);
#endif
}