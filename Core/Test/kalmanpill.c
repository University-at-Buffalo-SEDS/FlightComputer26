/* Core/Inc/kalmanpill.c */

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

void mxcheck(const matrix *m, uint16_t rows, uint16_t cols,
						 const char *name)
{
  ok (m != NULL, "m null");
	ok (m->pData != NULL, "data null");
	ok (((uintptr_t)m->pData & 0x3) == 0, "data not 4-byte aligned");

  ok (m->pData != NULL, "data null");
  ok (m->numRows == rows, "rows mismatch");
	ok (m->numCols == cols, "cols mismatch");
}

void vecheck(const matrix *m, float *v, const char *name)
{
	ok (m != NULL, "m null");
	ok (m->numCols == 1, "not a vec");
  ok (m->pData != NULL, "data null");
  ok (((uintptr_t)m->pData & 0x3) == 0, "data not 4-byte aligned");

	ok (m->pData == v, "wrong data");
}

void offcheck(const matrix *c, const matrix *p, const char *name)
{
	ok (p, "p null");
	ok (c, "c null");

	conditional float *k = p->pData + p->numRows*p->numCols;
	ok (k == c->pData, "wrong offset");
}