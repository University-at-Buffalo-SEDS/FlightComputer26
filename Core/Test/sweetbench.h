/*
 * Benchmark for Flight Computer 26.
 *
 * The first version of this benchmark is designed to
 * measure the time it takes to execute between any two
 * points in one thread, including synchronization and
 * preemption overheads (sans out-of-order execution,
 * because Cortex-M33 has an in-order issue pipeline).
 */

#ifndef _SWEETBENCH_H
#define _SWEETBENCH_H

#ifdef FC_BENCHMARK

#include "platform.h"

#define _SB_ID		"SB "
#define _SB_MAX		32
#define _SB_COUNT 100
#define _SB_BUF		48

struct _sb_context {
	size_t count;
	fu32 setoff;
	fu32 min_ms;
};

static struct _sb_context **_sb_meta = NULL;

static void __attribute__((constructor)) _sb_init(void)
{
	_sb_meta = _sbrk(_SB_MAX * sizeof(struct _sb_context));
	memset(_sb_meta, 0, _SB_MAX * sizeof(struct _sb_context));
}

static inline void _sb_log(fu16 idx)
{
	char buf[_SB_BUF];
	const char *str = _SB_ID "task %u min: %u\n";

	/* Buffer is sufficiently large.
	 */
	sprintf(buf, str, idx, _sb_meta[idx]->min_ms);
	log_msg(buf);
}

static inline void
_sb_setoff(fu16 idx, size_t count, bool flush)
{
	if (idx >= _SB_MAX || _sb_meta[idx]->setoff != 0)
	{
		/* Either more than two setoffs before catch or bad ID.
		 */
		log_err(_SB_ID "error in %u", idx);
		return;
	}

	if (_sb_meta[idx]->count == 0)
	{
		_sb_meta[idx]->count = count;
		_sb_meta[idx]->min_ms = UINT_FAST32_MAX;

		if (flush)
		{
			invalidate_dcache();
		}
	}

	_sb_meta[idx]->setoff = now_ms();
}

static inline void _sb_catch(fu16 idx)
{
	fu32 elapsed = now_ms() - _sb_meta[idx]->setoff;

	if (_sb_meta[idx]->count == 0)
	{
		/* Catch before start is allowed for flexibility.
		 */
		return;
	}

	_sb_meta[idx]->min_ms = elapsed < _sb_meta[idx]->min_ms
															? elapsed
															: _sb_meta[idx]->min_ms;

	if (--_sb_meta[idx]->count == 0)
	{
		_sb_meta[idx]->setoff = 0;
		_sb_log(idx);
	}
}


/* API */

#define _sb_slf(x) x
#define _sb_ovl(_1, _2, _3, fn, ...) fn

#define _sb_so1(i) 				_sb_setoff((i), _SB_COUNT, true)
#define _sb_so2(i, c) 		_sb_setoff((i), (c), true)
#define _sb_so3(i, c, f) 	_sb_setoff((i), (c), (f))

#define sweetbench_start(...)	\
	_sb_slf(_sb_ovl(__VA_ARGS__, _sb_so3, _sb_so2, _sb_so1)(__VA_ARGS__))

#define sweetbench_catch(idx) _sb_catch((idx))

#else

#define sweetbench_start(...) ((void)0)
#define sweetbench_catch(idx) ((void)0)

#endif // FC_BENCHMARK


#endif // _SWEETBENCH_H