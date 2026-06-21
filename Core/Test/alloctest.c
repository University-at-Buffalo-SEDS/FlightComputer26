#include "fcapi.h"
#include "fctypes.h"

extern void *telemetryMalloc(size_t);
extern void telemetryFree(void *);

void test_telemetry_alloc(void)
{
	for (fu32 k = 8; k < 130000; ++k)
	{
		fu64 *p = telemetryMalloc(k);
		*p = k;

		if (*p % 30000 == 0)
		{
			blink(Green, false, 1);
		}

		telemetryFree(p);
	}

	blink(Blue, false, 4);
}