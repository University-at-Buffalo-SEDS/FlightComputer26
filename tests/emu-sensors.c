/*
 * Tools for Distribution Task emulation.
 */

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "platform.h"
#include "deployment.h"


static inline float fgen(float low, float high)
{
  return low + (high - low)*((float)rand() / (float)(RAND_MAX + 1u));
}

static inline float abnormal(float bound)
{
  return (bound > 0) ? fgen(bound, bound + MAX_DEVIATION)
                     : fgen(bound - MAX_DEVIATION, bound);
}

static inline void sanitize(float *val, float bound)
{
  if (bound > 0) {
    if (*val > bound)
      *val = fgen(bound - MAX_DEVIATION, bound);
  } else {
    if (*val < bound)
      *val = fgen(bound, bound + MAX_DEVIATION);
  }
}

/*
 * https://www.literateprograms.org/box-muller_transform__c_.html
 */
static float noise(float mu, float sigma)
{
  static fu8 is_cached = 0;
  static double cache;

  double u, v, s;

  if (is_cached) {
    is_cached = 0;
    return (float)(sigma * cache + mu);
  }
  is_cached = 1;

  do {
    u = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
    v = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
    s = u*u + v*v;
  } while (s == 0 || s >= 1);

  s = sqrt(-2.0 * log(s) / s);

  cache = s * v;
  return (float)((u * s) * sigma + mu);
}

/*
 * h1: starting height, m
 * v1: starting velocity, m/s
 * a: acceleration on the interval, m/s^2
 * samp: amount of samples to produce
 * sigma: standard deviation for produced sample
 */
void produce_normal(float h1, float v1, float a, ULONG samp, float sigma)
{
  float alt = h1, vel = v1;
  float dt = 1.0f / SAMPLE_HZ;

  for (; samp > 0; --samp)
  {
    /* 
     * Theoretical values, under constant acceleration
     * That is, acceleration should manually specified each call
     */
    vel += a * dt;
    alt += vel * dt + 0.5f * a * dt * dt;

    /*
     * Relative noise prediction:
     * velocity < altitude < acceleration
     *
     * Make mu positive for biased outputs
     */
    float da = noise(0.0f, sigma);
    float dv = noise(0.0f, sigma * 0.4f);
    float dh = noise(0.0f, sigma * 0.8f);
    
    filter_t buf = {alt + dh, vel + dv, a + da};

    sanitize(&buf.alt, SN_MAX_ALT);
    // push sample to sensor-ukf exchange medium
    emu_sleep(SAMPLE_TK);
  }
}