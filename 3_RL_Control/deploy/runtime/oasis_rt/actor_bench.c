/* Equivalence + latency check of actor_core.c without Python (runs on the Pi as is).
 *   actor_bench <inputs.f32> <outputs.f32> [iters]
 * inputs: N x OASIS_N_IN float32 raw histories; writes N x A actions (A = 2 or 4); then times `iters` calls
 * cycling through the inputs and prints min / median / p99 / max in microseconds. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "actor_weights.h"

void oasis_actor_forward(const float *x_raw, float *action);

static double now_us(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e6 + t.tv_nsec * 1e-3; }
static int cmp(const void *a, const void *b) { double d = *(const double *)a - *(const double *)b; return (d > 0) - (d < 0); }

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s inputs.f32 outputs.f32 [iters]\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "rb"); if (!f) { perror("inputs"); return 1; }
    fseek(f, 0, SEEK_END); long n = ftell(f) / (long)(sizeof(float) * OASIS_N_IN); fseek(f, 0, SEEK_SET);
    float *x = malloc(sizeof(float) * OASIS_N_IN * n), *y = malloc(sizeof(float) * (OASIS_NOUT / 2) * n);
    if (fread(x, sizeof(float) * OASIS_N_IN, n, f) != (size_t)n) { fprintf(stderr, "short read\n"); return 1; }
    fclose(f);
    for (long i = 0; i < n; i++) oasis_actor_forward(x + i * OASIS_N_IN, y + (OASIS_NOUT / 2) * i);
    FILE *g = fopen(argv[2], "wb"); fwrite(y, sizeof(float) * (OASIS_NOUT / 2), n, g); fclose(g);
    int iters = argc > 3 ? atoi(argv[3]) : 20000; double *t = malloc(sizeof(double) * iters); float a[OASIS_NOUT / 2];
    for (int i = 0; i < iters; i++) {
        double t0 = now_us(); oasis_actor_forward(x + (i % n) * OASIS_N_IN, a); t[i] = now_us() - t0;
    }
    qsort(t, iters, sizeof(double), cmp);
    printf("{\"n_inputs\": %ld, \"iters\": %d, \"us_min\": %.2f, \"us_median\": %.2f, \"us_p99\": %.2f, \"us_max\": %.2f}\n",
           n, iters, t[0], t[iters / 2], t[(int)(iters * 0.99)], t[iters - 1]);
    return 0;
}
