/* O.A.S.I.S bolster actor -- dependency-free C inference (fp32).
 * Weights/normalisation are generated into actor_weights.h by deploy/export/export_actor.py.
 * Build on the Pi 5:   gcc -O3 -mcpu=cortex-a76 -ffast-math -shared -fPIC actor_core.c -o liboasis_actor.so
 * Build on x86 (test): gcc -O3 -march=native -ffast-math -shared -fPIC actor_core.c -o liboasis_actor.so
 *
 * forward(): raw stacked observation history (oldest first, N_IN = (K+1)*OD floats, exactly what
 * oasis_rt.actor.History produces) -> action [c, d(, cp, dp)] in [-1, 1] (residual prior added if trained so).
 */
#include <math.h>
#include "actor_weights.h"

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void dense(const float *restrict w, const float *restrict b, const float *restrict x,
                  float *restrict y, int n_out, int n_in, int relu)
{
    for (int o = 0; o < n_out; o++) {
        const float *restrict row = w + (long)o * n_in;
        float acc = b[o];
        for (int i = 0; i < n_in; i++) acc += row[i] * x[i];      /* auto-vectorised (NEON / AVX) */
        y[o] = relu ? (acc > 0.f ? acc : 0.f) : acc;
    }
}

#define OASIS_A (OASIS_NOUT / 2)            /* actions: 2 (yaw c, d) or 4 (+ pitch cp, dp) */

int oasis_actor_dims(int *n_in, int *n_out) { *n_in = OASIS_N_IN; *n_out = OASIS_A; return OASIS_OD; }

void oasis_actor_forward(const float *restrict x_raw, float *restrict action)
{
    static float z[OASIS_N_IN], h1[OASIS_H1], h2[OASIS_H2], y[OASIS_NOUT];
    for (int i = 0; i < OASIS_N_IN; i++) z[i] = clampf((x_raw[i] - OASIS_MEAN[i]) * OASIS_INV_STD[i], -5.f, 5.f);
    dense(OASIS_W0, OASIS_B0, z, h1, OASIS_H1, OASIS_N_IN, 1);
    dense(OASIS_W1, OASIS_B1, h1, h2, OASIS_H2, OASIS_H1, 1);
    dense(OASIS_W2, OASIS_B2, h2, y, OASIS_NOUT, OASIS_H2, 0);
    /* residual prior [-1, ff(a_y pred), cp, dp]: pitch of the side the body will be pushed to (sim/rl/common.ff_baseline) */
    const float ay = x_raw[OASIS_N_IN - OASIS_OD + OASIS_FF_IDX];       /* newest frame, a_y prediction */
    const float m = fabsf(ay) > OASIS_FF_DEADBAND ? fabsf(ay) - OASIS_FF_DEADBAND : 0.f;
    const float sg = ay > 0.f ? 1.f : (ay < 0.f ? -1.f : 0.f);
    float prior[4] = {clampf(-1.f + OASIS_FF_BOTH * m, -1.f, 1.f), clampf(-OASIS_FF_GAIN * sg * m, -1.f, 1.f), 0.f, 0.f};
    if (OASIS_A == 4 && OASIS_PITCH_PRIOR != 0.f) {
        float s = clampf(-ay / 0.15f, -1.f, 1.f);
        float pL = OASIS_PITCH_PRIOR * (s > 0.f ? s : 0.f), pR = OASIS_PITCH_PRIOR * (s < 0.f ? -s : 0.f);
        prior[2] = 0.5f * (pL + pR); prior[3] = 0.5f * (pL - pR);
    }
    for (int k = 0; k < OASIS_A; k++) {
        float mu = tanhf(y[k]);
        action[k] = OASIS_RESIDUAL ? clampf(prior[k] + OASIS_RES_SCALE * mu, -1.f, 1.f) : mu;
    }
}

/* Same network without normalisation/prior/tanh: normalised input -> 4 raw outputs.
 * This is the part that would run on the Hailo; exposed here for equivalence tests. */
void oasis_actor_core(const float *restrict z, float *restrict y4)
{
    static float h1[OASIS_H1], h2[OASIS_H2];
    dense(OASIS_W0, OASIS_B0, z, h1, OASIS_H1, OASIS_N_IN, 1);
    dense(OASIS_W1, OASIS_B1, h1, h2, OASIS_H2, OASIS_H1, 1);
    dense(OASIS_W2, OASIS_B2, h2, y4, OASIS_NOUT, OASIS_H2, 0);
}
