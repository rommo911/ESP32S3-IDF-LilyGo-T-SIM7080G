#include "math.h"

typedef struct
{
    float a0, a1, a2, b1, b2;
    float z1, z2;
} Biquad_t;
extern Biquad_t bp_filter;
// ----------------- Helper functions -----------------

// Compute biquad coefficients (bandpass, butterworth-like) using bilinear transform
// center freq = sqrt(low*high), Q derived from bandwidth
void biquad_design_bandpass(Biquad_t &bq, float fs, float f1, float f2);
// Apply biquad to single sample (direct form IIA variant)
float biquad_process(Biquad_t &bq, float in);
// Convert DMP int32_t quaternion (Q30) to normalized float quaternion [w,x,y,z]
void quat_q30_to_float(const int32_t q_in[4], float q_out[4]);

// push into circular energy buffer
void energy_push(float val);

// compute RMS over last N samples (N <= energy_count)
float energy_rms_last(int N);
// Rotate sensor accel into world frame using quaternion
// sensor a_sensor (g) -> a_world (g)
void rotate_accel_world(const float q[4], const float a_sensor[3], float a_world[3]);

float angleDiff(float a, float b);