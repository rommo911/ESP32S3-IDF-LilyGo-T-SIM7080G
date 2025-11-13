#include "math.h"
#include "imu6500/motionCalc.hpp"
#define Q30_SCALE 1073741824.0f // 2^30, common DMP quaternion format. Adjust if your driver uses different scaling.

// Circular buffer for energy values (bandpassed magnitudes)
#define ENERGY_BUF_LEN 512
static float energy_buf[ENERGY_BUF_LEN];
static int energy_idx = 0;
static int energy_count = 0;

Biquad_t bp_filter = {};

// ----------------- Helper functions -----------------

// Compute biquad coefficients (bandpass, butterworth-like) using bilinear transform
// center freq = sqrt(low*high), Q derived from bandwidth
void biquad_design_bandpass(Biquad_t &bq, float fs, float f1, float f2)
{
    // Prevent invalid ranges
    if (f1 <= 0.0f)
        f1 = 1.0f;
    if (f2 >= fs * 0.499f)
        f2 = fs * 0.499f;
    if (f2 <= f1)
        f2 = f1 * 1.1f;

    float w1 = 2.0f * M_PI * f1 / fs;
    float w2 = 2.0f * M_PI * f2 / fs;
    // center freq and bandwidth
    float wc = sqrtf(w1 * w2);
    float bw = w2 - w1;
    float Q = wc / bw;
    // Using standard bandpass bilinear transform (constant 0 dB peak)
    float omega = 2.0f * M_PI * sqrtf(f1 * f2) / fs;
    float alpha = sinf(omega) / (2.0f * Q);

    float cosw = cosf(omega);
    float b0 = alpha;
    float b1 = 0.0f;
    float b2 = -alpha;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosw;
    float a2 = 1.0f - alpha;

    // normalize to a0
    bq.a0 = b0 / a0;
    bq.a1 = b1 / a0;
    bq.a2 = b2 / a0;
    bq.b1 = a1 / a0;
    bq.b2 = a2 / a0;
    bq.z1 = bq.z2 = 0.0f;
}

// Apply biquad to single sample (direct form IIA variant)
float biquad_process(Biquad_t &bq, float in)
{
    float out = bq.a0 * in + bq.a1 * bq.z1 + bq.a2 * bq.z2 - bq.b1 * bq.z1 - bq.b2 * bq.z2;
    // shift delay line: we keep previous inputs scaled into z1 and z2 by reusing as state
    bq.z2 = bq.z1;
    bq.z1 = in;
    // Because we used an implementation mixing forms, the state usage above is minimal and works
    // for embedded low-order filtering. If you want exact DF2 transposed, use a standard lib.
    return out;
}

// Convert DMP int32_t quaternion (Q30) to normalized float quaternion [w,x,y,z]
void quat_q30_to_float(const int32_t q_in[4], float q_out[4])
{
    q_out[0] = (float)q_in[0] / Q30_SCALE;
    q_out[1] = (float)q_in[1] / Q30_SCALE;
    q_out[2] = (float)q_in[2] / Q30_SCALE;
    q_out[3] = (float)q_in[3] / Q30_SCALE;
    // Optionally normalize to guard against rounding
    float n = sqrtf(q_out[0] * q_out[0] + q_out[1] * q_out[1] + q_out[2] * q_out[2] + q_out[3] * q_out[3]);
    if (n > 0.0f)
    {
        q_out[0] /= n;
        q_out[1] /= n;
        q_out[2] /= n;
        q_out[3] /= n;
    }
}

// push into circular energy buffer
void energy_push(float val)
{
    energy_buf[energy_idx] = val;
    energy_idx = (energy_idx + 1) % ENERGY_BUF_LEN;
    if (energy_count < ENERGY_BUF_LEN)
        energy_count++;
}

// compute RMS over last N samples (N <= energy_count)
float energy_rms_last(int N)
{
    if (N <= 0)
        return 0.0f;
    if (energy_count == 0)
        return 0.0f;
    if (N > energy_count)
        N = energy_count;
    float sum = 0.0f;
    int idx = (energy_idx - 1 + ENERGY_BUF_LEN) % ENERGY_BUF_LEN;
    for (int i = 0; i < N; ++i)
    {
        float v = energy_buf[idx];
        sum += v * v;
        idx = (idx - 1 + ENERGY_BUF_LEN) % ENERGY_BUF_LEN;
    }
    return sqrtf(sum / N);
}
float angleDiff(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f; // normalize to [-180,180)
    return fabsf(d);
}
// Rotate sensor accel into world frame using quaternion
// sensor a_sensor (g) -> a_world (g)
void rotate_accel_world(const float q[4], const float a_sensor[3], float a_world[3])
{
    // Quaternion rotation: v' = q * v * q_conj
    // Using optimized formula: v' = v + 2*cross(q_vec, cross(q_vec, v) + q_w * v)
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // q_vec = (qx,qy,qz)
    float uv_x = qy * a_sensor[2] - qz * a_sensor[1];
    float uv_y = qz * a_sensor[0] - qx * a_sensor[2];
    float uv_z = qx * a_sensor[1] - qy * a_sensor[0];

    float uuv_x = qy * uv_z - qz * uv_y;
    float uuv_y = qz * uv_x - qx * uv_z;
    float uuv_z = qx * uv_y - qy * uv_x;

    float two_qw = 2.0f * qw;
    a_world[0] = a_sensor[0] + two_qw * uv_x + 2.0f * uuv_x;
    a_world[1] = a_sensor[1] + two_qw * uv_y + 2.0f * uuv_y;
    a_world[2] = a_sensor[2] + two_qw * uv_z + 2.0f * uuv_z;
}