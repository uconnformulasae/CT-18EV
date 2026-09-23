/**
 * @file    soc_kf_tables.h
 * @brief   Molicel P50B at 25 degC. GENERATED - see spec Appendix A.
 *          OCV from the C/20 discharge sweep (the charge sweep is wrong by up
 *          to 230 mV). R0/R1 split by K = 0.6444, weakest below ~20% SoC.
 *          Breakpoints uniform so lookups index by multiply.
 */

#ifndef SOC_KF_TABLES_H
#define SOC_KF_TABLES_H

#define SOC_KF_N_BP   41
#define SOC_KF_TAU1_S 24.5f
#define SOC_KF_CAP_AH 4.9901f /* per cell, C/20 measured */

static const float soc_kf_ocv[SOC_KF_N_BP] = {
    2.518034f, 2.701741f, 2.815751f, 2.895162f, 2.966806f, 3.037089f, 3.103833f,
    3.162523f, 3.214335f, 3.262872f, 3.310932f, 3.358357f, 3.401686f, 3.440076f,
    3.475111f, 3.511550f, 3.542733f, 3.565035f, 3.598555f, 3.635808f, 3.671690f,
    3.707425f, 3.741456f, 3.769078f, 3.792770f, 3.814216f, 3.833872f, 3.852394f,
    3.872476f, 3.901744f, 3.937221f, 3.968490f, 3.999181f, 4.027092f, 4.049651f,
    4.064622f, 4.074081f, 4.083870f, 4.098676f, 4.126030f, 4.186791f,
};

static const float soc_kf_r0[SOC_KF_N_BP] = {
    0.046514f, 0.046499f, 0.046296f, 0.046106f, 0.032502f, 0.022171f, 0.018295f,
    0.015625f, 0.012700f, 0.011176f, 0.010729f, 0.010409f, 0.010422f, 0.010609f,
    0.010735f, 0.010849f, 0.010943f, 0.011066f, 0.011306f, 0.011545f, 0.011701f,
    0.011847f, 0.012019f, 0.012173f, 0.012285f, 0.012349f, 0.012012f, 0.011631f,
    0.011533f, 0.011503f, 0.011924f, 0.012374f, 0.012314f, 0.012166f, 0.012030f,
    0.011818f, 0.011397f, 0.011018f, 0.010993f, 0.011285f, 0.011160f,
};

static const float soc_kf_r1[SOC_KF_N_BP] = {
    0.025668f, 0.025659f, 0.025548f, 0.025443f, 0.017936f, 0.012235f, 0.010096f,
    0.008622f, 0.007009f, 0.006167f, 0.005920f, 0.005744f, 0.005751f, 0.005854f,
    0.005924f, 0.005987f, 0.006039f, 0.006107f, 0.006239f, 0.006371f, 0.006457f,
    0.006537f, 0.006633f, 0.006717f, 0.006779f, 0.006815f, 0.006628f, 0.006418f,
    0.006365f, 0.006348f, 0.006580f, 0.006828f, 0.006795f, 0.006714f, 0.006638f,
    0.006522f, 0.006289f, 0.006080f, 0.006067f, 0.006227f, 0.006158f,
};

#define SOC_KF_TEMP_MIN_C 10.0f
#define SOC_KF_TEMP_MAX_C 55.0f
#define SOC_KF_TEMP_N_BP  10

/* Resistance scale relative to 25 degC, 10 to 55 degC in 5 degC steps. */
static const float soc_kf_r_temp[SOC_KF_TEMP_N_BP] = {
    1.48f, 1.29f, 1.13f, 1.00f, 0.89f, 0.79f, 0.71f, 0.64f, 0.59f, 0.55f,
};

#endif /* SOC_KF_TABLES_H */
