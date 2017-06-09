/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "hal/hal.h"
#include "hal_task.h"
#include "lib.h"
#include "pm_control.h"
#include "pm_math.h"
#include "shell.h"

#define AP_ERROR_BARRIER()	\
	{ if (pm.m_errno != PMC_OK) { break; } }

#define AP_PRINT_ERROR()	\
	{ if (pm.m_errno != PMC_OK) { printf("ERROR %i: %s" EOL, \
		pm.m_errno, pmc_strerror(pm.m_errno)); } }

#define AP_WAIT_FOR(expr)	\
	{ do { if (pm.m_errno != PMC_OK || (expr)) break; vTaskDelay(1); } while (1); }

#define AP_WAIT_FOR_IDLE()	AP_WAIT_FOR(pm.m_state == PMC_STATE_IDLE)

void pm_i_slew_rate_auto(const char *s);
void pm_i_gain_auto(const char *s);
void pm_const_E_wb(const char *s);
void pm_lu_threshold_auto(const char *s);
void pm_s_slew_rate_auto(const char *s);

SH_DEF(ap_identify_base)
{
	float			IMP[6];

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		printf("A0 %3f (A)" EOL, &pm.scal_A[0]);
		printf("B0 %3f (A)" EOL, &pm.scal_B[0]);

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.wave_DFT, IMP);
		pm.const_R += IMP[0];

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.wave_DFT, IMP);
		pm.const_R += IMP[0];

		printf("R %4e (Ohm)" EOL, &pm.const_R);

		pmc_request(&pm, PMC_STATE_WAVE_SINE);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

		if (fabsf(IMP[2]) < 45.f) {

			pm.const_Ld = IMP[0];
			pm.const_Lq = IMP[1];
		}
		else {
			pm.const_Ld = IMP[1];
			pm.const_Lq = IMP[0];
		}

		pmc_request(&pm, PMC_STATE_WAVE_SINE);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

		if (fabsf(IMP[2]) < 45.f) {

			pm.const_Ld = IMP[0];
			pm.const_Lq = IMP[1];
		}
		else {
			pm.const_Ld = IMP[1];
			pm.const_Lq = IMP[0];
		}

		pm.const_Ld_inversed = 1.f / pm.const_Ld;
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

		printf("Ld %4e (H)" EOL, &pm.const_Ld);
		printf("Lq %4e (H)" EOL, &pm.const_Lq);

		pm_i_slew_rate_auto(EOL);
		pm_i_gain_auto(EOL);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_identify_const_R_abc)
{
	float			temp[2], iSP, U, R[3], SD;
	int			xPWM;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		temp[0] = pm.wave_i_hold_X;
		temp[1] = pm.wave_i_hold_Y;

		iSP = pm.wave_i_hold_X;
		stof(&iSP, s);

		printf("SP %3f (A)" EOL, &iSP);

		U = iSP * pm.const_R / pm.const_U;
		xPWM = U * pm.pwm_resolution;
		U *= 100.f;

		printf("U: %i (tk) %2f %%" EOL, xPWM, &U);

		pm.wave_i_hold_X = iSP;
		pm.wave_i_hold_Y = 0.f;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.wave_DFT, R + 0);
		R[0] += pm.const_R;

		printf("R[a] %4e (Ohm)" EOL, &R[0]);

		pm.wave_i_hold_X = - .5f * iSP;
		pm.wave_i_hold_Y = .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.wave_DFT, R + 1);
		R[1] += pm.const_R;

		printf("R[b] %4e (Ohm)" EOL, &R[1]);

		pm.wave_i_hold_X = - .5f * iSP;
		pm.wave_i_hold_Y = - .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.wave_DFT, R + 2);
		R[2] += pm.const_R;

		printf("R[c] %4e (Ohm)" EOL, &R[2]);

		pm.const_R = (R[0] + R[1] + R[2]) / 3.f;
		SD = sqrtf((R[0] - pm.const_R) * (R[0] - pm.const_R)
			+ (R[1] - pm.const_R) * (R[1] - pm.const_R)
			+ (R[2] - pm.const_R) * (R[2] - pm.const_R));
		SD = 100.f * SD / pm.const_R;

		printf("R %4e (Ohm) SD %1f %%" EOL, &pm.const_R, &SD);

		pm.wave_i_hold_X = temp[0];
		pm.wave_i_hold_Y = temp[1];
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_identify_const_E)
{
	SH_ASSERT(pm.lu_region != PMC_LU_DISABLED);
	SH_ASSERT(ts.p_irq_callback == NULL);

	ts.av_IN[0] = &pm.lu_X[0];
	ts.av_IN[1] = &pm.lu_X[1];
	ts.av_IN[2] = &pm.lu_X[4];
	ts.av_IN[3] = &pm.drift_Q;

	ts.av_VAL[0] = 0.f;
	ts.av_VAL[1] = 0.f;
	ts.av_VAL[2] = 0.f;
	ts.av_VAL[3] = 0.f;

	ts.av_variable_N = 4;
	ts.av_sample_N = 0;
	ts.av_sample_MAX = pm.freq_hz * pm.tm_measure;

	halFence();
	ts.p_irq_callback = &ts_av_handler;

	do {
		AP_WAIT_FOR(ts.p_irq_callback == NULL);

		ts.av_VAL[0] /= (float) ts.av_sample_N;
		ts.av_VAL[1] /= (float) ts.av_sample_N;
		ts.av_VAL[2] /= (float) ts.av_sample_N;
		ts.av_VAL[3] /= (float) ts.av_sample_N;

		pm.const_E -= ts.av_VAL[3] / ts.av_VAL[2];
		pm_const_E_wb(EOL);
	}
	while (0);

	AP_PRINT_ERROR();
}

static void
ap_wait_for_settle(float wSP)
{
	const float		dT = .1f;
	const float		thresholdX4 = 20.f;
	float			leftT, X4, lastX4, dX4;

	pm.s_set_point = wSP;

	leftT = 3.f;
	lastX4 = ts_av_float_1(pm.lu_X + 4, dT);

	do {
		X4 = ts_av_float_1(pm.lu_X + 4, dT);

		AP_ERROR_BARRIER();

		dX4 = X4 - lastX4;
		lastX4 = X4;

		if (fabsf(dX4) < thresholdX4)
			break;

		leftT -= dT;
	}
	while (leftT > 0.f);
}

static void
ap_J_fsm_handler()
{
	float		temp;

	if (pm.m_errno != PMC_OK) {

		ts.p_irq_callback = NULL;
		return ;
	}

	switch (ts.ap_J_fsm_state) {

		case 0:
			ts.ap_J_fsm_state = 1;
			ts.ap_J_vars[0] = 0.f;
			ts.ap_J_vars[1] = pm.lu_X[4];

			pm.t_value = 0;
			pm.t_end = pm.freq_hz * ts.ap_J_measure_T;
			break;

		case 1:
			ts.ap_J_vars[0] += pm.lu_X[1];
			pm.t_value++;

			if (pm.t_value >= pm.t_end) {

				ts.ap_J_fsm_state = 2;
				ts.ap_J_vars[2] = pm.lu_X[4];
			}
			break;

		case 2:
			temp = pm.const_E * ts.ap_J_vars[0];
			temp *= 1.5f * pm.const_Zp * pm.const_Zp;
			temp *= pm.dT / (ts.ap_J_vars[2] - ts.ap_J_vars[1]);
			ts.ap_J_vars[3] = temp;
			ts.p_irq_callback = NULL;
			break;

		default:
			break;
	}
}

SH_DEF(ap_J_measure_T)
{
	stof(&ts.ap_J_measure_T, s);
	printf("%3e (Sec)" EOL, &ts.ap_J_measure_T);
}

SH_DEF(ap_identify_const_J)
{
	float			wSP1, wSP2, J;

	SH_ASSERT(pm.lu_region != PMC_LU_DISABLED);
	SH_ASSERT(pm.m_bitmask & PMC_BIT_SPEED_CONTROL_LOOP);
	SH_ASSERT((pm.m_bitmask & PMC_BIT_POSITION_CONTROL_LOOP) == 0);
	SH_ASSERT((pm.m_bitmask & PMC_BIT_FORCED_CONTROL) == 0);
	SH_ASSERT(ts.p_irq_callback == NULL);

	wSP1 = 2.f * pm.lu_threshold_high;
	wSP2 = 1.f / pm.const_E;

	if (strchr(s, '-') != NULL) {

		wSP1 = - wSP1;
		wSP2 = - wSP2;
	}

	do {
		AP_ERROR_BARRIER();

		ap_wait_for_settle(wSP1);
		AP_ERROR_BARRIER();

		ts.ap_J_fsm_state = 0;

		halFence();
		ts.p_irq_callback = &ap_J_fsm_handler;
		pm.s_set_point = wSP2;

		AP_WAIT_FOR(ts.p_irq_callback == NULL);
		AP_ERROR_BARRIER();

		J = ts.ap_J_vars[3];
		ts.ap_J_fsm_state = 0;

		halFence();
		ts.p_irq_callback = &ap_J_fsm_handler;
		pm.s_set_point = wSP1;

		AP_WAIT_FOR(ts.p_irq_callback == NULL);
		AP_ERROR_BARRIER();

		pm.const_J = (J + ts.ap_J_vars[3]) * .5f;
		printf("J %4e (kgm2)" EOL, &pm.const_J);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_blind_spinup)
{
	float			wSP;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	pm.m_bitmask |= PMC_BIT_SPEED_CONTROL_LOOP | PMC_BIT_FORCED_CONTROL;

	do {
		AP_ERROR_BARRIER();

		if (strchr(s, 'f') == NULL) {

			pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
			AP_WAIT_FOR_IDLE();

			pmc_request(&pm, PMC_STATE_WAVE_HOLD);
			AP_WAIT_FOR_IDLE();
		}

		pmc_request(&pm, PMC_STATE_START);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		wSP = 2.f * pm.lu_threshold_high;
		wSP = (strchr(s, '-') != NULL) ? - wSP : wSP;

		ap_wait_for_settle(wSP);
		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_probe_base)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		ap_identify_base(EOL);
		AP_ERROR_BARRIER();

		ap_blind_spinup("f");
		AP_ERROR_BARRIER();

		pm.m_bitmask &= ~PMC_BIT_FORCED_CONTROL;

		ap_identify_const_E(EOL);
		AP_ERROR_BARRIER();

		ap_wait_for_settle(1.f / pm.const_E);
		AP_ERROR_BARRIER();

		ap_identify_const_E(EOL);
		AP_ERROR_BARRIER();

		pm_lu_threshold_auto(EOL);

		ap_identify_const_J(EOL);
		AP_ERROR_BARRIER();

		pm_s_slew_rate_auto(EOL);
		pmc_request(&pm, PMC_STATE_STOP);

		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_probe_hfi)
{
	float			Qpc;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	Qpc = fabsf(1.f - pm.const_Ld / pm.const_Lq) * 100.f;

	printf("Q %.1f %%" EOL, &Qpc);

	do {
		pm.m_bitmask |= PMC_BIT_HIGH_FREQUENCY_INJECTION;

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_START);
		AP_WAIT_FOR_IDLE();

	}
	while (0);
}

