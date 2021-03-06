#include <stdlib.h>
#include <math.h>

#include "blm.h"
#include "lib.h"

void blm_AB_DQ(double R, double A, double B, double *D, double *Q)
{
	double		X, Y, rS, rC;

	X = A;
	Y = .577350269189626 * A + 1.15470053837925 * B;

	rS = sin(R);
	rC = cos(R);

	*D = rC * X + rS * Y;
	*Q = rC * Y - rS * X;
}

void blm_DQ_AB(double R, double D, double Q, double *A, double *B)
{
	double		X, Y, rS, rC;

	rS = sin(R);
	rC = cos(R);

	X = rC * D - rS * Q;
	Y = rS * D + rC * Q;

	*A = X;
	*B = - .5 * X + .866025403784439 * Y;
}

void blm_Enable(blm_t *m)
{
	double		Kv;

	m->Tsim = 0.;		/* Simulation time (Second) */
        m->dT = 1. / 30000.;	/* PWM period */
	m->sT = 1E-6;		/* Solver step */
	m->PWM_R = 2800;	/* PWM resolution */

	/* Winding resistance. (Ohm)
         * */
	m->R = 2.4E-1;

	/* Winding inductance. (Henry)
         * */
	m->Ld = 5.2E-4;
	m->Lq = 6.5E-4;

	/* BEMF constant. (Weber)
         * */
	Kv = 15.7; /* Total RPM per Volt */
        m->E = 60. / 2. / M_PI / sqrt(3.) / (Kv * m->Zp);

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 15;

	/* Thermal capacity. (Joule/K)
	 * */
	m->Ct = 30.;

	/* Thermal resistance. (K/Watt)
	 * */
	m->Rt = 0.5;

	/* Source voltage. (Volt)
	 * */
	m->U = 48.;

	/* Source internal resistance. (Ohm)
	 * */
	m->Rs = 0.2;

	/* Decoupling capacitance. (Farad)
	 * */
	m->Cb = 940E-6;

	/* Moment of inertia.
	 * */
	m->J = 5E-3;

	/* Load torque constants.
	 * */
	m->M[0] = 0E-3;
	m->M[1] = 5E-5;
	m->M[2] = 5E-7;
	m->M[3] = 2E-2;

	/* ADC conversion time (s).
	 * */
	m->T_ADC = 0.643E-6;

	/* Sensor time constant (s).
	 * */
	m->tau_I = 0.636E-6;
	m->tau_U = 25.53E-6;

	/* Hall sensor angles.
	 * */
	m->HS[0] = + 30.;
	m->HS[1] = + 150.;
	m->HS[2] = - 90.;

	/* Incremental Encoder.
	 * */
	m->EP_PPR = 2600;	/* Mechanical resolution */
	m->EP_Zq = 1.0;		/* Reduction ratio */

	/* External flags.
	 * */
	m->sync_F = 0;
}

void blm_Stop(blm_t *m)
{
	m->X[0] = 0.;	/* Axis D current (Ampere) */
	m->X[1] = 0.;	/* Axis Q current (Ampere) */
	m->X[2] = 0.;	/* Electrical Speed (Radian/Sec) */
	m->X[3] = 0.;	/* Electrical Position (Radian) */
	m->X[4] = 25.;	/* Temperature (Celsius) */
	m->X[5] = 0.;	/* Energy consumption (Joule) */
	m->X[6] = 5.;	/* DC link voltage (Volt) */

	m->X[7] = 0.;	/* Current Sensor A */
	m->X[8] = 0.;	/* Current Sensor B */
	m->X[9] = 0.;	/* Voltage Sensor A */
	m->X[10] = 0.;	/* Voltage Sensor B */
	m->X[11] = 0.;	/* Voltage Sensor C */

	m->X[12] = 0.;	/* QEP internals */
	m->X[13] = 1.;	/* QEP internals */

	m->VSI[0] = 0;
	m->VSI[1] = 0;
	m->VSI[2] = 0;
	m->surge_I = 0;
}

static void
blm_DQ_Equation(const blm_t *m, const double X[7], double D[7])
{
	double		UA, UB, UD, UQ, Q;
	double		R1, E1, MT, ML, wS;

	/* Thermal drift.
	 * */
	R1 = m->R  * (1. + 4E-3 * (X[4] - 25.));
	E1 = m->E  * (1. - 1E-3 * (X[4] - 25.));

	/* BEMF waveform distortion.
	 * */
	E1 += m->E * sin(X[3] * 3.) * 0E-2;

	/* Voltage from VSI.
	 * */
	Q = (m->VSI[0] + m->VSI[1] + m->VSI[2]) / 3.;
	UA = (m->VSI[0] - Q) * X[6];
	UB = (m->VSI[1] - Q) * X[6];

	blm_AB_DQ(X[3], UA, UB, &UD, &UQ);

	/* Energy consumption equation.
	 * */
	D[5] = 1.5 * (X[0] * UD + X[1] * UQ);

	/* DC link voltage equation.
	 * */
	D[6] = ((m->U - X[6]) / m->Rs - D[5] / X[6]) / m->Cb;

	/* Electrical equations.
	 * */
	UD += - R1 * X[0] + m->Lq * X[2] * X[1];
	UQ += - R1 * X[1] - m->Ld * X[2] * X[0] - E1 * X[2];

	if (m->HI_Z == 0) {

		D[0] = UD / m->Ld;
		D[1] = UQ / m->Lq;
	}
	else {
		D[0] = 0.;
		D[1] = 0.;
	}

	/* Torque production.
	 * */
	MT = 1.5 * m->Zp * (E1 - (m->Lq - m->Ld) * X[0]) * X[1];

	/* Load.
	 * */
	wS = X[2] / m->Zp;
	ML = m->M[0] - wS * (m->M[1] + fabs(wS) * m->M[2]);
	ML += (wS < 0.) ? m->M[3] : - m->M[3];

	/* Mechanical equations.
	 * */
	D[2] = m->Zp * (MT + ML) / m->J;
	D[3] = X[2];

	/* Thermal equation.
	 * */
	D[4] = (1.5f * R1 * (X[0] * X[0] + X[1] * X[1])
			+ (25. - X[4]) / m->Rt) / m->Ct;
}

static void
blm_Solve(blm_t *m, double dT)
{
	double		S1[7], S2[7], X2[7];
	double		iA, iB, uA, uB, uC;
	double		uMIN, KI, KU;
	int		j;

	if (m->HI_Z != 0) {

		m->X[0] = 0.;
		m->X[1] = 0.;
	}

	/* Second-order ODE solver.
	 * */

	blm_DQ_Equation(m, m->X, S1);

	for (j = 0; j < 7; ++j)
		X2[j] = m->X[j] + S1[j] * dT;

	blm_DQ_Equation(m, X2, S2);

	for (j = 0; j < 7; ++j)
		m->X[j] += (S1[j] + S2[j]) * dT / 2.;

	/* Sensor transient (fast).
	 * */
	KI = 1.0 - exp(- dT / m->tau_I);
	KU = 1.0 - exp(- dT / m->tau_U);

	blm_DQ_AB(m->X[3], m->X[0], m->X[1], &iA, &iB);

	m->X[7] += (iA - m->X[7]) * KI;
	m->X[8] += (iB - m->X[8]) * KI;

	if (m->HI_Z == 0) {

		uA = m->VSI[0] * m->X[6];
		uB = m->VSI[1] * m->X[6];
		uC = m->VSI[2] * m->X[6];
	}
	else {
		blm_DQ_AB(m->X[3], 0., m->E * m->X[2], &uA, &uB);

		uC = 0. - (uA + uB);

		uMIN = (uA < uB) ? uA : uB;
		uMIN = (uMIN < uC) ? uMIN : uC;

		uA += 0. - uMIN;
		uB += 0. - uMIN;
		uC += 0. - uMIN;
	}

	m->X[9]  += (uA - m->X[9])  * KU;
	m->X[10] += (uB - m->X[10]) * KU;
	m->X[11] += (uC - m->X[11]) * KU;
}

static void
blm_Solve_Split(blm_t *m, double dT)
{
	double		sT = m->sT;

	if (dT > 0.) {

		if (m->surge_I == 2) {

			/* Distortion of A.
			 * */
			m->X[7] += 12.;
			m->surge_I = 0;
		}

		if (m->surge_I == 3) {

			/* Distortion of B.
			 * */
			m->X[8] += 12.;
			m->surge_I = 0;
		}

		if (m->surge_I == 4) {

			/* Distortion of C.
			 * */
			m->surge_I = 0;
		}

		/* Split the long interval.
		 * */
		while (dT > sT) {

			blm_Solve(m, sT);
			dT -= sT;
		}

		blm_Solve(m, dT);

		/* Wrap the angular position.
		 * */
		m->X[3] = (m->X[3] < - M_PI) ? m->X[3] + 2. * M_PI :
			(m->X[3] > M_PI) ? m->X[3] - 2. * M_PI : m->X[3];
	}
}

static int
blm_ADC(double u)
{
	int		ADC;

	u += lib_gauss() * 5E-4;

	ADC = (int) (u * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;

	return ADC;
}

static void
blm_sample_HS(blm_t *m)
{
	double			EX, EY, SX, SY;
	int			HS = 0;

	EX = cos(m->X[3]);
	EY = sin(m->X[3]);

	SX = cos(m->HS[0] * M_PI / 180.);
	SY = sin(m->HS[0] * M_PI / 180.);

	HS |= (EX * SX + EY * SY < 0.) ? 1 : 0;

	SX = cos(m->HS[1] * M_PI / 180.);
	SY = sin(m->HS[1] * M_PI / 180.);

	HS |= (EX * SX + EY * SY < 0.) ? 2 : 0;

	SX = cos(m->HS[2] * M_PI / 180.);
	SY = sin(m->HS[2] * M_PI / 180.);

	HS |= (EX * SX + EY * SY < 0.) ? 4 : 0;

	m->pulse_HS = HS;
}

static void
blm_sample_EP(blm_t *m)
{
	double		A, B, INC;

	A = cos(m->X[12]) * cos(m->X[3]) + sin(m->X[12]) * sin(m->X[3]);
	B = cos(m->X[12]) * sin(m->X[3]) - sin(m->X[12]) * cos(m->X[3]);

	INC = atan2(B, A) / (2. * M_PI * (double) m->Zp);

	m->X[12] = m->X[3];
	m->X[13] += INC * (double) m->EP_PPR / m->EP_Zq;

	m->X[13] = (m->X[13] < 0.) ? m->X[13] + 65536. :
		(m->X[13] >= 65536.) ? m->X[13] - 65536. : m->X[13];

	m->pulse_EP = (int) (m->X[13]);
}

static void
blm_VSI_Sample(blm_t *m, int N)
{
	const double	range_I = 165.;
	const double	range_U = 60.;

	int		ADC;

	if (N == 0) {

		ADC = blm_ADC(m->X[7] / 2. / range_I + .5);
		m->ADC_IA = (ADC - 2047) * range_I / 2048.;

		ADC = blm_ADC(m->X[8] / 2. / range_I + .5);
		m->ADC_IB = (ADC - 2047) * range_I / 2048.;
	}
	else if (N == 1) {

		ADC = blm_ADC(m->X[6] / range_U);
		m->ADC_US = ADC * range_U / 4096.;

		ADC = blm_ADC(m->X[9] / range_U);
		m->ADC_UA = ADC * range_U / 4096.;
	}
	else if (N == 2) {

		ADC = blm_ADC(m->X[10] / range_U);
		m->ADC_UB = ADC * range_U / 4096.;

		ADC = blm_ADC(m->X[11] / range_U);
		m->ADC_UC = ADC * range_U / 4096.;

		blm_sample_HS(m);
		blm_sample_EP(m);
	}
}

static void
blm_VSI_Solve(blm_t *m)
{
	int		Tev[5], pm[5], n, k, tmp;
	double		tTIM, dT;

	tTIM = m->dT / m->PWM_R / 2.;

	/* ADC sampling.
	 * */
	Tev[0] = m->PWM_R - (int) (m->T_ADC / tTIM);
	Tev[1] = m->PWM_R - (int) (2. * m->T_ADC / tTIM);

	/* FETs switching.
	 * */
	Tev[2] = (m->PWM_A < 0) ? 0 : (m->PWM_A > m->PWM_R) ? m->PWM_R : m->PWM_A;
	Tev[3] = (m->PWM_B < 0) ? 0 : (m->PWM_B > m->PWM_R) ? m->PWM_R : m->PWM_B;
	Tev[4] = (m->PWM_C < 0) ? 0 : (m->PWM_C > m->PWM_R) ? m->PWM_R : m->PWM_C;

	for (n = 0; n < 5; ++n)
		pm[n] = n;

	/* Get sorted events.
	 * */
	for (n = 0; n < 5; ++n) {

		for (k = n + 1; k < 5; ++k) {

			if (Tev[pm[n]] < Tev[pm[k]]) {

				tmp = pm[n];
				pm[n] = pm[k];
				pm[k] = tmp;
			}
		}
	}

	/* Count Up.
	 * */
	blm_VSI_Sample(m, 0);

	tmp = m->PWM_R;

	for (n = 0; n < 5; ++n) {

		dT = tTIM * (tmp - Tev[pm[n]]);
		blm_Solve_Split(m, dT);

		if (pm[n] < 2) {

			blm_VSI_Sample(m, pm[n] + 1);
		}
		else {
			m->VSI[pm[n] - 2] = 1;
			m->surge_I = pm[n];
		}

		tmp = Tev[pm[n]];
	}

	dT = tTIM * (Tev[pm[4]]);
	blm_Solve_Split(m, dT);

	/* Keep only three of them.
	 * */
	for (n = 0, k = 0; n < 5; ++n) {

		tmp = pm[n];

		if (tmp != 0 && tmp != 1) {

			pm[k++] = tmp;
		}
	}

	/* Count Down.
	 * */
	for (n = 2, tmp = 0; n >= 0; --n) {

		dT = tTIM * (Tev[pm[n]] - tmp);
		blm_Solve_Split(m, dT);

		m->VSI[pm[n] - 2] = 0;
		m->surge_I = pm[n];

		tmp = Tev[pm[n]];
	}

	dT = tTIM * (m->PWM_R - Tev[pm[0]]);
	blm_Solve_Split(m, dT);

	/* Power.
	 * */
	m->iP = m->X[5] / m->dT;
	m->X[5] = 0.;
}

void blm_Update(blm_t *m)
{
	blm_VSI_Solve(m);
	m->Tsim += m->dT;
}

