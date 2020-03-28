#include "sim_motor.h"
#include "data_structures.h"
#include "VESC_interface.h"
#include <math.h>
#include <stdio.h>

struct simParameters param;
//struct mc_values VESC;
float pipefds[2];

//BLDC Model
void initSim(char initType)
{
	switch(initType)
	{
		case 'd':
		{
			param.Ts = 1.0F / 20e3F;	// 20kHz control frequency can be 350 KHz too. Needs clarification
			param.J = 1e-3F;
			param.Ld = 20e-6F;
			param.Lq = 20e-6F;
			param.flux_linkage = 20e-2F;
			param.Rs = 20e-2F;
			param.v_max_adc = 48;
			param.pole_pairs = 8;
			printf("Initializing with default values... done\n");
			break;
		}
		case 'u':
		{
			printf("Enter user defined motor parameters below: \n");
			printf("Enter Bus Voltage in V in whole number: ");
			scanf("%d", &param.v_max_adc);
			printf("Enter Sampling period in seconds: ");
			scanf("%f", &param.Ts);
			printf("Enter Rotor+Load Inertia in Nm*s^2: ");
			scanf("%f", &param.J);
			printf("Enter Stator Resistance Phase to Neutral in Ohms: ");
			scanf("%f", &param.Rs);
			printf("Enter Inductance in d-Direction in H: ");
			scanf("%f", &param.Ld);
			printf("Enter Inductance in q-Direction in H: ");
			scanf("%f", &param.Lq);
			printf("Enter Flux linkage of the permanent magnets in mWb: ");
			scanf("%f", &param.flux_linkage);
			printf("Enter pole pairs: ");
			scanf("%d", &param.pole_pairs);
			printf("Saving user-defined motor properties....\n");
			break;
		}
		default:
		{
			printf("Input not recognised. Using default values instead. \n");
			param.Ts = 1.0F / 20e3F;	// 20kHz control frequency can be 350 KHz too. Needs clarification
			param.J = 1e-3F;
			param.Ld = 20e-6F;
			param.Lq = 20e-6F;
			param.flux_linkage = 20e-2F;
			param.Rs = 20e-2F;
			param.v_max_adc = 48;
			param.pole_pairs = 8;
			printf("Initializing with default values... done\n");
		}
	}
	
	param.km = 1.5 * param.pole_pairs;
	param.me = 0.0;
	param.va = 0.0;
	param.vb = 0.0;
	param.vc = 0.0;
	param.ia = 0.0;
	param.ib = 0.0;
	param.ic = 0.0;
	param.we = 0.0;
	param.v_alpha = 0.0;
	param.v_beta = 0.0;
	param.i_alpha = 0.0;
	param.i_beta = 0.0;
	param.id_int = 0.0;
	param.iq = 0.0;
	param.phi = 20.0;
	param.tsj = param.Ts / param.J;
	utils_fast_sincos_better(param.phi, (float*)&param.sin_phi,
		(float*)&param.cos_phi);
}

boolean runSim(float va, float vb, float T_mech)
{
	float arr[20];
	runElec(24.0, 24.0);
	runMech(T_mech);
	run_param_park_clark_inverse();
#ifdef DEBUG
		printf("\n***WARNING. DEBUG MESSAGES ARE ENABLED!***\n");
		printf("Change debug option() to off in CMakeLists.txt or comment our #define DEBUG in VESC_interface.h to disable debug!\n");
		printf("\nInput Voltage: %.2f V\r\n", (float)param.v_max_adc);
		//printf("Motor current: %.2f A\r\n", (float)abs(param.ia / 1000));
		printf("Electrical Torque: %.2f Nm\r\n", param.me / 100);
		printf("Mechanical Torque: %.2f Nm\r\n", T_mech);
		printf("Speed: %.2f RPM\r\n", ((param.we * 60) / (2 * M_PI)));
		printf("Speed: %.2f rad/sec\r\n", (float)param.we);
		printf("Rotor Position: %.2f rad\r\n", param.phi);
		printf("Current in d-direction: %.3f A\r\n", param.id/100);
		printf("Current in q-direction: %.3f A\r\n", param.iq/100);
		printf("Alpha axis voltage: %.2f V\r\n", (float)param.v_alpha);
		printf("Beta axis voltage: %.2f V\r\n", (float)param.v_beta);
		printf("Phase A voltage: %.2f V\r\n", (float)param.va);
		printf("Phase B voltage: %.2f V\r\n", (float)param.vb);
		printf("Phase C voltage: %.2f V\r\n", (float)param.vc);
		printf("Alpha axis current: %.3f A\r\n", param.i_alpha/1000);
		printf("Beta axis current: %.3f A\r\n", param.i_beta/1000);
		printf("Phase A current: %.3f A\r\n", param.ia/1000);
		printf("Phase B current: %.3f A\r\n", param.ib/1000);
		printf("Phase C current: %.3f A\r\n", param.ic/1000);
#endif
	arr[1] = (float)param.v_max_adc;
	close(pipefds[0]);
	//close(1);
	//dup(pipefds[1]);
	write(pipefds[1], arr, sizeof(arr));
	return True;
}
void runElec(float v_alpha, float v_beta)
{
	utils_fast_sincos_better(param.phi, (float*)&param.sin_phi,
		(float*)&param.cos_phi);

	param.vd = param.cos_phi * v_alpha + param.sin_phi * v_beta;
	param.vq = param.cos_phi * v_beta - param.sin_phi * v_alpha;

	// d axis current
	param.id_int += ((param.vd +
		param.we *
		param.pole_pairs *
		param.Lq * param.iq -
		param.Rs * param.id)
		* param.Ts) / param.Ld;
	param.id = param.id_int - param.flux_linkage / param.Ld;

	// q axis current
	param.iq += (param.vq -
		param.we *
		param.pole_pairs *
		(param.Ld * param.id + param.flux_linkage) -
		param.Rs * param.iq)
		* param.Ts / param.Lq;
}
void runMech(float ml)
{
	param.me = param.km * (param.flux_linkage +
		(param.Ld - param.Lq) *
		param.id) * param.iq;
	// omega
	float w_aux = param.we + param.tsj * (param.me - ml);

	if (abs(w_aux) < 0.0) 
	{
		param.we = 0;
	}
	else 
	{
		param.we = abs(w_aux);
	}

	// phi
	param.phi += param.we * param.Ts;
}void run_param_park_clark_inverse(void)
{
	//	Park Inverse
	param.i_alpha = param.cos_phi * param.id -
		param.sin_phi * param.iq;
	param.i_beta = param.cos_phi * param.iq +
		param.sin_phi * param.id;

	param.v_alpha = param.cos_phi * param.vd -
		param.sin_phi * param.vq;
	param.v_beta = param.cos_phi * param.vq +
		param.sin_phi * param.vd;

	//	Clark Inverse
	param.ia = param.i_alpha;
	param.ib = -0.5 * param.i_alpha + 0.866 * param.i_beta;
	param.ic = -0.5 * param.i_alpha - 0.866 * param.i_beta;

	param.va = param.v_alpha;
	param.vb = -0.5 * param.v_alpha + 0.866 * param.v_beta;
	param.vc = -0.5 * param.v_alpha - 0.866 * param.v_beta;
}


/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float* sin, float* cos)
{
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}

	while (angle > M_PI) {
		angle -= 2.0 * M_PI;
	}

	//compute sine
	if (angle < 0.0) {
		*sin = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		}
		else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	}
	else {
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		}
		else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5 * M_PI;
	if (angle > M_PI) {
		angle -= 2.0 * M_PI;
	}

	if (angle < 0.0) {
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		}
		else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	}
	else {
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		}
		else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	}
}

/*Salient Pole SM model*/
/*
void initSim(void)
{

	param.Ts = 1.0F / 20e3F;	// 20kHz control frequency can be 350 KHz too. Needs clarification
	param.J = 1e-3F;
	param.Ld = 60e-6F;
	param.Lq = 80e-6F;
	param.Psi = 6.821e-3F;	// this is calculated for 200 Kv motor. See readme file for more details
	param.Rs = 20e-3F;
}

boolean runSim(float vd, float vq, float T_mech)
{
	runElec(vd, vq);
	runMech(T_mech);
	return True;
}

void runElec(float vd, float vq)
{
	VESC.id += (vd + w * Lq * iq - Rs * id) * Ts / Ld;
	VESC.iq += (vq - w * Ld * id - Rs * iq - w * Psi) * Ts / Lq;
}

void runMech(float T_mech)
{
	VESC.T_elec = 1.5 * (Psi * iq + (Lq - Ld) * id * iq);
	VESC.omega += (Ts/J) * (VESC.T_elec - T_mech);
	VESC.phi += VESC.omega * Ts;
}

*/
