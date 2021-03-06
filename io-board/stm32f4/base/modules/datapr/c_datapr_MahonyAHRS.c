/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_MahonyAHRS.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    8-Jul-2014
  * @brief   Madgwick's implementation of Mayhony's AHRS algorithm.
  *          See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  ******************************************************************************/

/** 
	 Date     | Author       | Notes
	----------|--------------|----------------
	29/09/2011| SOH Madgwick | Initial release
	02/10/2011| SOH Madgwick | Optimised for reduced CPU load
***********************************************/

/** @addtogroup Datapr_MahonyAHRS
  * @{
  */

/** @addtogroup Datapr_MahonyAHRS
  *	\brief DImplementacão do algoritmo encontrado no artigo "Nonlinear Complementary FIlters on the Special Orthogonal Group",
  *	de Robert Mahony. Adaptado da implementacão feita por Madgwick, encontrado em http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/.
  *
  * Este site também aborda este filtro: http://www.olliw.eu/2013/imu-data-fusing/
  *
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "c_datapr_MahonyAHRS.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/

#define sampleFreq	200.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 7.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

/* Private variables ---------------------------------------------------------*/

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

/* Private function prototypes -----------------------------------------------*/

float invSqrt(float x);

/* Private functions ---------------------------------------------------------*/

/** \brief AHRS algorithm update
 *
 * Implementacao em quaternions do filtro complementar não linear encontrado no artigo "Nonlinear Complementary FIlters on the Special Orthogonal Group",
 *	de Robert Mahony.
 *
 * @param q orientacao atual estimada pelo filtro, em quaternions
 * @param velAngular_corrigida leitura do giroscopio corrigida pelo bias estimado pelo filtro
 * @param gx medida giroscopio eixo X
 * @param gy medida giroscopio eixo Y
 * @param gz medida giroscopio eixo Z
 * @param ax medida acelerometro eixo X
 * @param ay medida acelerometro eixo Y
 * @param az medida acelerometro eixo Z
 * @param mx medida magnetometro eixo X
 * @param my medida magnetometro eixo Y
 * @param mz medida magnetometro eixo Z
 */

void c_datapr_MahonyAHRSupdate(float * q, float gx, float gy, float gz, float ax, float ay, float az,
								float mx, float my, float mz,long sample_time_gyro_us) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float q0, q1, q2, q3;
	float sample_time_gyro;

	//Transform the sample time in us to s
	sample_time_gyro = (float)(sample_time_gyro_us)*1E-6;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		c_datapr_MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az,sample_time_gyro);
		return;
	}

	q0=q[0]; q1=q[1]; q2=q[2]; q3=q[3];

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * sample_time_gyro;;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * sample_time_gyro;
			integralFBz += twoKi * halfez * sample_time_gyro;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * sample_time_gyro);		// pre-multiply common factors
	gy *= (0.5f * sample_time_gyro);
	gz *= (0.5f * sample_time_gyro);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	q[0] = q0 * recipNorm;
	q[1] = q1 * recipNorm;
	q[2] = q2 * recipNorm;
	q[3] = q3 *recipNorm;

}


/** \brief IMU algorithm update
 * Implementacao em quaternions do filtro complementar não linear encontrado no artigo "Nonlinear Complementary FIlters on the Special Orthogonal Group",
 *	de Robert Mahony. Este algoritmo NAO USA MAGNETOMETRO.
 *
 *	Este metodo é usado internamente e é chamado quando a leitura dos magnetometros são todas iguais a zero.
 *
 * @param q orientacao atual estimada pelo filtro, em quaternions
 * @param velAngular_corrigida leitura do giroscopio corrigida pelo bias estimado pelo filtro
 * @param gx medida giroscopio eixo X
 * @param gy medida giroscopio eixo Y
 * @param gz medida giroscopio eixo Z
 * @param ax medida acelerometro eixo X
 * @param ay medida acelerometro eixo Y
 * @param az medida acelerometro eixo Z
 */

void c_datapr_MahonyAHRSupdateIMU(float * q, float gx, float gy, float gz, float ax, float ay, float az, long sample_time_gyro_us) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float q0, q1, q2, q3;

	q0=q[0]; q1=q[1]; q2=q[2]; q3=q[3];

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f * sample_time_gyro_us);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f * sample_time_gyro_us);
			integralFBz += twoKi * halfez * (1.0f * sample_time_gyro_us);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	//The other funtion have sampleTime like a parameter this not have.
	gx *= (0.5f * (1.0f * sample_time_gyro_us));		// pre-multiply common factors
	gy *= (0.5f * (1.0f * sample_time_gyro_us));
	gz *= (0.5f * (1.0f * sample_time_gyro_us));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	q[0] = q0 * recipNorm;
	q[1] = q1 * recipNorm;
	q[2] = q2 * recipNorm;
	q[3] = q3 *recipNorm;
}

/** \brief Fast inverse square-root
 *
 *	Para uso interno. Aclamada e mistica funcão utilizada no Quake III Arena.
 *	Realiza o equivalente a operacão 1/sqrt(x), porém de forma mais rápida e gloriosa.
 *	Se voce for geek de verdade então ganhará o dia lendo o link:
 *
 *	http://en.wikipedia.org/wiki/Fast_inverse_square_root
 *
 *
 * @param X 1/sqrt(x)
 */ 

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
  * @}
  */

/**
  * @}
  */
