/*
 * DCM.h
 *
 *  Created on: 04.10.2013
 *      Author: tuuzdu
 */

#ifndef DCM_H_
#define DCM_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(x) ((x)>0?(x):-(x))

void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);

#endif /* DCM_H_ */
