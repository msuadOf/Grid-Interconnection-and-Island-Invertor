/* Includes_BEGIN */
#include <math.h>
#include "spll_sogi.h"
/* Includes_END */

/* Externs_BEGIN */
/* extern double func(double a); */



//! \brief Resets internal storage data of the module
//! \param *spll_obj The SPLL_1PH_SOGI structure pointer
//! \return None
//!
static inline void SPLL_1PH_SOGI_reset(SPLL_1PH_SOGI *spll_obj)
{
    spll_obj->u[0]=(float32_t)(0.0);
    spll_obj->u[1]=(float32_t)(0.0);
    spll_obj->u[2]=(float32_t)(0.0);
    
    spll_obj->osg_u[0]=(float32_t)(0.0);
    spll_obj->osg_u[1]=(float32_t)(0.0);
    spll_obj->osg_u[2]=(float32_t)(0.0);
    
    spll_obj->osg_qu[0]=(float32_t)(0.0);
    spll_obj->osg_qu[1]=(float32_t)(0.0);
    spll_obj->osg_qu[2]=(float32_t)(0.0);
    
    spll_obj->u_Q[0]=(float32_t)(0.0);
    spll_obj->u_Q[1]=(float32_t)(0.0);
    
    spll_obj->u_D[0]=(float32_t)(0.0);
    spll_obj->u_D[1]=(float32_t)(0.0);
    
    spll_obj->ylf[0]=(float32_t)(0.0);
    spll_obj->ylf[1]=(float32_t)(0.0);
    
    spll_obj->fo=(float32_t)(0.0);
    
    spll_obj->theta=(float32_t)(0.0);
    
    spll_obj->sine=(float32_t)(0.0);
    spll_obj->cosine=(float32_t)(0.0);
}

//! \brief Calculates the SPLL_1PH_SOGI coefficient
//! \param *spll_obj The SPLL_1PH_SOGI structure
//! \return None
//!
static inline void SPLL_1PH_SOGI_coeff_calc(SPLL_1PH_SOGI *spll_obj)
{
    float32_t osgx,osgy,temp, wn;
    wn= spll_obj->fn *(float32_t) 2.0f * (float32_t) 3.14159265f;
    spll_obj->osg_coeff.osg_k=(float32_t)(0.5);
    osgx = (float32_t)(2.0f*0.5f*wn*spll_obj->delta_t);
    spll_obj->osg_coeff.osg_x=(float32_t)(osgx);
    osgy = (float32_t)(wn*spll_obj->delta_t*wn*spll_obj->delta_t);
    spll_obj->osg_coeff.osg_y=(float32_t)(osgy);
    temp = (float32_t)1.0/(osgx+osgy+4.0f);
    spll_obj->osg_coeff.osg_b0=((float32_t)osgx*temp);
    spll_obj->osg_coeff.osg_b2=((float32_t)(-1.0f)*spll_obj->osg_coeff.osg_b0);
    spll_obj->osg_coeff.osg_a1=((float32_t)(2.0*(4.0f-osgy))*temp);
    spll_obj->osg_coeff.osg_a2=((float32_t)(osgx-osgy-4)*temp);
    spll_obj->osg_coeff.osg_qb0=((float32_t)(0.5f*osgy)*temp);
    spll_obj->osg_coeff.osg_qb1=(spll_obj->osg_coeff.osg_qb0*(float32_t)(2.0));
    spll_obj->osg_coeff.osg_qb2=spll_obj->osg_coeff.osg_qb0;
}

//! \brief Configures the SPLL_1PH_SOGI module
//! \param *spll_obj The SPLL_1PH_SOGI structure
//! \param acFreq Nominal AC frequency for the SPLL Module
//! \param isrFrequency Frequency at which SPLL module is run
//! \param lpf_b0 B0 coefficient of LPF of SPLL
//! \param lpf_b1 B1 coefficient of LPF of SPLL
//! \return None
//!
static inline void SPLL_1PH_SOGI_config(SPLL_1PH_SOGI *spll_obj,
                         float32_t acFreq,
                         float32_t isrFrequency,
                         float32_t lpf_b0,
                         float32_t lpf_b1)
{
    spll_obj->fn=acFreq;
    spll_obj->delta_t=((1.0f)/isrFrequency);

    SPLL_1PH_SOGI_coeff_calc(spll_obj);

    spll_obj->lpf_coeff.b0=lpf_b0;
    spll_obj->lpf_coeff.b1=lpf_b1;
}

//! \brief Run the SPLL_1PH_SOGI module
//! \param *spll_obj The SPLL_1PH_SOGI structure pointer
//! \param acValue AC grid voltage in per unit (pu)
//! \return None
//!
static inline void SPLL_1PH_SOGI_run(SPLL_1PH_SOGI *spll_obj,
                                     float32_t acValue)
{
    // Update the spll_obj->u[0] with the grid value
    spll_obj->u[0]=acValue;

    //
    // Orthogonal Signal Generator
    //
    spll_obj->osg_u[0]=(spll_obj->osg_coeff.osg_b0*
                       (spll_obj->u[0]-spll_obj->u[2])) +
                       (spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]) +
                       (spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2]);

    spll_obj->osg_u[2]=spll_obj->osg_u[1];
    spll_obj->osg_u[1]=spll_obj->osg_u[0];

    spll_obj->osg_qu[0]=(spll_obj->osg_coeff.osg_qb0*spll_obj->u[0]) +
                        (spll_obj->osg_coeff.osg_qb1*spll_obj->u[1]) +
                        (spll_obj->osg_coeff.osg_qb2*spll_obj->u[2]) +
                        (spll_obj->osg_coeff.osg_a1*spll_obj->osg_qu[1]) +
                        (spll_obj->osg_coeff.osg_a2*spll_obj->osg_qu[2]);

    spll_obj->osg_qu[2]=spll_obj->osg_qu[1];
    spll_obj->osg_qu[1]=spll_obj->osg_qu[0];

    spll_obj->u[2]=spll_obj->u[1];
    spll_obj->u[1]=spll_obj->u[0];

    //
    // Park Transform from alpha beta to d-q axis
    //
    spll_obj->u_Q[0]=(spll_obj->cosine*spll_obj->osg_u[0]) +
                     (spll_obj->sine*spll_obj->osg_qu[0]);
    spll_obj->u_D[0]=(spll_obj->cosine*spll_obj->osg_qu[0]) -
                     (spll_obj->sine*spll_obj->osg_u[0]);

    //
    // Loop Filter
    //
    spll_obj->ylf[0]=spll_obj->ylf[1] +
                     (spll_obj->lpf_coeff.b0*spll_obj->u_Q[0]) +
                     (spll_obj->lpf_coeff.b1*spll_obj->u_Q[1]);
    spll_obj->ylf[1]=spll_obj->ylf[0];

    spll_obj->u_Q[1]=spll_obj->u_Q[0];

    //
    // VCO
    //
    spll_obj->fo=spll_obj->fn+spll_obj->ylf[0];

    spll_obj->theta=spll_obj->theta + (spll_obj->fo*spll_obj->delta_t)*
                       (float32_t)(2.0*3.1415926f);

    if(spll_obj->theta>(float32_t)(2.0*3.1415926f))
    {
        spll_obj->theta=spll_obj->theta - (float32_t)(2.0*3.1415926f);
        //spll_obj->theta=0;
    }


    spll_obj->sine=(float32_t)arm_sin_f32(spll_obj->theta);
    spll_obj->cosine=(float32_t)arm_cos_f32(spll_obj->theta);
}

SPLL_1PH_SOGI spll1;
/* Externs_END */

void spll_Start_wrapper(void)
{
/* Start_BEGIN */
SPLL_1PH_SOGI_reset(&spll1);
SPLL_1PH_SOGI_config(&spll1,
                     AC_FREQ,
                     ISR_FREQ,
                     B0_LPF,
                     B1_LPF);
/* Start_END */
}

void spll_Outputs_wrapper(const float *u0,
                          float *y0,
                          float *y1,
                          float *sinf)
{
/* Output_BEGIN */
SPLL_1PH_SOGI_run(&spll1,(float32_t)*u0);
    *sinf = spll1.sine;
    *y0 = spll1.u_D[0];
    *y1 = spll1.u_Q[0];
/* Output_END */
}

void spll_Terminate_wrapper(void)
{
/* Terminate_BEGIN */
/*
 * 此处显示自定义终止代码。
 */
/* Terminate_END */
}