#include "epcl_variables.h"

MotorParam::MotorParam(float Pole_Set, float Rs_Set, float Lds_Set, float Lqs_Set, float Lamf_Set, float Jm_Set, float Bm_Set, float Is_Rated_Set, float V_Rated_Set){
    this->p = 0.5 * Pole_Set;
    this->Inv_p = 1. / this->p;
    this->Rs = Rs_Set;
    this->Lds = Lds_Set;
    this->Lqs = Lqs_Set;
    this->Lamf = Lamf_Set;
    this->Kt = 1.5 * this->p * this->Lamf;
    this->Inv_Kt = 1. / this->Kt;

    this->Jm = Jm_Set; this->Inv_Jm = 1./this->Jm;
    this->Bm = Bm_Set;
    this->Bm_per_Jm = this->Bm / this->Jm;

    // this->Ias = 0.; this->Ibs = 0.; this->Ics = 0.;
    // this->Idss = 0.; this->Iqss = 0.;
    // this->Idse = 0.; this->Iqse = 0.;
    this->Is_Rated = Is_Rated_Set;
    this->Is_Rated_Sq = this->Is_Rated * this->Is_Rated;
    this->Idse_Rated = (this->Lamf - std::sqrt(8 * (this->Lds - this->Lqs) * (this->Lds - this->Lqs) * this->Is_Rated_Sq + this->Lamf * this->Lamf)) / (-4 * (this->Lds - this->Lqs));
    this->Iqse_Rated = std::sqrt(this->Is_Rated_Sq - this->Idse_Rated * this->Idse_Rated);
    this->Te_Rated = 1.5 * this->p * (this->Lamf * this->Iqse_Rated + (this->Lds - this->Lqs) * this->Idse_Rated * this->Iqse_Rated);

    // this->Idsfe = 0.; this->Iqsfe = 0.;

    // IIR2Init(&(this->NF_Idse), K_NF, (1. / (Tsamp * Inj_Cnt_Max_Set) * TWOPI), 0.1, Tsamp);
    // IIR2Init(&(this->NF_Iqse), K_NF, (1. / (Tsamp * Inj_Cnt_Max_Set) * TWOPI), 0.1, Tsamp);

    // MavgInit(&(this->Idse_Mavg), 4);
    // MavgInit(&(this->Iqse_Mavg), 4);

    this->Vdc_Rated = V_Rated_Set; this->Inv_Vdc_Rated = 1/V_Rated_Set;

    // this->Thetar = 0.; this->Thetarm = 0.; this->Thetarm_Off = 0.;
    // this->Wr = 0.; this->Wrm = 0.; this->Wrpm = 0.;

    // int i, j = 0;
    // for (i=0; i<3; i++)
    // {
    //     for (j=0;j<21;j++)
    //     {
    //         if (MOTIdx==1) this->MTPA_LUT[i][j]=MTPA_LUT_8P12S[i][j];
    //         else if (MOTIdx==2) {
    //             this->MTPA_LUT[i][j]=MTPA_LUT_8P9S[i][j];
    //             this->Te_Rated = 4.5;
    //         }
    //         else;
    //     }
    // }
    // this->MTPA_dTe = this->MTPA_LUT[0][1] - this->MTPA_LUT[0][0];
}