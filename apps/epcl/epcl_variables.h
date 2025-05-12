#ifndef _EPCL_VARIABLES_H_
#define _EPCL_VARIABLES_H_

#include <cmath>

class MotorParam
{
    private:
    // float Ias, Ibs, Ics;
    // float Idss, Iqss;
    // float Idse, Iqse;
    float Is_Rated, Is_Rated_Sq;
    float Idse_Rated, Iqse_Rated;
    float Te_Rated;

    float Vdc_Rated, Inv_Vdc_Rated;

    //float Thetar, Thetarm, Thetarm_Off;
    //float Wr, Wrm, Wrpm;

    float p, Inv_p;  // pole, pole pair
    float Rs, Lds, Lqs;
    float Lamf, Kt, Inv_Kt;
    float Jm, Bm, Inv_Jm;
    float Bm_per_Jm;
    public:
    MotorParam();
    MotorParam(float Pole_Set, float Rs_Set, float Lds_Set, float Lqs_Set, float Lamf_Set, float Jm_Set, float Bm_Set, float Is_Rated_Set, float Inj_Cnt_Max_Set); // MOTIdx, Pole, Rs, Lds, Lqs, Lamf, Jm, Bm, Is_Rated, Inj_Cnt_Max_Set
    // float MTPA_LUT[3][21];
    // float MTPA_dTe;
};

#endif