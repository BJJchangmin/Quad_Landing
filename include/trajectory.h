#ifndef __TRAJECTORY_H__
#define	__TRAJECTORY_H__

#include "globVariable.h"

class trajectory
{
    private:

        double squat_T_pause;
        double freq_squat;
        double squat_r0;
        double squat_rc;

        //Jumping Parameter
        double K_thrust;
        double zeta_thrust;
        double omega_thrust;
        double K_land;
        double zeta_land;
        double omega_land;

        

        double jump_r0;
        double jump_rc;
        double jump_rt;

        double jump_T_stand;
        double jump_T_crouch;
        double jump_T_pause;
        double jump_T_land;
        double jump_T_recover;
        double jump_qd_max;



    public:
        trajectory();
        ~trajectory();
        void Squat(double t,StateModel_* state_model);
        void Jumping(double t, StateModel_* state_model, int mode_admitt);
        void Hold(StateModel_* state_model);
        Vector3d admitance_Param_out(double omega_n, double zeta, double k);


};



#endif // !__TRAJECTORY_H__
