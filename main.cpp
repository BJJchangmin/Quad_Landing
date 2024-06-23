#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for bool
//#include<unistd.h> //for usl eep
#include <math.h>
//#include <resource.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "controller.h"
#include "dataLogging.h"
#include "animation.h"
#include "kinematics.h"
#include "trajectory.h"


mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot

double simEndtime = 100;	// Simulation End Time
// state parameters
StateModel_ state_Model_FL;
StateModel_ state_Model_FR;
StateModel_ state_Model_RL;
StateModel_ state_Model_RR;

const int leg_FL_no = 0;
const int leg_FR_no = 3;
const int leg_RL_no = 6;
const int leg_RR_no = 9;

controller ctrl_FL; // other class is in main loop
controller ctrl_FR;
controller ctrl_RL;
controller ctrl_RR;

kinematics kin_FL;
kinematics kin_FR;
kinematics kin_RL;
kinematics kin_RR;

trajectory tra_FL;
trajectory tra_FR;
trajectory tra_RL;
trajectory tra_RR;

/* Controllers */
int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
int flag_admitt = 0;        // flag for switching ON/OFF admittance control
/* Trajectory selection*/
int cmd_motion_type = 0;   
int mode_admitt = 1; // This is must be ON

/***************** Main Controller *****************/
void mycontroller(const mjModel* m, mjData* d)
{
   
    
    double time_run = d->time;
    
    //Admittance Control
    ctrl_FL.admittanceCtrl(&state_Model_FL,5,2,10000, flag_admitt); //parameter(omega_n,zeta,k)
    ctrl_FR.admittanceCtrl(&state_Model_FL,5,2,10000, flag_admitt);
    ctrl_RL.admittanceCtrl(&state_Model_FL,5,2,10000, flag_admitt);
    ctrl_RR.admittanceCtrl(&state_Model_FL,5,2,10000, flag_admitt);

    // PID Control
    ctrl_FL.pid_gain_pos(5500,200, 150); //(kp,kd,freq)
    ctrl_FR.pid_gain_pos(5500,200, 150); 
    ctrl_RL.pid_gain_pos(5500,200, 150); 
    ctrl_RR.pid_gain_pos(5500,200, 150); 

    state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * ctrl_FL.PID_pos(&state_Model_FL); // RW position feedback
    state_Model_FR.tau_bi = state_Model_FR.jacbRW_trans * ctrl_FR.PID_pos(&state_Model_FR);
    state_Model_RL.tau_bi = state_Model_RL.jacbRW_trans * ctrl_RL.PID_pos(&state_Model_RL);
    state_Model_RR.tau_bi = state_Model_RR.jacbRW_trans * ctrl_RR.PID_pos(&state_Model_RR);
    // DOB control
    state_Model_FL.tau_bi = state_Model_FL.tau_bi + ctrl_FL.DOBRW(&state_Model_FL, 150, flag_DOB);
    state_Model_FR.tau_bi = state_Model_FR.tau_bi + ctrl_FR.DOBRW(&state_Model_FR, 150, flag_DOB);
    state_Model_RL.tau_bi = state_Model_RL.tau_bi + ctrl_RL.DOBRW(&state_Model_RL, 150, flag_DOB);
    state_Model_RR.tau_bi = state_Model_RR.tau_bi + ctrl_RR.DOBRW(&state_Model_RR, 150, flag_DOB);
    
    // Force Observer
    ctrl_FL.FOBRW(&state_Model_FL, 100); // Rotating Workspace Force Observer (RWFOB)
    ctrl_FR.FOBRW(&state_Model_FR, 100); 
    ctrl_RL.FOBRW(&state_Model_RL, 100); 
    ctrl_RR.FOBRW(&state_Model_RR, 100); 
    
   // Torque input Biarticular
    double Kp_Hip = 5000; double Kd_Hip = 20;
    d->ctrl[0] = Kp_Hip*(0-d->qpos[7]) + Kd_Hip*(0-d->qvel[7]); //FLHAA  
    d->ctrl[1] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1] ;
    d->ctrl[2] = state_Model_FL.tau_bi[1];

    d->ctrl[3] = Kp_Hip*(0-d->qpos[10]) + Kd_Hip*(0-d->qvel[10]); //FRHAA  
    d->ctrl[4] = state_Model_FR.tau_bi[0] + state_Model_FR.tau_bi[1] ;
    d->ctrl[5] = state_Model_FR.tau_bi[1];

    d->ctrl[6] = Kp_Hip*(0-d->qpos[13]) + Kd_Hip*(0-d->qvel[13]); //RLHAA  
    d->ctrl[7] = state_Model_RL.tau_bi[0] + state_Model_RL.tau_bi[1] ;
    d->ctrl[8] = state_Model_RL.tau_bi[1];

    d->ctrl[9] = Kp_Hip*(0-d->qpos[16]) + Kd_Hip*(0-d->qvel[16]); //FLHAA  
    d->ctrl[10] = state_Model_RR.tau_bi[0] + state_Model_RR.tau_bi[1] ;
    d->ctrl[11] = state_Model_RR.tau_bi[1];
        
    

    if (loop_index % data_frequency == 0) {  
        save_data_leg(m, d, &state_Model_FL, ctrl_FL.Data_Return(ctrl_FL),fid_FL);
        save_data_leg(m, d, &state_Model_FR, ctrl_FR.Data_Return(ctrl_FR),fid_FR);
        save_data_leg(m, d, &state_Model_RL, ctrl_RL.Data_Return(ctrl_RL),fid_RL);
        save_data_leg(m, d, &state_Model_RR, ctrl_RR.Data_Return(ctrl_RR),fid_RR);
        save_data_trunk(m, d, fid_Trunk);
    }
    loop_index += 1;
}

void Initialize()
{


    double arr_view[] = {-88.95, -17.5, 1.8, 0.04, 0.000000, 0.27};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    
    fid_FL = fopen(datapath_FL, "w");
    fid_FR = fopen(datapath_FR, "w");
    fid_RL = fopen(datapath_RL, "w");
    fid_RR = fopen(datapath_RR, "w");
    fid_Trunk = fopen(datapath_Trunk, "w");

    init_save_data_leg(fid_FL);
    init_save_data_leg(fid_FR);
    init_save_data_leg(fid_RL);
    init_save_data_leg (fid_RR);
    init_save_data_trunk(fid_Trunk);


    
    // Initialization
    
    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] =  0.3536;   // qpos[0,1,2] : trunk pos                                                                                                                 
                        // qpos[3,4,5.6] : trunk orientation quaternian
    
    d->qpos[3] = -0.73;
    d->qpos[4] = 0.73;
    d->qpos[5] = 0;
    d->qpos[6] = 0;

    d->qpos[7] = 0; //FLHAA         //d->ctrl[0] FLHAA
    d->qpos[8] = pi/4; //FLHIP       //d->ctrl[1] FLHIP
    d->qpos[9] = pi/2; //FLKNEE        //d->ctrl[2] FLKNEE
    d->qpos[10] = 0; //FRHAA        //d->ctrl[3] FRHAA
    d->qpos[11] = pi/4; //FRHIP        //d->ctrl[4] FRHIP
    d->qpos[12] = pi/2; //FRKNEE       //d->ctrl[5] FRKNEE
    d->qpos[13] = 0; //RLHAA        //d->ctrl[6] RLHAA
    d->qpos[14] = pi/4; //RLHIP        //d->ctrl[7] RLHIP
    d->qpos[15] = pi/2; //RLKNEE       //d->ctrl[8] RLKNEE
    d->qpos[16] = 0; //RRHAA        //d->ctrl[9] RRHAA
    d->qpos[17] = pi/4; //RRHIP        //d->ctrl[10] RRHIP
    d->qpos[18] = pi/2; //RRKNEE       //d->ctrl[11] RRKNEE
    

    kin_FL.model_param_cal(m, d, &state_Model_FL); // state init is before. Caution Error.
    kin_FR.model_param_cal(m, d, &state_Model_FR);
    kin_RL.model_param_cal(m, d, &state_Model_RL);
    kin_RR.model_param_cal(m, d, &state_Model_RR);
    
    kin_FL.state_init(m,d, &state_Model_FL);
    kin_FR.state_init(m,d, &state_Model_FR); 
    kin_RL.state_init(m,d, &state_Model_RL); 
    kin_RR.state_init(m,d, &state_Model_RR);  
    
    // custom controller
    mjcb_control = mycontroller;
}


/***************** Main Function *****************/
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);


    // make data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    Initialize();
    
    /***************** Simulation Loop *****************/
    // use the first while condition if you want to simulate for a period.
    int i = 0;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        // Otherwise add a cpu timer and exit this loop when it is time to render.

        mjtNum simstart = d->time;
        //printf(" %f  %f \n", d->ctrl[0], d->ctrl[1]);
        state_Model_FL.time = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            /* Trajectory Generation */
            
            if (cmd_motion_type == 0)   // Squat
            {
                tra_FL.Squat(d->time, &state_Model_FL);
                tra_FR.Squat(d->time, &state_Model_FR);
                tra_RL.Squat(d->time, &state_Model_RL);
                tra_RR.Squat(d->time, &state_Model_RR);
            }
            else if (cmd_motion_type == 1)   // Jumping
            {
                tra_FL.Jumping(d->time, &state_Model_FL, mode_admitt);
                tra_FR.Jumping(d->time, &state_Model_FR, mode_admitt);
                tra_RL.Jumping(d->time, &state_Model_RL, mode_admitt);
                tra_RR.Jumping(d->time, &state_Model_RR, mode_admitt);
            }
            else
            {  
                tra_FL.Hold(&state_Model_FL);  // Hold stance
                tra_FR.Hold(&state_Model_FR);
                tra_RL.Hold(&state_Model_RL);
                tra_RR.Hold(&state_Model_RR);
            }
            
            kin_FL.sensor_measure(m, d, &state_Model_FL, leg_FL_no); // get joint sensor data & calculate biarticular angles
            kin_FR.sensor_measure(m, d, &state_Model_FR, leg_FR_no);
            kin_RL.sensor_measure(m, d, &state_Model_RL, leg_RL_no);
            kin_RR.sensor_measure(m, d, &state_Model_RR, leg_RR_no);

            kin_FL.model_param_cal(m, d,&state_Model_FL); // calculate model parameters
            kin_FR.model_param_cal(m, d,&state_Model_FR);
            kin_RL.model_param_cal(m, d,&state_Model_RL);
            kin_RR.model_param_cal(m, d,&state_Model_RR);


            kin_FL.jacobianRW(&state_Model_FL);
            kin_FR.jacobianRW(&state_Model_FR);
            kin_RL.jacobianRW(&state_Model_RL);
            kin_RR.jacobianRW(&state_Model_RR);            // calculate RW Jacobian
            
            if (d->time < 10)
            {
                //printf("ref: %f \n", state_Model_FL.posRW_ref[1]);
                //printf(" qddot_bi_tustin(0) = %f ,%f ", state_Model_FL.qddot_bi_tustin[0],state_Model_FL.qddot_bi_tustin[1]);
                //printf(" qddot_bi(1) = %f ,%f \n", state_Model_FL.qddot_bi[0],state_Model_FL.qddot_bi[1]);
    
            }
            
            
            kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics
            kin_FR.fwdKinematics_cal(&state_Model_FR);
            kin_RL.fwdKinematics_cal(&state_Model_RL);
            kin_RR.fwdKinematics_cal(&state_Model_RR);

            mj_step(m, d);
           
            kin_FL.state_update(&state_Model_FL);
            kin_FR.state_update(&state_Model_FR);
            kin_RL.state_update(&state_Model_RL);
            kin_RR.state_update(&state_Model_RR);

            ctrl_FL.ctrl_update();
            ctrl_FR.ctrl_update();
            ctrl_RL.ctrl_update();
            ctrl_RR.ctrl_update();

        }

        if (d->time >= simEndtime) {
            fclose(fid_FL);
            fclose(fid_FR);
            fclose(fid_RL);
            fclose(fid_RR);
            fclose(fid_Trunk);

        }
        //printf("%f \n", state_Model_FL.deltaPos[0]);
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        


        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = 0;
        //cam.lookat[2] = 0;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}


