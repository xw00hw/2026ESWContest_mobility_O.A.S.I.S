#include "joint_math.h"
#include <math.h>

#define RAD_PER_DEG 0.01745329251994329577
#define DEG_PER_RAD 57.295779513082320877

/* Exact C4_R8 inverse kinematics, same asin branch as the original viewer.
   R=Rz(-yaw)*Rx(pitch); r=24mm, centers=(-65,-36,+/-48),
   anchors=(-55,12,+/-48), rod length squared=2980 mm^2.
   Local crank phi offsets: D3=-phi1, D4=phi1, D5=phi2, D6=-phi2.
   Mirrored recorded pulse zeros; gain selected by JOINT_SERVO_RANGE_DEG.
   Acquisition and Experiment A fallback use the corrected 270deg model. */
int Joint_CrankAngles(double yaw_deg, double pitch_deg, double phi[2])
{
    if (!isfinite(yaw_deg) || !isfinite(pitch_deg) || yaw_deg < 0.0 ||
        yaw_deg > JOINT_YAW_LIMIT_MDEG/1000.0 ||
        pitch_deg < -JOINT_PITCH_LIMIT_MDEG/1000.0 ||
        pitch_deg > JOINT_PITCH_LIMIT_MDEG/1000.0) return 0;
    double cy=cos(-yaw_deg*RAD_PER_DEG), sy=sin(-yaw_deg*RAD_PER_DEG);
    double cp=cos(pitch_deg*RAD_PER_DEG), sp=sin(pitch_deg*RAD_PER_DEG);
    for(unsigned i=0;i<2;i++) {
        double sign=i==0 ? 1.0 : -1.0, az=sign*48.0;
        double x=-55.0*cy-12.0*sy*cp+az*sy*sp;
        double y=-55.0*sy+12.0*cy*cp-az*cy*sp;
        double z=12.0*sp+az*cp;
        double dx=x+65.0, dy=y+36.0, dz=z-az;
        double rho=hypot(dy,dz);
        if(rho<1e-10) return 0;
        double q=(dx*dx+dy*dy+dz*dz+576.0-2980.0)/(48.0*rho);
        if(fabs(q)>1.0+1e-9) return 0;
        if(q>1.0) q=1.0;
        if(q< -1.0) q=-1.0;
        phi[i]=(asin(q)-atan2(sign*dz,dy))*DEG_PER_RAD;
    }
    return 1;
}

int Joint_Pulses(int32_t yaw, int32_t pitch, uint16_t pulse[4])
{
    double phi[2];
    static const uint16_t center[4] = JOINT_BOOT_PULSES;
    if(!Joint_CrankAngles(yaw/1000.0,pitch/1000.0,phi)) return 0;
    for(unsigned i=0;i<4;i++) {
        double angle=i<2 ? phi[0] : -phi[1];
        if(i==0 || i==2) angle=-angle;
        double offset=angle*2000.0/JOINT_SERVO_RANGE_DEG;
        int32_t delta=(int32_t)(offset+(offset>=0.0 ? 0.5 : -0.5));
        int32_t us=center[i]+delta;
        if(us<500 || us>2500) return 0;
        pulse[i]=(uint16_t)us;
    }
    return 1;
}

uint32_t Joint_Duration(int32_t yaw0, int32_t pitch0, int32_t yaw1, int32_t pitch1)
{
    double previous[2], peak=0.0;
    if(!Joint_CrankAngles(yaw0/1000.0,pitch0/1000.0,previous)) return 0;
    /* Actual nonlinear quintic joint path, with conservative 25% speed margin.
       Runtime PWM slew guard independently checks every update. */
    for(unsigned step=1;step<=64;step++) {
        double t=step/64.0, f=t*t*t*(10.0+t*(-15.0+6.0*t));
        double phi[2];
        double y=(yaw0+(yaw1-yaw0)*f)/1000.0;
        double p=(pitch0+(pitch1-pitch0)*f)/1000.0;
        if(!Joint_CrankAngles(y,p,phi)) return 0;
        for(unsigned i=0;i<2;i++) {
            double derivative=fabs(phi[i]-previous[i])*64.0;
            if(derivative>peak) peak=derivative;
            previous[i]=phi[i];
        }
    }
    double duration=ceil(peak*1250.0/(MOTOR_RATE_MDEG_S/1000.0));
    return duration<200.0 ? 200U : (uint32_t)duration;
}

static int within(const uint16_t next[4], const uint16_t current[4], unsigned limit)
{
    for(unsigned i=0;i<4;i++) {
        int delta=(int)next[i]-(int)current[i];
        if(delta>(int)limit || delta< -(int)limit) return 0;
    }
    return 1;
}

int Joint_PosePulses(const int32_t pose[4], uint16_t pulse[4])
{
    uint16_t left[4], right[4];
    if(!Joint_Pulses(pose[0],pose[1],left) || !Joint_Pulses(pose[2],pose[3],right)) return 0;
    for(unsigned i=0;i<4;i++) pulse[i]=(i&1U) ? left[i] : right[i];
    return 1;
}

int Joint_PoseStep(const int32_t pose[4], int32_t next[4], unsigned limit, uint16_t pulse[4])
{
    uint16_t unused[4];
    for(unsigned side=0;side<2;side++) {
        unsigned j=2U*side;
        if(!Joint_LimitStep(pose[j],pose[j+1],&next[j],&next[j+1],limit,unused)) return 0;
    }
    return Joint_PosePulses(next,pulse);
}

int Joint_LimitStep(int32_t yaw, int32_t pitch, int32_t *ny, int32_t *np, unsigned limit, uint16_t pulse[4])
{
    uint16_t current[4], candidate[4];
    if(!Joint_Pulses(yaw,pitch,current) || !Joint_Pulses(*ny,*np,pulse)) return 0;
    if(within(pulse,current,limit)) return 1;
    int32_t dy=*ny-yaw, dp=*np-pitch, best_y=yaw, best_p=pitch;
    double low=0.0, high=1.0;
    for(unsigned i=0;i<20;i++) {
        double f=(low+high)*0.5;
        int32_t y=yaw+(int32_t)(dy*f), p=pitch+(int32_t)(dp*f);
        if(Joint_Pulses(y,p,candidate) && within(candidate,current,limit)) {
            low=f; best_y=y; best_p=p;
        } else high=f;
    }
    *ny=best_y; *np=best_p;
    return Joint_Pulses(*ny,*np,pulse);
}
