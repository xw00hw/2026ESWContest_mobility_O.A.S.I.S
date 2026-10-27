#ifndef OASIS_FALLBACK_CONTROL
#error "fallback_geometry is exclusively for the explicit-session fallback image"
#endif
#include "fallback_geometry.h"
#include "fallback_geometry_profile.h"

#define CAD_TEXT_IMPL(value) #value
#define CAD_TEXT(value) CAD_TEXT_IMPL(value)
typedef struct { int32_t yaw_mdeg, pitch_mdeg; } CadKnot;
static const CadKnot knots[FALLBACK_CAD_KNOT_COUNT]=FALLBACK_CAD_KNOTS;

const char *Fallback_CadDescriptor(void)
{
    return "#CAD,OASIS_A,v2,profile_sha256=" FALLBACK_CAD_PROFILE_SHA256
        ",yaw_max_mdeg=" CAD_TEXT(FALLBACK_CAD_YAW_MAX_MDEG)
        ",pitch_max_mdeg=" CAD_TEXT(FALLBACK_CAD_PITCH_MAX_MDEG)
        ",guard=CAD_KINEMATIC\r\n";
}

int32_t Fallback_CadPitchLimitMdeg(int32_t yaw_mdeg)
{
    if (yaw_mdeg<0 || yaw_mdeg>FALLBACK_CAD_YAW_MAX_MDEG) return -1;
    for (unsigned i=1;i<FALLBACK_CAD_KNOT_COUNT;i++) {
        if (yaw_mdeg<=knots[i].yaw_mdeg) {
            const CadKnot *a=&knots[i-1], *b=&knots[i];
            int32_t span=b->yaw_mdeg-a->yaw_mdeg;
            /* Profile generation verifies positive span and nonnegative
             * limits; retain a fail-closed guard against malformed tables. */
            if (span<=0 || a->pitch_mdeg<0 || b->pitch_mdeg<0) return -1;
            int64_t numerator=(int64_t)a->pitch_mdeg*(b->yaw_mdeg-yaw_mdeg)
                +(int64_t)b->pitch_mdeg*(yaw_mdeg-a->yaw_mdeg);
            int32_t value=(int32_t)(numerator/span);
            if (value<0 || value>FALLBACK_CAD_PITCH_MAX_MDEG) return -1;
            return value;
        }
    }
    return -1;
}

bool Fallback_CadPoseAllowed(const int32_t pose_mdeg[4])
{
    if (!pose_mdeg) return false;
    for (unsigned side=0;side<2;side++) {
        int32_t limit=Fallback_CadPitchLimitMdeg(pose_mdeg[2U*side]);
        int32_t pitch=pose_mdeg[2U*side+1U];
        if (limit<0 || pitch< -limit || pitch>limit) return false;
    }
    return true;
}

bool Fallback_CadRoundInward(int32_t pose_mdeg[4])
{
    if (!pose_mdeg) return false;
    for (unsigned side=0;side<2;side++) {
        int32_t limit=Fallback_CadPitchLimitMdeg(pose_mdeg[2U*side]);
        int32_t pitch=pose_mdeg[2U*side+1U];
        if (limit<0) return false;
        if (pitch>limit) {
            if ((int64_t)pitch-limit>1) return false;
            pose_mdeg[2U*side+1U]=limit;
        } else if (pitch< -limit) {
            if (-(int64_t)pitch-limit>1) return false;
            pose_mdeg[2U*side+1U]= -limit;
        }
    }
    return true;
}
