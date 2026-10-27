#ifndef FALLBACK_GEOMETRY_H
#define FALLBACK_GEOMETRY_H
#include <stdbool.h>
#include <stdint.h>

/* Experiment A only. This is a CAD kinematic/clearance envelope, never a
 * motor load, contact-force, temperature or occupant-safety rating. The
 * conventional collection firmware and common R8 inverse math are unchanged. */
const char *Fallback_CadDescriptor(void);
/* Integer mdeg; negative result means yaw lies outside the compiled profile.
 * Positive rational interpolation is floored to avoid expanding the table. */
int32_t Fallback_CadPitchLimitMdeg(int32_t yaw_mdeg);
bool Fallback_CadPoseAllowed(const int32_t pose_mdeg[4]);
/* Only for a previously validated target's interpolated integer waypoint.
 * Permit at most 1mdeg inward pitch correction caused by axis quantization.
 * Never use this helper to accept an out-of-envelope external target. */
bool Fallback_CadRoundInward(int32_t pose_mdeg[4]);
#endif
