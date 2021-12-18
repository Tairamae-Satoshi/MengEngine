#pragma once

#include"Skeleton.h"

namespace Animation
{
	// The job computes the transformations (rotations) that needs to be applied to
	// the first two joints of the chain (named start and middle joints) such that
	// the third joint (named end) reaches the provided target position (if
	// possible). The job outputs start and middle joint rotation corrections as
	// quaternions.
	struct IKTwoBoneJob
	{
		IKTwoBoneJob();

		bool Validate() const;

		bool Run();

		// Job input.

		// Target IK position, in model-space. This is the position the end of the
		// joint chain will try to reach.
		Vector3 target;

		// Normalized middle joint rotation axis, in middle joint local-space. Default
		// value is z axis. This axis is usually fixed for a given skeleton (as it's
		// in middle joint space). Its direction is defined like this: a positive
		// rotation around this axis will open the angle between the two bones. This
		// in turn also to define which side the two joints must bend. Job validation
		// will fail if mid_axis isn't normalized.
		//Vector3 mid_axis;

		// Pole vector, in model-space. The pole vector defines the direction the
		// middle joint should point to, allowing to control IK chain orientation.
		// Note that IK chain orientation will flip when target vector and the pole
		// vector are aligned/crossing each other. It's caller responsibility to
		// ensure that this doesn't happen.
		Vector3 pole_vector;

		// Twist_angle rotates IK chain around the vector define by start-to-target
		// vector. Default is 0.
		float twist_angle;

		// Soften ratio allows the chain to gradually fall behind the target
		// position. This prevents the joint chain from snapping into the final
		// position, softening the final degrees before the joint chain becomes flat.
		// This ratio represents the distance to the end, from which softening is
		// starting.
		float soften;

		// Weight given to the IK correction clamped in range [0,1]. This allows to
		// blend / interpolate from no IK applied (0 weight) to full IK (1).
		float weight;

		// Local mid_intial_rot
		Quaternion mid_initial_rot;

		// Model-space matrices of the start, middle and end joints of the chain.
		// The 3 joints should be ancestors. They don't need to be direct
		// ancestors though.
		Matrix start_joint;
		Matrix mid_joint;
		Matrix end_joint;

		// Job output.

		// Local-space corrections to apply to start and middle joints in order for
		// end joint to reach target position.
		// These quaternions must be multiplied to the local-space quaternion of their
		// respective joints.
		Quaternion start_joint_correction;
		Quaternion mid_joint_correction;

		// Optional boolean output value, set to true if target can be reached with IK
		// computations. Reachability is driven by bone chain length, soften ratio and
		// target distance. Target is considered unreached if weight is less than 1.
		bool reached;

	};
}