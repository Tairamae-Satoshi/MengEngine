#pragma once

#include "Skeleton.h"

namespace Animation
{
// IKAimJob rotates a joint so it aims at a target. Joint aim
// direction and up vectors can be different from joint axis. The job computes
// the transformation (rotation) that needs to be applied to the joints such
// that a provided forward vector (in joint local-space) aims at the target
// position (in skeleton model-space). Up vector (in joint local-space) is also
// used to keep the joint oriented in the same direction as the pole vector.
// The job also exposes an offset (in joint local-space) from where the forward
// vector should aim the target.
// Result is unstable if joint-to-target direction is parallel to pole vector,
// or if target is too close to joint position.
	struct IKAimJob
	{
		IKAimJob();

		bool Validate() const;

		bool Run();

		// Target position to aim at, in model-space
		Vector3 target;

		// Joint forward axis, in joint local-space, to be aimed at target position,
		// This vector shall be normalized, otherwise validation will fail.
		// Default is x axis.
		Vector3 forward;

		// Offset position from the joint in local-space, that will aim at target.
		Vector3 offset;

		// Joint up axis, in joint local-space, used to keep the joint oriented in the
		// same direction as the pole vector. Default is y axis.
		Vector3 up;

		// Pole vector, in model-space. The pole vector defines the direction
		// the up should point to.  Note that IK chain orientation will flip when
		// target vector and the pole vector are aligned/crossing each other. It's
		// caller responsibility to ensure that this doesn't happen.
		Vector3 pole_vector;

		// Twist_angle rotates joint around the target vector.
		// Default is 0.

		float twist_angle;

		// Weight given to the IK correction clamped in range [0,1]. This allows to
		// blend / interpolate from no IK applied (0 weight) to full IK (1).
		float weight;

		// Joint model-space matrix.
		const Matrix* joint;

		// Job output.

		// Output local-space joint correction quaternion. It needs to be multiplied
		// with joint local-space quaternion.
		Quaternion* joint_correction;

		// Optional boolean output value, set to true if target can be reached with IK
		// computations. Target is considered not reachable when target is between
		// joint and offset position.
		bool reached;

	};
}