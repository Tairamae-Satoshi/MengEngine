#pragma once
#include "..//..//pch.h"
#include "IKRig.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	struct LimbIKData
	{
		LimbIKData() :
			length_scale(0.0f),
			dir(Vector3::Zero),
			joint_dir(Vector3::Zero)
		{}

		float length_scale;

		Vector3 dir;

		Vector3 joint_dir;
	};

	class IKPose
	{
	public:
		void ApplyRig(IKRig rig)
		{

		}

	private:

		void ApplyHip(IKRig rig)
		{

		}

		void ApplyLimb(IKRig rig, IKChain chain, LimbIKData limb)
		{

		}


		// IK Data for limbs is first the Direction toward the End Effector,
		// The scaled length to the end effector, plus the direction that
		// the KNEE or ELBOW is pointing. For IK Targeting, Dir is FORWARD and
		// joint dir is UP
		LimbIKData leg_l;
		LimbIKData leg_r;
		LimbIKData arm_l;
		LimbIKData arm_r;


	};
}