#pragma once
#include "..//..//pch.h"
#include "IKRig.h"
#include "..//IKTwoBoneJob.h"
#include "..//IKThreeBoneJob.h"

using namespace DirectX::SimpleMath;

// FIXUP£º
// There is a bug in the right hand at the run animation of ratio 0.681


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

		Vector3 mid_axis; // The normal of plane which is constructed by two bone, in mid joint space

		Vector3 debug_mid_axis;

		Vector3 pole; // In model space
	};

	struct HipIKData
	{
		HipIKData() :
			bind_height(0.0f),
			movement(Vector3::Zero),
			dir(Vector3::Zero),
			twist(0.0f)
		{}

		float bind_height;

		Vector3 movement;

		Vector3 dir;

		float twist;
	};

	struct LookTwistIKData
	{
		LookTwistIKData():
			look_dir(Vector3::Zero),
			twist_dir(Vector3::Zero)
		{}

		Vector3 look_dir;
		
		Vector3 twist_dir;
	};

	class IKPose
	{
	public:
		void ApplyRig(IKRig& rig, float dt);

		void ApplyHip(IKRig& ik_rig);

		void ApplyLimb(IKRig& ik_rig, const IKChain& ik_chain, const LimbIKData& ik_limb);

		void ApplyGrounding(Vector3& start, Vector3& target, float y_lmt);

		void ApplySpine(IKRig& ik_rig, const IKChain& ik_chain, const std::vector<LookTwistIKData>& ik_lts);

		void ApplyLookTwist(IKRig& ik_rig, const IKPoint& ik_point, const LookTwistIKData& ik_lt);

		void ApplySpringBone(IKRig& ik_rig, const IKChain& ik_chain, SpringBoneJob& spring_bone, float dt);

		HipIKData hip;

		// IK Data for limbs is first the Direction toward the End Effector,
		// The scaled length to the end effector, plus the direction that
		// the KNEE or ELBOW is pointing. For IK Targeting, Dir is FORWARD and
		// joint dir is UP
		LimbIKData leg_l;

		LimbIKData leg_r;

		LimbIKData arm_l;

		LimbIKData arm_r;

		LookTwistIKData foot_l;

		LookTwistIKData foot_r;

		LookTwistIKData hand_l;

		LookTwistIKData hand_r;

		LookTwistIKData head;

		std::vector<LookTwistIKData> spine;
	};
}