#pragma once
#include"IKRig.h"
#include"IKPose.h"

namespace Animation
{
	class IKCompute {
	public:

		static void Run(IKRig& ik_rig, IKPose& ik_pose);

	private:

		static void Hip(const IKRig& ik_rig, IKPose& ik_pose);

		static void Limb(const IKRig& ik_rig, const IKChain& ik_chain, LimbIKData& ik_limb);

		static void LookTwist(const IKRig& ik_rig, const IKPoint& ik_point, LookTwistIKData& ik_lt);

		static void Spine(const IKRig& ik_rig, const IKChain& ik_chain, std::vector<LookTwistIKData>& ik_lts);
	};
}