#include "IKCompute.h"

namespace Animation
{
	void IKCompute::Run(IKRig& ik_rig, IKPose& ik_pose)
	{
		ik_rig.UpdateWorld();

		Hip(ik_rig, ik_pose);
		Limb(ik_rig, ik_rig.chains["leg_l"], ik_pose.leg_l);
		Limb(ik_rig, ik_rig.chains["leg_r"], ik_pose.leg_r);
		Limb(ik_rig, ik_rig.chains["arm_l"], ik_pose.arm_l);
		Limb(ik_rig, ik_rig.chains["arm_r"], ik_pose.arm_r);

		LookTwist(ik_rig, ik_rig.points["foot_l"], ik_pose.foot_l);
		LookTwist(ik_rig, ik_rig.points["foot_r"], ik_pose.foot_r);

		LookTwist(ik_rig, ik_rig.points["hand_l"], ik_pose.hand_l);
		LookTwist(ik_rig, ik_rig.points["hand_r"], ik_pose.hand_r);

		Spine(ik_rig, ik_rig.chains["spine"], ik_pose.spine);

		LookTwist(ik_rig, ik_rig.points["head"], ik_pose.head);
	}

	void IKCompute::Hip(const IKRig& ik_rig, IKPose& ik_pose)
	{
		// First thing we need is the Hip bone from the Animated Pose
		// Plus what the hip's Bind Pose as well.
		// We use these two states to determine what change the animation did to the tpose.
		unsigned int b_idx = ik_rig.points.at("hip").idx;
		Transform bind_pose_w = (ik_rig.tpose_world)[b_idx];
		Transform curr_pose_w = (ik_rig.pose_world)[b_idx];

		// Lets create the Quaternion Inverse Direction based on the
		// TBone's World Space rotation. We don't really know the orientation 
		// of the bone's starting rotation and our targets will have their own
		// orientation, so by doing this we can easily say no matter what the
		// default direction of the hip, we want to say all hips bones point 
		// at the FORWARD axis and the tail of the bone points UP.
		Quaternion q_inv = bind_pose_w.mRot.mValue.Inversed();
		Vector3 alt_fwd = Vector3::Transform(Vector3::Forward, q_inv);
		Vector3 alt_up = Vector3::Transform(Vector3::Up, q_inv);

		Vector3 pose_fwd = Vector3::Transform(alt_fwd, curr_pose_w.mRot.mValue);
		Vector3 pose_up = Vector3::Transform(alt_up, curr_pose_w.mRot.mValue);

		// With our directions known between our TPose and Animated Pose, Next we
		// start to calculate the Swing and Twist Values to swing our TPose into
		// The animation direction
		Quaternion swing = bind_pose_w.mRot.mValue * Quaternion::CreateFromVectors(Vector3::Forward, pose_fwd);

		Vector3 swing_up = Vector3::Transform(Vector3::Up, swing);
		float twist = Vector3::Angle(swing_up, pose_up);

		if (twist <= (0.01 * MathHelper::Pi / 180.0f)) {
			twist = 0.0f; // If Less the .01 Degree, dont bother twisting.
		}
		else {
			// The difference between Pose UP and Swing UP is what makes up our twist since they both
			// share the same forward access. The issue is that we do not know if that twist is in the Negative direction
			// or positive. So by computing the Swing Left Direction, we can use the Dot Product to determine
			// if swing UP is Over 90 Degrees, if so then its a positive twist else its negative.
			Vector3 swing_lft = swing_up.Cross(pose_fwd);
			if (swing_lft.Dot(pose_up) >= 0) twist = -twist;
		}


		// Save all the info we need to our IK Pose.
		ik_pose.hip.bind_height = bind_pose_w.mTrans.mValue.y; // The Bind Pose Height of the Hip, Helps with scaling.
		ik_pose.hip.movement = (curr_pose_w.mTrans.mValue - bind_pose_w.mTrans.mValue); // How much movement did the hip do between Bind and Animated.
		ik_pose.hip.dir = pose_fwd; // Pose Forward is the direction we want the Hip to Point to.
		ik_pose.hip.twist = twist; // How Much Twisting to Apply after pointing in the correct direction.

		//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[b_idx].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), pose_fwd, 3.0f, Vector4(Colors::Blue));
		//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[b_idx].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), pose_up, 8.0f, Vector4(Colors::Red));


	}

	void IKCompute::Limb(const IKRig& ik_rig, const IKChain& ik_chain, LimbIKData& ik_limb)
	{
		// The IK Solvers I put together takes transforms as input, not rotations.
		// The first thing we need is the WS Transform of the start of the chain
		// plus the parent's WS Transform. When are are building a full body IK
		// We need to do things in a certain order to build things correctly.
		// So before we can do legs, we need the hip/root to be moved to where it needs to go
		// The issue is that when people walk, you are actually falling forward, you catch
		// yourself when your front foot touches the floor, in the process you lift yourself 
		// up a bit. During a whole walk, or run cycle, a person's hip is always moving up and down
		// Because of that, the distance from the Hip to the floor is constantly changing
		// which is important if we want to have the legs stretch correctly since each IK leg
		// length scale is based on the hip being at a certain height at the time.
		//const Transform& joint_start_l = ik_rig.pose[ik_chain.GetFirstJoint()];
		//const Transform& joint_end_l = ik_rig.pose[ik_chain.end_idx];

		const Transform& joint_start_w = ik_rig.pose_world[ik_chain.GetFirstJoint()];
		const Transform& joint_mid_w = ik_rig.pose_world[ik_chain.joints[1]];
		const Transform& joint_end_w = ik_rig.pose_world[ik_chain.end_idx];
		float ab = (joint_end_w.mTrans.mValue - joint_mid_w.mTrans.mValue).Length() + (joint_start_w.mTrans.mValue - joint_mid_w.mTrans.mValue).Length();
		Vector3 ab_dir = joint_end_w.mTrans.mValue - joint_start_w.mTrans.mValue;
		float ab_len = ab_dir.Length();
		float ab_t_len = (ik_rig.tpose_world[ik_chain.end_idx].mTrans.mValue - ik_rig.tpose_world[ik_chain.GetFirstJoint()].mTrans.mValue).Length();
		// Compute the final IK Information needed for the Limb
		ik_limb.length_scale = ab_len / ik_chain.total_length;
		ik_limb.dir = ab_dir.Normalized();

		// We use the first bone of the chain plus the Pre computed ALT UP to easily get the direction of the joint
		Vector3 j_dir = Vector3::Transform(ik_chain.alt_up, joint_start_w.mRot.mValue);
		Vector3 lft_dir = j_dir.Cross(ab_dir);
		ik_limb.joint_dir = ab_dir.Cross(lft_dir).Normalized();

		//ik_limb.mid_axis = Vector3::Transform(-ik_chain.alt_up.Cross(ik_chain.alt_fwd), joint_start_w.mRot.mValue);
		//ik_limb.mid_axis = Vector3::Transform(-ik_limb.mid_axis, joint_mid_w.mRot.mValue.Inversed());
		ik_limb.mid_axis = Vector3::Transform(-lft_dir.Normalized(), joint_mid_w.mRot.mValue.Inversed());
		ik_limb.mid_axis = Vector3::Transform(ik_limb.mid_axis, (*ik_rig.tpose)[ik_chain.joints[1]].mRot.mValue);
		//ik_limb.mid_axis = -lft_dir.Normalized();

		ik_limb.pole = Vector3::Transform(ik_chain.alt_fwd, joint_mid_w.mRot.mValue); ;

		//ik_limb.pole = (joint_end_w.mTrans.mValue - joint_mid_w.mTrans.mValue).Normalized();
		//ik_limb.pole = j_dir.Normalized();
		ik_limb.debug_mid_axis = -lft_dir.Normalized();
		// Debug
		//ik_rig.skeleton->graphic_debug->DrawLine(joint_mid_w.mTrans.mValue*0.05f+ Vector3(-5.0f, 0.0f, 0.0f), -lft_dir.Normalized(), 10.0f, Vector4(Colors::Red));
	}

	void IKCompute::LookTwist(const IKRig& ik_rig, const IKPoint& ik_point, LookTwistIKData& ik_lt) {
		const Transform& curr_pose_w = ik_rig.pose_world[ik_point.idx];
		const Transform& bind_pose_w = ik_rig.tpose_world[ik_point.idx];

		// First compute the Quaternion Invert Directions based on the Defined
		// Directions that was passed into the function. Most often, your look
		// direction is FORWARD and the Direction used to determine twist is UP.
		// But there are times we need the directions to be different depending
		// on how we view the bone in certain situations.

		Vector3 curr_look_dir_w = Vector3::Transform(ik_point.alt_fwd, curr_pose_w.mRot.mValue);
		Vector3 curr_twist_dir_w = Vector3::Transform(ik_point.alt_up, curr_pose_w.mRot.mValue);

		ik_lt.look_dir = std::move(curr_look_dir_w);
		ik_lt.twist_dir = std::move(curr_twist_dir_w);

		//ik_rig.skeleton->graphic_debug->DrawLine(curr_pose_w.mTrans.mValue * 0.05f + Vector3(-5.0f, 0.0f, 0.0f), ik_lt.look_dir, 10.0f, Vector4(Colors::Yellow));
	}

	void IKCompute::Spine(const IKRig& ik_rig, const IKChain& ik_chain, std::vector<LookTwistIKData>& ik_lts)
	{
		std::vector<int> idxs = { ik_chain.GetFirstJoint(), ik_chain.GetLastJoint() };
		const Vector3 look_dir = Vector3::Transform(ik_chain.alt_fwd, ik_rig.tpose_world[ik_chain.GetFirstJoint()].mRot.mValue);
		const Vector3 twist_dir = Vector3::Transform(ik_chain.alt_up, ik_rig.tpose_world[ik_chain.GetFirstJoint()].mRot.mValue);

		ik_lts.clear();
		for (auto idx : idxs)
		{
			const Transform& bind_pose_w = ik_rig.tpose_world[idx];
			const Transform& curr_pose_w = ik_rig.pose_world[idx];

			Quaternion q_inv = bind_pose_w.mRot.mValue.Inversed();
			Vector3 curr_look_dir = Vector3::Transform(look_dir, q_inv); // In local space 
			Vector3 curr_twist_dir = Vector3::Transform(twist_dir, q_inv);

			curr_look_dir = Vector3::Transform(curr_look_dir, curr_pose_w.mRot.mValue); // In model space
			curr_twist_dir = Vector3::Transform(curr_twist_dir, curr_pose_w.mRot.mValue);

			// Debug
			//ik_rig.skeleton->graphic_debug->DrawLine(curr_pose_w.mTrans.mValue * 0.05f + Vector3(-5.0f, 0.0f, 0.0f), curr_twist_dir, 3.0f, Vector4(Colors::Red));

			LookTwistIKData ik_lt;
			ik_lt.look_dir = curr_look_dir;
			ik_lt.twist_dir = curr_twist_dir;
			ik_lts.push_back(std::move(ik_lt));
		}
	}
}