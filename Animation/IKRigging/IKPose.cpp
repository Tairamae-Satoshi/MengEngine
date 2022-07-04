#include "IKPose.h"

namespace Animation
{
	void IKPose::ApplyRig(IKRig& rig, float dt)
	{
		rig.pose = *rig.tpose;
		rig.UpdateWorld();

		this->ApplyHip(rig);
		rig.UpdateWorld();

		if (rig.chains.find("spine") != rig.chains.end()) {
			this->ApplySpine(rig, rig.chains["spine"], spine);
			rig.UpdateWorld();
		}

		if (rig.chains.find("tail") != rig.chains.end()) this->ApplySpringBone(rig, rig.chains["tail"], rig.spring_bones["tail"], dt);

		this->ApplyLimb(rig, rig.chains["leg_l"], leg_l);
		this->ApplyLimb(rig, rig.chains["leg_r"], leg_r);
		if (rig.chains.find("arm_l") != rig.chains.end()) this->ApplyLimb(rig, rig.chains["arm_l"], arm_l);
		if (rig.chains.find("arm_r") != rig.chains.end()) this->ApplyLimb(rig, rig.chains["arm_r"], arm_r);
		rig.UpdateWorld();

		this->ApplyLookTwist(rig, rig.points["head"], head);
		this->ApplyLookTwist(rig, rig.points["foot_l"], foot_l);
		this->ApplyLookTwist(rig, rig.points["foot_r"], foot_r);
		if (rig.points.find("arm_l") != rig.points.end()) this->ApplyLookTwist(rig, rig.points["hand_l"], hand_l);
		if (rig.points.find("arm_l") != rig.points.end()) this->ApplyLookTwist(rig, rig.points["hand_r"], hand_r);
		rig.UpdateWorld();
	}

	void IKPose::ApplyHip(IKRig& ik_rig)
	{
		// First step is we need to get access to the Rig's TPose and Pose Hip Bone.
		// The idea is to transform our Bind Pose into a New Pose based on IK Data
		unsigned int b_idx = ik_rig.points["hip"].idx;
		const Transform& bind_pose_l = (*ik_rig.tpose)[b_idx];
		const Transform& bind_pose_w = (ik_rig.tpose_world)[b_idx];

		// Apply IK Swing & Twist ( HANDLE ROTATION )
		// When we compute the IK Hip, We used quaternion invert direction and defined that
		// the hip always points in the FORWARD Axis, so We can use that to quicky get Swing Rotation
		// By using that inverse direction trick, we are easily able to apply the same movement to both characters.

		// TODO: if the hip isn't the root bone
		Quaternion b_rot = bind_pose_l.mRot.mValue;
		Quaternion q = b_rot * Quaternion::CreateFromVectors(Vector3::Forward, hip.dir);

		// If There is a Twist Value, Apply that as a PreMultiplication.
		if (this->hip.twist != 0.0f) {
			q = q * Quaternion::CreateFromAxisAngle(this->hip.dir, this->hip.twist);
		}

		ik_rig.pose[b_idx].mRot.mValue = q;

		// Translation
		float h_scl = bind_pose_w.mTrans.mValue.y / this->hip.bind_height;
		Vector3 pos = this->hip.movement * h_scl + bind_pose_w.mTrans.mValue;

		// MAYBE we want to keep the stride distance exact, we can reset the XZ positions
		// BUT we need to keep the Y Movement scaled, else our leg IK won't work well since
		// our source is taller then our targets, this will cause our target legs to always
		// straighten out.
		//pos.x = this->hip.movement.x;
		//pos.z = this->hip.movement.z;
		ik_rig.pose[b_idx].mTrans.mValue = pos;

		// Debug
		//Quaternion q_inv = bind_pose_w.mRot.mValue.Inversed();
		//Vector3 alt_fwd = Vector3::Transform(Vector3::Forward, q_inv);
		//Vector3 alt_up = Vector3::Transform(Vector3::Up, q_inv);

		//Vector3 pose_fwd = Vector3::Transform(alt_fwd, q);
		//Vector3 pose_up = Vector3::Transform(alt_up, q);

		//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[b_idx].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), pose_fwd, 3.0f, Vector4(Colors::Yellow));
		//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[b_idx].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f),  pose_up, 18.0f, Vector4(Colors::Yellow));

	}

	void IKPose::ApplyLimb(IKRig& ik_rig, const IKChain& ik_chain, const LimbIKData& ik_limb)
	{
		float len = ik_chain.total_length * ik_limb.length_scale;

		Vector3 target = ik_rig.pose_world[ik_chain.GetFirstJoint()].mTrans.mValue + ik_limb.dir * len;

		ApplyGrounding(ik_rig.pose_world[ik_chain.GetFirstJoint()].mTrans.mValue, target, 0);

		if (ik_chain.ik_solver == "TwoBone")
		{
			IKTwoBoneJob ik_job;
			ik_job.target = target;
			// Mid axis (knee) is constant (usualy), and arbitratry defined by
			// skeleton/rig setup.
			ik_job.weight = 1.0f;
			ik_job.soften = 1.0f;
			ik_job.twist_angle = 0.0f;
			//ik_job.mid_initial_rot = ik_rig.pose[ik_chain.joints[1]].mRot.mValue;
			ik_job.start_joint = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.joints[0]].mScale.mValue,
				ik_rig.pose_world[ik_chain.joints[0]].mTrans.mValue,
				ik_rig.pose_world[ik_chain.joints[0]].mRot.mValue);
			ik_job.mid_joint = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.joints[1]].mScale.mValue,
				ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue,
				ik_rig.pose_world[ik_chain.joints[1]].mRot.mValue);
			ik_job.end_joint = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.end_idx].mScale.mValue,
				ik_rig.pose_world[ik_chain.end_idx].mTrans.mValue,
				ik_rig.pose_world[ik_chain.end_idx].mRot.mValue);
			ik_job.mid_axis = Vector3::Transform(ik_limb.mid_axis, ik_rig.tpose_world[ik_chain.joints[1]].mRot.mValue.Inversed());
			ik_job.pole_vector = ik_limb.pole;
			ik_job.Run();

			// Apply IK quaternions to their respective local-space transforms.
			// Model-space transformations needs to be updated after a call to this
			// function.
			// FIXME: Note the order of multiplication. q_local = q_model * q_s * q_w * q_p^-1 ?

			//Quaternion p_rot = ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_chain.joints[0])].mRot.mValue;
			//Quaternion q_correction = ik_rig.pose_world[ik_chain.joints[0]].mRot.mValue * ik_job.start_joint_correction;
			//ik_rig.pose[ik_chain.joints[0]].mRot.mValue = ik_rig.pose[ik_chain.joints[0]].mRot.mValue * ik_job.start_joint_correction;
			//ik_rig.pose[ik_chain.joints[0]].mRot.mValue = q_correction * p_rot.Inversed();
			ik_rig.pose[ik_chain.joints[0]].mRot.mValue = ik_job.start_joint_correction * ik_rig.pose[ik_chain.joints[0]].mRot.mValue;

			//p_rot = ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_chain.joints[1])].mRot.mValue;
			//q_correction = ik_rig.pose_world[ik_chain.joints[1]].mRot.mValue * ik_job.mid_joint_correction;
			//ik_rig.pose[ik_chain.joints[1]].mRot.mValue = ik_rig.pose[ik_chain.joints[1]].mRot.mValue * ik_job.mid_joint_correction;
			//ik_rig.pose[ik_chain.joints[1]].mRot.mValue = q_correction * p_rot.Inversed(); // This one should be the correct one, but didn't work. Why?
			ik_rig.pose[ik_chain.joints[1]].mRot.mValue = ik_job.mid_joint_correction * ik_rig.pose[ik_chain.joints[1]].mRot.mValue;

			// Debug
			// FIXUP: the leg's mid axis do not align with the ref.
			//ik_rig.UpdateWorld();
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), Vector3::Transform(ik_limb.mid_axis, ik_rig.pose_world[ik_chain.joints[1]].mRot.mValue), 3.0f, Vector4(Colors::Red));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_limb.debug_mid_axis, 3.0f, Vector4(Colors::Blue));

			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_job.pole_vector, 5.0f, Vector4(Colors::Red));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), (ik_rig.pose_world[ik_chain.end_idx].mTrans.mValue - ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue).Normalized(), 5.0f, Vector4(Colors::Blue));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[0]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), target * 0.05 + Vector3(5.0f, 0.0f, 0.0f), Vector4(Colors::Red));

			/*const Transform& joint_start_w = ik_rig.pose_world[ik_chain.GetFirstJoint()];
			const Transform& joint_mid_w = ik_rig.pose_world[ik_chain.joints[1]];
			const Transform& joint_end_w = ik_rig.pose_world[ik_chain.end_idx];
			float ab = (joint_end_w.mTrans.mValue - joint_mid_w.mTrans.mValue).Length() + (joint_start_w.mTrans.mValue - joint_mid_w.mTrans.mValue).Length();
			Vector3 ab_dir = joint_end_w.mTrans.mValue - joint_start_w.mTrans.mValue;
			float ab_len = ab_dir.Length();

			Vector3 j_dir = Vector3::Transform(ik_chain.alt_up, joint_start_w.mRot.mValue);
			j_dir = Vector3::Transform(ik_chain.alt_up, ik_rig.tpose_world[ik_chain.joints[0]].mRot.mValue);
			Vector3 lft_dir = j_dir.Cross(ab_dir);

			ik_job.mid_axis = -lft_dir.Normalized();*/

			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), j_dir, 3.0f, Vector4(Colors::Green));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_job.mid_axis, 3.0f, Vector4(Colors::Red));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_limb.debug_mid_axis, 3.0f, Vector4(Colors::Blue));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[0]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ab_dir.Normalized(), ab_len * 0.05f, Vector4(Colors::Blue));

			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_job.pole_vector, 5.0f, Vector4(Colors::Red));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), (ik_rig.pose_world[ik_chain.end_idx].mTrans.mValue - ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue).Normalized(), 5.0f, Vector4(Colors::Blue));
			//ik_rig.skeleton->graphic_debug->DrawLine(ik_rig.pose_world[ik_chain.joints[0]].mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), target * 0.05 + Vector3(5.0f, 0.0f, 0.0f), Vector4(Colors::Red));
		}
		else if(ik_chain.ik_solver == "ThreeBone")
		{
			IKThreeBoneJob ik_job;
			ik_job.target = target;
			ik_job.joints[0] = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.joints[0]].mScale.mValue,
				ik_rig.pose_world[ik_chain.joints[0]].mTrans.mValue,
				ik_rig.pose_world[ik_chain.joints[0]].mRot.mValue);
			ik_job.joints[1] = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.joints[1]].mScale.mValue,
				ik_rig.pose_world[ik_chain.joints[1]].mTrans.mValue,
				ik_rig.pose_world[ik_chain.joints[1]].mRot.mValue);
			ik_job.joints[2] = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.joints[2]].mScale.mValue,
				ik_rig.pose_world[ik_chain.joints[2]].mTrans.mValue,
				ik_rig.pose_world[ik_chain.joints[2]].mRot.mValue);
			ik_job.joints[3] = Matrix::CreateAffineTransformation(
				ik_rig.pose_world[ik_chain.end_idx].mScale.mValue,
				ik_rig.pose_world[ik_chain.end_idx].mTrans.mValue,
				ik_rig.pose_world[ik_chain.end_idx].mRot.mValue);
			ik_job.mid_axis = Vector3::Transform(ik_limb.mid_axis, ik_rig.tpose_world[ik_chain.joints[1]].mRot.mValue.Inversed());
			ik_job.mid_axis = Vector3::TransformVector(ik_limb.mid_axis, ik_job.joints[1]);
			ik_job.pole_vector = ik_limb.pole;
			ik_job.Run();

			ik_rig.pose[ik_chain.joints[0]].mRot.mValue = ik_job.joints_correction[0] * ik_rig.pose[ik_chain.joints[0]].mRot.mValue;
			ik_rig.pose[ik_chain.joints[1]].mRot.mValue = ik_job.joints_correction[1] * ik_rig.pose[ik_chain.joints[1]].mRot.mValue;
			ik_rig.pose[ik_chain.joints[2]].mRot.mValue = ik_job.joints_correction[2] * ik_rig.pose[ik_chain.joints[2]].mRot.mValue;

		}		
	}

	void IKPose::ApplyGrounding(Vector3& start, Vector3& target, float y_lmt) {
		if (target.y >= 0) return;

		const Vector3& a = start;
		const Vector3& b = target;
		float s = (y_lmt - a.y) / (b.y - a.y);

		target = Vector3((b.x - a.x) * s + a.x, y_lmt, (b.z - a.z) * s + a.z);
	}


	void IKPose::ApplySpine(IKRig& ik_rig, const IKChain& ik_chain, const std::vector<LookTwistIKData>& ik_lts) {
		// For the spline, we have the start and end IK directions. Since spines can have various
		// amount of bones, the simplest solution is to lerp from start to finish. The first
		// spine bone is important to control offsets from the hips, and the final one usually
		// controls the chest which dictates where the arms and head are going to be located.
		// Anything between is how the spine would kind of react.

		// Since we are building up the Skeleton, We look at the pose object to know where the Hips
		// currently exist in World Space.
		Transform p_tran_w = ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_chain.GetFirstJoint())];

		for (int i = 0; i < ik_chain.joints.size(); i++)
		{
			int idx = ik_chain.joints[i];
			const Transform& bind_pose_l = (*ik_rig.tpose)[idx];

			float t = (float)i / float(ik_chain.joints.size() - 1);

			Vector3 ik_look = Vector3::Lerp(ik_lts[0].look_dir, ik_lts[1].look_dir, t);
			Vector3 ik_twist = Vector3::Lerp(ik_lts[0].twist_dir, ik_lts[1].twist_dir, t);

			Vector3 alt_look = ik_chain.alt_fwd;
			Vector3 alt_twist = ik_chain.alt_up;

			Transform c_tran_w = p_tran_w;
			c_tran_w = c_tran_w.Add(bind_pose_l);

			Vector3 cur_look = Vector3::Transform(alt_look, c_tran_w.mRot.mValue);

			Quaternion swing = Quaternion::CreateFromVectors(cur_look, ik_look);
			Quaternion rot = c_tran_w.mRot.mValue * swing;

			Vector3 cur_twist = Vector3::Transform(alt_twist, rot);
			Quaternion twist = Quaternion::CreateFromVectors(cur_twist, ik_twist);
			rot = rot * twist;

			rot = rot * p_tran_w.mRot.mValue.Inversed();

			ik_rig.pose[idx].mRot.mValue = rot;

			if (t != 1.0f) {
				p_tran_w = p_tran_w.Add(ik_rig.pose[idx]);
			}

			//ik_rig.UpdateWorld();
			//cur_twist = Vector3::Transform(ik_chain.alt_up, ik_rig.pose_world[idx].mRot.mValue);

			//ik_rig.skeleton->graphic_debug->DrawLine(c_tran_w.mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), cur_twist, 8.0f, Vector4(Colors::Red));
			//ik_rig.skeleton->graphic_debug->DrawLine(c_tran_w.mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_twist, 10.0f, Vector4(Colors::Blue));
			//
		}
	}

	void IKPose::ApplyLookTwist(IKRig& ik_rig, const IKPoint& ik_point, const LookTwistIKData& ik_lt) {
		// First we need to get the WS Rotation of the parent to the Foot
		// Then Add the Foot's LS Bind rotation. The idea is to see where
		// the foot will currently be if it has yet to have any rotation
		// applied to it.\

		const Transform curr_pose_w = ik_rig.pose_world[ik_point.idx];
		const Transform bind_pose_w = ik_rig.tpose_world[ik_point.idx];
		const Transform curr_pose_l = ik_rig.pose[ik_point.idx];
		const Transform bind_pose_l = (*ik_rig.tpose)[ik_point.idx];

		Quaternion p_rot = ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_point.idx)].mRot.mValue;
		Quaternion c_rot = curr_pose_w.mRot.mValue;
		Quaternion qq = p_rot * curr_pose_l.mRot.mValue;

		// After the HIP was moved and The Limb IK is complete, This is where 
		// the ALT Look Direction currently points to.
		Vector3 curr_look_dir = Vector3::Transform(ik_point.alt_fwd, curr_pose_w.mRot.mValue);
		Vector3 curr_twist_dir = Vector3::Transform(ik_point.alt_up, curr_pose_w.mRot.mValue);

		// Now we start building out final rotation that we
		// want to apply to the bone to get it pointing at the
		// right direction and twisted to match the original animation.
		const Quaternion swing = Quaternion::CreateFromVectors(curr_look_dir, ik_lt.look_dir);

		// Now we need to know where the Twist Direction points to after 
		// swing rotation has been applied. Then use it to compute our twist rotation.
		curr_twist_dir = Vector3::Transform(curr_twist_dir, swing);
		const Quaternion twist = Quaternion::CreateFromVectors(curr_twist_dir, ik_lt.twist_dir);

		Quaternion q_correction = curr_pose_w.mRot.mValue * swing * twist * p_rot.Inversed();
		ik_rig.pose[ik_point.idx].mRot.mValue = q_correction;

		// Debug
		//ik_rig.UpdateWorld();
		//const Transform c_l = ik_rig.pose[ik_point.idx];
		//const Transform c_w = ik_rig.pose_world[ik_point.idx];

		//curr_look_dir = Vector3::Transform(ik_point.alt_fwd, c_w.mRot.mValue);
		//curr_twist_dir = Vector3::Transform(ik_point.alt_up, c_w.mRot.mValue);


		//ik_rig.skeleton->graphic_debug->DrawLine(c_w.mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), curr_twist_dir, 3.0f, Vector4(Colors::Pink));
		//ik_rig.skeleton->graphic_debug->DrawLine(c_w.mTrans.mValue * 0.05f + Vector3(5.0f, 0.0f, 0.0f), ik_lt.twist_dir, 3.0f, Vector4(Colors::Blue));

	}


	void IKPose::ApplySpringBone(IKRig& ik_rig, const IKChain& ik_chain, SpringBoneJob& spring_bone, float dt)
	{
		char out[50];
		sprintf(out, "%f", dt);
		LOG(out);

		if (!spring_bone.is_pos_reset)
		{
			spring_bone.Reset(ik_rig.pose_world);
			spring_bone.is_pos_reset = true;
		}

		Transform p_tran_w = ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_chain.GetFirstJoint())];
		spring_bone.parent_w_t = p_tran_w;

		std::vector<Transform> joints_tran_l;
		for (auto idx : ik_chain.joints){
			joints_tran_l.push_back(ik_rig.pose[idx]);
		}
		spring_bone.joints_l_t = joints_tran_l;
		spring_bone.Run(dt);
		//for (int i = 0; i < spring_bone.joints.size(); i++)
		//{
		//	spring_bone.joint_correction_l_t[i] = spring_bone.joints_l_t[i];
		//}

		//for (int i = 0; i < spring_bone.joints.size() - 1; i++)
		//{
		//	Transform joint_w_t = spring_bone.parent_w_t.Add(spring_bone.joints_l_t[i]);

		//	float bone_len = spring_bone.joints_l_t[i + 1].mTrans.mValue.Length();
		//	Vector3 tail = Vector3(0.0f, 0.0f, bone_len);
		//	tail = Vector3::Transform(tail, Transform::ToMatrix(joint_w_t));

		//	//Vector3 pos = spring_bone.joints[i].spring.position;
		//	//Vector3 pos_w = Vector3::Transform(Vector3(0.0f, 0.0f, -bone_len), Transform::ToMatrix(ik_rig.pose_world[spring_bone.joints[i].idx]));
		//	spring_bone.joints[i].spring.Update(dt, tail);
		//	Vector3 spring_pos = spring_bone.joints[i].spring.position;

		//	Vector3 resting_dir = (tail - joint_w_t.mTrans.mValue).Normalized(); // Dir to resting position
		//	Vector3 spring_dir = (spring_pos - joint_w_t.mTrans.mValue).Normalized(); // Dir to spring position
		//	//ik_rig.skeleton->graphic_debug->DrawLine(joint_w_t.mTrans.mValue * 0.02f + Vector3(5.0f, 1.0f, 0.0f), resting_dir, bone_len * 0.02f, Vector4(DirectX::Colors::Red));
		//	//ik_rig.skeleton->graphic_debug->DrawLine(joint_w_t.mTrans.mValue * 0.02f + Vector3(5.0f, 0.0f, 0.0f), spring_dir, bone_len * 0.02f, Vector4(DirectX::Colors::Blue));

		//	Quaternion rot = IsVector3Equal(resting_dir, spring_dir) ? Quaternion::Identity : Quaternion::CreateFromVectors(resting_dir, spring_dir);
		//	Quaternion temp = rot;
		//	rot = joint_w_t.mRot.mValue * rot * spring_bone.parent_w_t.mRot.mValue.Inversed();

		//	spring_bone.joint_correction_l_t[i].mRot.mValue = rot;
		//	float w = rot.w;
		//	spring_bone.parent_w_t = spring_bone.parent_w_t.Add(spring_bone.joint_correction_l_t[i]);
		//	//LOG("1");
		//}

		for (int i = 0; i < ik_chain.joints.size(); i++) {
			int idx = ik_chain.joints[i];
			ik_rig.pose[idx] = spring_bone.joint_correction_l_t[i];
			//ik_rig.pose[idx].mRot.mValue = Quaternion::CreateFromAxisAngle(Vector3(0, 1, 0), MathHelper::Pi * 0.05f);
		}

	}

}