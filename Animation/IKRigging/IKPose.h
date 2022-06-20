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

		Vector3 mid_axis; // The normal of plane which is constructed by two bone

		Vector3 pole;
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
		void ApplyRig(IKRig& rig)
		{
			rig.pose = *rig.tpose;
			//this->ApplyHip(rig);
			rig.UpdateWorld();

			//this->ApplyLimb(rig, rig.chains["leg_l"], leg_l);
			//this->ApplyLimb(rig, rig.chains["leg_r"], leg_r);
			this->ApplyLimb(rig, rig.chains["arm_l"], arm_l);
			this->ApplyLimb(rig, rig.chains["arm_r"], arm_r);
			rig.UpdateWorld();

			//this->ApplyLookTwist(rig, rig.points["foot_l"], foot_l);
			//this->ApplyLookTwist(rig, rig.points["foot_r"], foot_r);
			this->ApplyLookTwist(rig, rig.points["hand_l"], hand_l);
			this->ApplyLookTwist(rig, rig.points["hand_r"], hand_r);
			rig.UpdateWorld();
		}

		void ApplyHip(IKRig& rig)
		{
			// First step is we need to get access to the Rig's TPose and Pose Hip Bone.
			// The idea is to transform our Bind Pose into a New Pose based on IK Data
			unsigned int b_idx = rig.points["hip"].idx;
			Transform bind_pose_l = (*rig.tpose)[b_idx];
			Transform bind_pose_w = (rig.tpose_world)[b_idx];

			// Apply IK Swing & Twist ( HANDLE ROTATION )
			// When we compute the IK Hip, We used quaternion invert direction and defined that
			// the hip always points in the FORWARD Axis, so We can use that to quicky get Swing Rotation
			// By using that inverse direction trick, we are easily able to apply the same movement to both characters.

			// TODO: if the hip isn't the root bone
			Quaternion b_rot = bind_pose_l.mRot.mValue;
			Quaternion q = b_rot * Quaternion::CreateFromVectors(Vector3::Forward, hip.dir);

			// If There is a Twist Value, Apply that as a PreMultiplication.
			if (this->hip.twist != 0.0f){
				q = Quaternion::CreateFromAxisAngle(this->hip.dir, this->hip.twist) * q;
			}

			rig.pose[b_idx].mRot.mValue = q;

			// Translation
			float h_scl = bind_pose_w.mTrans.mValue.y / this->hip.bind_height;
			Vector3 pos = this->hip.movement * h_scl + bind_pose_w.mTrans.mValue;

			// MAYBE we want to keep the stride distance exact, we can reset the XZ positions
			// BUT we need to keep the Y Movement scaled, else our leg IK won't work well since
			// our source is taller then our targets, this will cause our target legs to always
			// straighten out.
			//pos.x = this->hip.movement.x;
			//pos.z = this->hip.movement.z;
			rig.pose[b_idx].mTrans.mValue = pos;
		}

		void ApplyLimb(IKRig& ik_rig, const IKChain& ik_chain, const LimbIKData& ik_limb)
		{
			float len = ik_chain.total_length * ik_limb.length_scale;

			Vector3 target = ik_rig.pose_world[ik_chain.GetFirstJoint()].mTrans.mValue + ik_limb.dir * len;

			//Vector3 pole_vector_ms = models[ik_chain.joints[1]].Up();//FIXME
			IKTwoBoneJob ik_job;
			ik_job.target = target;
			ik_job.pole_vector = Vector3::Forward;
			// Mid axis (knee) is constant (usualy), and arbitratry defined by
			// skeleton/rig setup.
			//ik_job.mid_axis = Vector3::UnitX;
			ik_job.weight = 1.0f;
			ik_job.soften = 1.0f;
			ik_job.twist_angle = 0.0f;
			ik_job.mid_initial_rot = ik_rig.pose[ik_chain.joints[1]].mRot.mValue;
			ik_job.start_joint =  Matrix::CreateAffineTransformation(
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
			ik_job.mid_axis = Vector3::TransformVector(
				ik_limb.mid_axis,
				ik_job.mid_joint.Invert());
			ik_job.pole_vector = ik_limb.pole;

			ik_job.Run();

			// Apply IK quaternions to their respective local-space transforms.
			// Model-space transformations needs to be updated after a call to this
			// function.
			// Note the order of multiplication 
			//ik_rig.pose[ik_chain.joints[0]].mRot.mValue = ik_job.start_joint_correction * ik_rig.pose[ik_chain.joints[0]].mRot.mValue;
			ik_rig.pose[ik_chain.joints[1]].mRot.mValue = ik_job.mid_joint_correction * ik_rig.pose[ik_chain.joints[1]].mRot.mValue;
			//ik_rig.pose[ik_chain.joints[0]].mRot.mValue = Quaternion::CreateFromAxisAngle(ik_job.mid_axis, /*bent_side_flip **/ -0.5f*MathHelper::Pi) * ik_rig.pose[ik_chain.joints[1]].mRot.mValue;

		}

		void ApplyLookTwist(IKRig& ik_rig, const IKPoint& ik_point, LookTwistIKData& ik_lt) {
			// First we need to get the WS Rotation of the parent to the Foot
			// Then Add the Foot's LS Bind rotation. The idea is to see where
			// the foot will currently be if it has yet to have any rotation
			// applied to it.
			const Transform& curr_pose_w = ik_rig.pose_world[ik_point.idx];
			const Transform& bind_pose_w = ik_rig.tpose_world[ik_point.idx];
			const Transform& curr_pose_l = ik_rig.pose[ik_point.idx];
			const Transform& bind_pose_l = (*ik_rig.tpose)[ik_point.idx];

			Quaternion p_rot = bind_pose_l.mRot.mValue *ik_rig.pose_world[ik_rig.skeleton->GetJointParentIndex(ik_point.idx)].mRot.mValue;
			Quaternion c_rot = curr_pose_w.mRot.mValue;

			// After the HIP was moved and The Limb IK is complete, This is where 
			// the ALT Look Direction currently points to.
			Vector3 curr_look_dir = Vector3::Transform(ik_point.alt_fwd, curr_pose_w.mRot.mValue);

			// Now we start building out final rotation that we
			// want to apply to the bone to get it pointing at the
			// right direction and twisted to match the original animation.
			Quaternion swing = Quaternion::CreateFromVectors(curr_look_dir, ik_lt.look_dir);
			Vector3 v = Vector3::Transform(curr_look_dir, swing);

			Quaternion rot = swing * c_rot;

			// Now we need to know where the Twist Direction points to after 
			// swing rotation has been applied. Then use it to compute our twist rotation.
			Vector3 curr_twist_dir = Vector3::Transform(ik_point.alt_up, rot);
			Quaternion twist = Quaternion::CreateFromVectors(curr_twist_dir, ik_lt.twist_dir);

			ik_rig.pose[ik_point.idx].mRot.mValue = /*twist **/ swing * curr_pose_l.mRot.mValue;
		}

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
	};
}