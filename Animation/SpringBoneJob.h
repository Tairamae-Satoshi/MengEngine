#pragma once

#include"AnimationDatabase.h"
#include "Spring.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class SemiImplicitEuler
	{
	public:
		SemiImplicitEuler(float osc = 1.0f, float damp = 1.0f, float damp_time = 0.0f) 
		{
			position = Vector3::Zero;
			velocity = Vector3::Zero;
			this->osc_ps = MathHelper::Pi * 2 * osc;
			if (damp_time > 0) {
				this->damping = damp / (-this->osc_ps * damp_time);
			}
			else{
				this->damping = damp;
			}
		}

		void Update(float dt, const Vector3& target_pos)
		{
			simple_spring_damper_implicit(position, velocity, target_pos, halftime, dt);
		}

		Vector3 position;

		Vector3 velocity;

		float osc_ps;

		float halftime;

		float damping;
	};

	class SpringBoneJob
	{
	public:
		struct SpringJoint
		{
			int idx;
			SemiImplicitEuler spring;
		};

		SpringBoneJob() {}

		void Init(std::vector<int> chain_joints, 
			float osc = 0.0f, float damp = 0.9f, float osc_inc = 0.0f, float damp_inc = 0.0f)
		{
			//this->skeleton = skeleton;
			int i = 0;
			for (i = 0; i < chain_joints.size(); i++)
			{
				SpringJoint spring_joint;
				spring_joint.idx = chain_joints[i];
				spring_joint.spring = SemiImplicitEuler(osc + osc_inc * i, damp + damp_inc * i);
				spring_joint.spring.halftime = MathHelper::Max(0.01f, i % 2 == 0 ? 0.1f - 0.01f * i: 100.0f); // Hack: to solve the problem of too many bone in tail..
				joints.push_back(spring_joint);
			}

			joint_correction_l_t.resize(chain_joints.size());

		}

		void Reset(const std::vector<Transform>& joints_w_t)
		{
			int i = 0;
			for (i = 0; i < joints.size(); i++)
			{
				if (i != joints.size() - 1) {
					float bone_len = (joints_w_t[joints[i + 1].idx].mTrans.mValue - joints_w_t[joints[i].idx].mTrans.mValue).Length();

					Vector3 tail = Vector3(0.0f, 0.0f, bone_len);
					joints[i].spring.position = Vector3::Transform(tail, Transform::ToMatrix(joints_w_t[joints[i].idx]));
					joints[i].spring.velocity = Vector3(0.0f, 0.0f, 0.0f);
				}
				else {
					joints[i].spring.position = Vector3(0.0f, 0.0f, 0.0f);
					joints[i].spring.velocity = Vector3(0.0f, 0.0f, 0.0f);
				}
			}

			joint_correction_l_t.resize(joints.size());

		}

		bool IsVector3Equal(const Vector3& a, const Vector3& b)
		{
			return (a - b).LengthSquared() < 1e-6 ? true : false;
		}

		void Run(float dt)
		{
			for (int i = 0; i < joints.size(); i++)
			{
				joint_correction_l_t[i] = joints_l_t[i];
			}

			for (int i = 0; i < joints.size() - 1; i++)
			{
				Transform joint_w_t = parent_w_t.Add(joints_l_t[i]);

				float bone_len = joints_l_t[i + 1].mTrans.mValue.Length();
				Vector3 tail = Vector3(0.0f, 0.0f, bone_len);
				tail = Vector3::Transform(tail, Transform::ToMatrix(joint_w_t));

				//Vector3 pos = spring_bone.joints[i].spring.position;
				//Vector3 pos_w = Vector3::Transform(Vector3(0.0f, 0.0f, -bone_len), Transform::ToMatrix(ik_rig.pose_world[spring_bone.joints[i].idx]));
				joints[i].spring.Update(dt, tail);
				Vector3 spring_pos = joints[i].spring.position;

				Vector3 resting_dir = (tail - joint_w_t.mTrans.mValue).Normalized(); // Dir to resting position
				Vector3 spring_dir = (spring_pos - joint_w_t.mTrans.mValue).Normalized(); // Dir to spring position
				//ik_rig.skeleton->graphic_debug->DrawLine(joint_w_t.mTrans.mValue * 0.02f + Vector3(5.0f, 1.0f, 0.0f), resting_dir, bone_len * 0.02f, Vector4(DirectX::Colors::Red));
				//ik_rig.skeleton->graphic_debug->DrawLine(joint_w_t.mTrans.mValue * 0.02f + Vector3(5.0f, 0.0f, 0.0f), spring_dir, bone_len * 0.02f, Vector4(DirectX::Colors::Blue));

				Quaternion rot = IsVector3Equal(resting_dir, spring_dir) ? Quaternion::Identity : Quaternion::CreateFromVectors(resting_dir, spring_dir);
				Quaternion temp = rot;
				rot = joint_w_t.mRot.mValue * rot * parent_w_t.mRot.mValue.Inversed();

				joint_correction_l_t[i].mRot.mValue = rot;
				float w = rot.w;
				parent_w_t = parent_w_t.Add(joint_correction_l_t[i]);
				//LOG("1");
			}
		}


		bool is_pos_reset = false;

		//const AnimationDatabase* skeleton;

		Transform parent_w_t; // Parent model/world transforms

		std::vector<Transform> joints_l_t; // joint local transforms

		std::vector<Transform> joint_correction_l_t;

		std::vector<SpringJoint> joints;

	};
}

