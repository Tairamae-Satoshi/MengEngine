#pragma once
#include "../Animation/BlendingJob.h"
#include "../Animation/LegController.h"
#include "../Animation/IKAimJob.h"
#include "../Animation/IKTwoBoneJob.h"
#include "RootMotion.h"
#include "CharacterController.h"

namespace Animation
{
	class Character
	{
	public:
		Character();

		Transform transform_;

		// TOFIX: Find why the model flips
		Matrix scale = XMMatrixScaling(0.05f, 0.05f, -0.05f) /** XMMatrixRotationY(MathHelper::Pi)*/;

		Skeleton skeleton_;

		std::vector<Transform> locals_;

		std::vector<Matrix> models_;

		RootMotion root_motion_;

		CharacterController character_controller_;

		// Root Motion
		Vector2 target_velocity_;

		bool transition_;

		LegController leg_controller_;

		bool Initialize();

		// Update Final models_ transform
		void UpdateFinalModelTransform();

		// Foot two bone ik
		float foot_ik_weight_ = 1.0f;

		float foot_ik_soften_ = 1.0f;

		void UpdateFootIK(Vector3 target);

		// Head aim ik
		int joints_chain_[4] = { -1, -1, -1, -1 };

		int chain_length_ = 4;

		// Weight given to every joint of the chain. If any joint has a weight of 1,
		// no other following joint will contribute (as the target will be reached).
		float joint_weight_ = 0.5f;

		// Overall weight given to the IK on the full chain. This allows blending in
		// and out of IK.
		float chain_weight_ = 1.0f;

		Vector3 eyes_offset_ = Vector3(0.0f, 0.0f, 0.0f);

		const char* k_joint_names_head[4] = { "Head", "Neck", "Spine2", "Spine1" };

		void UpdateBlendingMotion(BlendingJob& _blending_job);

		void UpdateHeadAimAtIK(Vector3 target);

		void UpdateRootMotion(bool transition);

		void UpdateController(float dt);
	};
}