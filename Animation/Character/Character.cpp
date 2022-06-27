#include "Character.h"

namespace Animation
{
	Character::Character() :
		target_velocity(Vector2::Zero),
		transition(false)
	{}

	bool Character::Initialize()
	{
		// Initialize the character controller
		character_controller.Initialize(&db);

		// Initialize the leg controller
		leg_controller.skeleton = &db;
		leg_controller.Initialize();

		// Look for each joint in the head chain.
		int found = 0;
		for (int i = 0; i < db.JointCount() && found != 4; i++) {
			std::string joint_name = db.GetJointName(i);

			if (joint_name.find(k_joint_names_head[found]) != joint_name.npos) {
				joints_chain_[found] = i;
				++found;
				i = 0;
			}
		}

		return true;
	}

	void Character::UpdateFinalModelTransform(bool isBindpose)
	{
		if (isBindpose)
			locals = db.GetBindPose();

		LocalToModelJob ltm_job;
		ltm_job.skeleton = &db;
		ltm_job.input = locals;
		ltm_job.Run(true, true);

		models = ltm_job.output;
		//transform.mTrans.mValue = Vector3::Transform(locals[0].mTrans.mValue, scale);
	}

	void Character::UpdateFootIK(Vector3 target)
	{
		LocalToModelJob ltm_job;
		ltm_job.skeleton = &db;
		ltm_job.input = locals;
		ltm_job.Run(true, false);
		models = ltm_job.output;

		const LegInfo& leg = leg_controller.legs[0];
		// Target position and pole vectors must be in model space.
		Vector3 target_ms = Vector3::Transform(target, scale.Invert());
		Vector3 pole_vector_ms = models[leg.knee].Up();//FIXME
		IKTwoBoneJob ik_job;
		ik_job.target = target_ms;
		ik_job.pole_vector = pole_vector_ms;
		// Mid axis (knee) is constant (usualy), and arbitratry defined by
		// skeleton/rig setup.
		//ik_job.mid_axis = Vector3::UnitX;
		ik_job.weight = foot_ik_weight;
		ik_job.soften = foot_ik_soften;
		ik_job.twist_angle = 0.0f;
		//ik_job.mid_initial_rot = locals[leg.knee].mRot.mValue;
		ik_job.start_joint = models[leg.hip];
		ik_job.mid_joint = models[leg.knee];
		ik_job.end_joint = models[leg.ankle];
		ik_job.Run();

		// Apply IK quaternions to their respective local-space transforms.
		// Model-space transformations needs to be updated after a call to this
		// function.
		// Note the order of multiplication 
		locals[leg.hip].mRot.mValue = ik_job.start_joint_correction * locals[leg.hip].mRot.mValue;
		locals[leg.knee].mRot.mValue = ik_job.mid_joint_correction * locals[leg.knee].mRot.mValue;

	}

	void Character::UpdateRenderItem()
	{
		for (auto ri : ritems)
		{
			std::copy(
				std::begin(models),
				std::end(models),
				&ri->skinnedConstant.BoneTransforms[0]);

			XMMATRIX m = XMMatrixScaling(transform.mScale.mValue.x, transform.mScale.mValue.y, transform.mScale.mValue.z);
			m *= XMMatrixRotationQuaternion(transform.mRot.mValue);
			m *= XMMatrixTranslationFromVector(transform.mTrans.mValue);
			XMStoreFloat4x4(&ri->World, m);
		}
	}

	void Character::UpdateBlendingMotion(BlendingJob& _blending_job)
	{
		Vector3 v = quat_inv_mul_vec3(character_controller.simulation_rotation, character_controller.simulation_velocity);
		Vector2 velocity = Vector2(v.x, -v.z); // TODO

		std::vector<float> weights = _blending_job.interpolator->Interpolate(velocity, true);
		
		Vector2 new_target_velocity = Vector2(character_controller.target_direction.x, character_controller.target_direction.z);
		if (new_target_velocity !=target_velocity){
			transition = true;
			target_velocity = new_target_velocity;
		}
		else{
			transition = false;
		}

		for (int i = 0; i < _blending_job.layers.size(); ++i)
		{
			_blending_job.layers[i].weight = weights[i];
		}

		_blending_job.Run();
		locals = _blending_job.output;
	}

	void Character::UpdateHeadAimAtIK(Vector3 target)
	{
		LocalToModelJob ltm_job;
		ltm_job.skeleton = &db;
		ltm_job.input = locals;
		ltm_job.Run(true, false);
		models = ltm_job.output;

		IKAimJob ik_job;

		// Pole vector and target position are constant for the whole algorithm, in model-space
		ik_job.pole_vector = Vector3::UnitY;
		ik_job.target = Vector3::Transform(target, scale.Invert());

		// The same quaternion will be used each time the job is run.
		Quaternion correction;
		ik_job.joint_correction = &correction;

		// The algorithm iteratively updates from the first joint (closer to the
		// head) to the last (the further ancestor, closer to the pelvis). Joints
		// order is already validated. For the first joint, aim IK is applied with
		// the global forward and offset, so the forward vector aligns in direction
		// of the target. If a weight lower that 1 is provided to the first joint,
		// then it will not fully align to the target. In this case further joint
		// will need to be updated. For the remaining joints, forward vector and
		// offset position are computed in each joint local-space, before IK is
		// applied:
		// 1. Rotates forward and offset position based on the result of the
		// previous joint IK.
		// 2. Brings forward and offset back in joint local-space.
		// Aim is iteratively applied up to the last selected joint of the
		// hierarchy. A weight of 1 is given to the last joint so we can guarantee
		// target is reached. Note that model-space transform of each joint doesn't
		// need to be updated between each pass, as joints are ordered from child to
		// parent.
		int previous_joint = -1;
		for (int i = 0, joint = joints_chain_[0]; i < chain_length_;
			++i, previous_joint = joint, joint = joints_chain_[i]) {
			// Setups the model-space matrix of the joint being processed by IK
			ik_job.joint = &models[joint];
			// Setup joint local_space up vector.
			ik_job.up = Vector3::UnitY;

			// Setups weights of IK job
			// the last joint being processed needs a full weight (1.f) to ensure
			// target is reached
			const bool last = i == (chain_length_ - 1);
			ik_job.weight = chain_weight_ * (last ? 1.0f : joint_weight_);

			// Setup offset and forward vector for the current joint being processed.
			if (i == 0) {
				// First joint, uses global forward and offset..
				ik_job.forward = -Vector3::UnitZ;//FIXME: why negative?
				ik_job.offset = eyes_offset_;
			}
			else {
				// Applies previous correction to "forward" and "offset", before
				// bringing them to model-space (_ms).
				Vector3 corrected_forward_ms = Vector3::TransformVector(Vector3::Transform(ik_job.forward,
					correction),
					models[previous_joint]);
				Vector3 corrected_offset_ms = Vector3::Transform(Vector3::Transform(ik_job.offset,
					correction),
					models[previous_joint]);

				// Brings "forward" and "offset" to joint local-space
				Matrix inv_joint = models[joint].Invert();
				ik_job.forward = Vector3::TransformVector(corrected_forward_ms, inv_joint).Normalized();
				ik_job.offset = Vector3::Transform(corrected_offset_ms, inv_joint);
			}

			// Runs IK aim job
			if (!ik_job.Run()) {
				//return false;
			}
			// Apply IK quaternion to its respective local-space transforms.
			locals[joint].mRot.mValue = *ik_job.joint_correction * locals[joint].mRot.mValue;
		}
	}

	void Character::UpdateController(float dt) {
		character_controller.Update(dt);
		locals = character_controller.curr_bone_transforms;
	}
}