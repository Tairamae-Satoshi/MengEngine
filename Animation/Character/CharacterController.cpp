#include "CharacterController.h"

namespace Animation
{
	CharacterController::CharacterController() :
		position(Vector3::Zero),
		velocity(Vector3::Zero),
		acceleration(Vector3::Zero) {}

	void CharacterController::Initialize(AnimationDatabase* db)
	{
		position = Vector3::Zero;
		velocity = Vector3::Zero;
		acceleration = Vector3::Zero;

		this->mm.animDatabase = db;
		this->db = db;

		frame_index = db->rangeStarts[0];
		curr_bone_transforms = db->GetTransformsAtPoseId(frame_index);

		trajectory_desired_velocities.resize(trajectory_points_size);
		trajectory_desired_rotations.resize(trajectory_points_size);
		trajectory_positions.resize(trajectory_points_size);
		trajectory_velocities.resize(trajectory_points_size);
		trajectory_accelerations.resize(trajectory_points_size);
		trajectory_rotations.resize(trajectory_points_size);
		trajectory_angular_velocities.resize(trajectory_points_size);

		// Build Motion Matching Job
		mm.Build();
	}

	float halflife_to_damping(float halflife, float eps = 1e-5f)
	{
		return (4.0f * 0.69314718056f) / (halflife + eps);
	}

	float fast_negexp(float x)
	{
		return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
	}

	void CharacterController::UpdateSimulationPositions(
		Vector3& position,
		Vector3& velocity,
		Vector3& acceleration,
		const Vector3& desired_velocity,
		const float halflife,
		const float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		Vector3 j0 = velocity - desired_velocity;
		Vector3 j1 = acceleration + j0 * y;
		float eydt = fast_negexp(y * dt);

		Vector3 position_prev = position;

		position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
			(j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
		velocity = eydt * (j0 + j1 * dt) + desired_velocity;
		acceleration = eydt * (acceleration - j1 * y * dt);

	}
}