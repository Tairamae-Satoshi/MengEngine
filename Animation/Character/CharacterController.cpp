#include "CharacterController.h"

namespace Animation
{
	CharacterController::CharacterController() :
		position(Vector3::Zero),
		velocity(Vector3::Zero),
		acceleration(Vector3::Zero) {
		trajectory_desired_velocities.resize(trajectory_points_size);
		trajectory_desired_rotations.resize(trajectory_points_size);
		trajectory_positions.resize(trajectory_points_size);
		trajectory_velocities.resize(trajectory_points_size);
		trajectory_accelerations.resize(trajectory_points_size);
		trajectory_rotations.resize(trajectory_points_size);
		trajectory_angular_velocities.resize(trajectory_points_size);
	}


	void CharacterController::Update(const Vector3& _direction, float _t)
	{
		target_direction = _direction;
		UpdateFacingDirection(_direction, _t);
		UpdateVelocity(_direction, _t);
	}

	void CharacterController::UpdateFacingDirection(const Vector3& _direction, float _t)
	{

	}

	void CharacterController::UpdateVelocity(const Vector3& _direction, float _t)
	{
		Vector3 target_velocity = _direction.Normalized();;
		UpdateSpringCharacter(position.x, velocity.x, acceleration.x, target_velocity.x, 0.5, _t);
		UpdateSpringCharacter(position.y, velocity.y, acceleration.y, target_velocity.y, 0.5, _t);
		UpdateSpringCharacter(position.z, velocity.z, acceleration.z, target_velocity.z, 0.5, _t);
	}

	float halflife_to_damping(float halflife, float eps = 1e-5f)
	{
		return (4.0f * 0.69314718056f) / (halflife + eps);
	}

	float fast_negexp(float x)
	{
		return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
	}

	void CharacterController::UpdateSpringCharacter(float& x, float& v, float& a, float v_goal, float halflife, float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		float j0 = v - v_goal;
		float j1 = a + j0 * y;
		float eydt = fast_negexp(y * dt);

		x = eydt * ((j1 * dt) / y) + dt * v_goal + x;
		v = eydt * (j0 + j1 * dt) + v_goal;
		a = eydt * (a - j1 * y * dt);
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