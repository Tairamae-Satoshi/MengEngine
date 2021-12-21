#include "CharacterController.h"

namespace Animation
{
	CharacterController::CharacterController() :
		position(Vector3::Zero),
		velocity(Vector3::Zero),
		acceleration(Vector3::Zero) {}


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
}