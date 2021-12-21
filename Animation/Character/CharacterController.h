#pragma once

#include "../pch.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class CharacterController
	{
	public:
		CharacterController();

		Vector3 position = Vector3::Zero;

		Vector3 velocity = Vector3::Zero;

		Vector3 acceleration = Vector3::Zero;

		Vector3 target_direction = Vector3::Zero;

		void Update(const Vector3& _direction, float _t);

	private:
		void UpdateFacingDirection(const Vector3& direction, float t);

		void UpdateVelocity(const Vector3& direction, float t);

		void UpdateSpringCharacter(float& x, float& v, float& a, float v_goal, float halflife, float dt);

		float max_forward_speed = 1.0f;
		float max_backwards_speed = 1.0f;
		float max_sideways_speed = 1.0f;
		float max_velocity_change = 0.2f;

	};
}