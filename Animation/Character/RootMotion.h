#pragma once

#include "../pch.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class RootMotion
	{
	public:
		Vector3 forward;

		Vector3 up;

		Vector3 position;

		Quaternion rotaion;

		Vector3 last_position = Vector3::Zero;

		Quaternion last_rotaion = Quaternion::Identity;

		Vector3 motion_begin_position = Vector3::Zero;

		Quaternion motion_begin_rotaion = Quaternion::Identity;


		Matrix transform;

		Matrix last_transform;

		RootMotion();

		void ApplyRootTransform(const Quaternion& q, const Vector3& t, bool _transition);
	};
}