#pragma once

#include "..//pch.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	struct IKThreeBoneJob
	{
		IKThreeBoneJob();

		bool Run();

		Vector3 target;

		Vector3 mid_axis; // In model space

		Vector3 pole_vector;

		std::vector<Matrix> joints;

		std::vector<Quaternion> joints_correction;

		bool reached;
	};
}