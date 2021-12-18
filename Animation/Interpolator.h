#pragma once

#include"..//pch.h"

using namespace DirectX;
using namespace DirectX::SimpleMath;

namespace Animation
{
	class Interpolator
	{
	public:
		Interpolator(const std::vector<Vector2>& points)
		{
			mPoints = points;
		}

		virtual std::vector<float> Interpolate(const Vector2& samplePoint)
		{
			return std::vector<float>();
		}

		std::vector<Vector2> mPoints;

	};
}