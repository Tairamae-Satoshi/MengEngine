#pragma once
#include "..//AnimationDatabase.h"

namespace Animation
{
	class IKPoint {
	public:
		IKPoint();

		IKPoint(unsigned int id);

		void Init(unsigned int id);

		void SetAlt(Vector3 fwd, Vector3 up, const std::vector<Transform>& tpose_w);

		int idx;

		Vector3 alt_fwd;

		Vector3 alt_up;
	};
}