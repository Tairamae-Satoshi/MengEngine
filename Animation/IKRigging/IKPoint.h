#pragma once
#include "..//AnimationDatabase.h"

namespace Animation
{
	class IKPoint {
	public:
		IKPoint():
			idx(-1)
		{}

		IKPoint(unsigned int id):
			idx(id)
		{}

		void Init(unsigned int id) {
			this->idx = id;
		}

		void SetAlt(Vector3 fwd, Vector3 up, const std::vector<Transform>& tpose_w)
		{
			if (!tpose_w.empty()) {
				Quaternion q = tpose_w[idx].mRot.mValue.Inversed();

				this->alt_fwd = Vector3::Transform(fwd, q);
				this->alt_up = Vector3::Transform(up, q);
			}
			else {
				this->alt_fwd = fwd;
				this->alt_up = up;
			}
		}

		int idx;

		Vector3 alt_fwd;

		Vector3 alt_up;
	};
}