#include "IKPoint.h"

namespace Animation
{
	IKPoint::IKPoint() :
		idx(-1)
	{}

	IKPoint::IKPoint(unsigned int id) :
		idx(id)
	{}

	void IKPoint::Init(unsigned int id) {
		this->idx = id;
	}

	void IKPoint::SetAlt(Vector3 fwd, Vector3 up, const std::vector<Transform>& tpose_w)
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
}