#include "IKChain.h"

namespace Animation
{
	void IKChain::AddBone(unsigned int id, float length)
	{
		this->joints.push_back(id);
		this->lengths.push_back(length);
		this->total_length += length;
		this->total_length_sqr = this->total_length * this->total_length;
	}

	int IKChain::GetFirstJoint() const
	{
		return joints[0];
	}

	int IKChain::GetLastJoint() const
	{
		return joints.back();
	}

	void IKChain::SetAlt(Vector3 fwd, Vector3 up, const std::vector<Transform>& tpose_w)
	{
		if (!tpose_w.empty()) {
			Quaternion q = tpose_w[joints[0]].mRot.mValue.Inversed();

			this->alt_fwd = Vector3::Transform(fwd, q);
			this->alt_up = Vector3::Transform(up, q);
		}
		else {
			LOG("Tpose is missing");
			this->alt_fwd = fwd;
			this->alt_up = up;
		}
	}

	void IKChain::ComputeLen(const std::vector<Transform>& tpose_w)
	{
		float sum = 0.0f;
		lengths.clear();
		for (size_t i = 0; i < joints.size() - 1; i++) {
			Vector3 bone = tpose_w[joints[i + 1]].mTrans.mValue - tpose_w[joints[i]].mTrans.mValue;
			lengths.push_back(bone.Length());
			sum += lengths.back();
		}

		if (end_idx != -1) {
			Vector3 bone = tpose_w[end_idx].mTrans.mValue - tpose_w[joints.back()].mTrans.mValue;
			lengths.push_back(bone.Length());
			sum += lengths.back();
		}

		this->total_length = sum;
		this->total_length_sqr = sum * sum;
	}
}