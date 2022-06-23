#pragma once

///<summary>
/// A Keyframe defines the bone transformation at an instant in time.
///</summary>

#include "../pch.h"

struct VectorKey
{
	DirectX::SimpleMath::Vector3 mValue = DirectX::SimpleMath::Vector3(0.0f, 0.0f, 0.0f);

	float mTimeTick = 0.0f;
};

struct QuatKey
{
	DirectX::SimpleMath::Quaternion mValue = DirectX::SimpleMath::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);

	float mTimeTick = 0.0f;
};

struct Transform
{
	QuatKey mRot;
	VectorKey mTrans;
	VectorKey mScale;

	static DirectX::SimpleMath::Matrix ToMatrix(const Transform& t)
	{
		return DirectX::SimpleMath::Matrix::CreateAffineTransformation(t.mScale.mValue, t.mTrans.mValue, t.mRot.mValue);
	}

	static Transform FromMatrix(DirectX::SimpleMath::Matrix m)
	{
		Transform t;
		m.Decompose(t.mScale.mValue, t.mRot.mValue, t.mTrans.mValue);
		return t;
	}

	Transform Add(const Transform& t)
	{
		DirectX::SimpleMath::Matrix m1 = ToMatrix(t);

		DirectX::SimpleMath::Matrix m2 = t.ToMatrix(*this);

		DirectX::SimpleMath::Matrix m = m1 * m2;

		*this = FromMatrix(m);

		return *this;
	}

	Transform Add(const DirectX::SimpleMath::Quaternion& rot, const DirectX::SimpleMath::Vector3& trans, const DirectX::SimpleMath::Vector3& scale)
	{
		Transform t;
		t.mRot.mValue = rot;
		t.mTrans.mValue = trans;
		t.mScale.mValue = scale;
		return this->Add(t);
	}
};
