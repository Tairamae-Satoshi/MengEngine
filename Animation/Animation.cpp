#include "Animation.h"
#include <algorithm>

using namespace DirectX;

namespace Animation
{
	float BoneAnimationSample::GetStartTime() const
	{
		return 0.0f;
	}

	float BoneAnimationSample::GetEndTime() const
	{
		return mLocalPose.size() - 1;
	}

	void BoneAnimationSample::Interpolate(float t, XMFLOAT4X4& M) const
	{
		t = t * (GetEndTime() - GetStartTime()) + GetStartTime();

		if (t <= GetStartTime())
		{
			Vector3 S = mLocalPose.front().mScale.mValue;
			Vector3  P = mLocalPose.front().mTrans.mValue;
			Quaternion Q = mLocalPose.front().mRot.mValue;

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
		if (t >= GetEndTime())
		{
			Vector3 S = mLocalPose.back().mScale.mValue;
			Vector3 P = mLocalPose.back().mTrans.mValue;
			Quaternion Q = mLocalPose.back().mRot.mValue;

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
		else
		{
			XMVECTOR S, P, Q;
			float ts = floorf(t);
			float lerpPercent = (t - ts);
			UINT i = (UINT)ts;

			S = Vector3::Lerp(mLocalPose[i].mScale.mValue, mLocalPose[i + 1].mScale.mValue, lerpPercent);
			P = Vector3::Lerp(mLocalPose[i].mTrans.mValue, mLocalPose[i + 1].mTrans.mValue, lerpPercent);
			Q = Quaternion::Slerp(mLocalPose[i].mRot.mValue, mLocalPose[i + 1].mRot.mValue, lerpPercent);

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
	}

	void BoneAnimationSample::Interpolate(float t, Transform& P) const
	{
		t = t * (GetEndTime() - GetStartTime());

		if (t <= GetStartTime())
		{
			P.mScale.mValue = mLocalPose.front().mScale.mValue;
			P.mTrans.mValue = mLocalPose.front().mTrans.mValue;
			P.mRot.mValue = mLocalPose.front().mRot.mValue;
		}
		if (t >= GetEndTime())
		{
			P.mScale.mValue = mLocalPose.back().mScale.mValue;
			P.mTrans.mValue = mLocalPose.back().mTrans.mValue;
			P.mRot.mValue = mLocalPose.back().mRot.mValue;
		}
		else
		{
			float ts = floorf(t);
			float lerpPercent = (t - ts);
			UINT i = (UINT)ts;

			P.mScale.mValue = Vector3::Lerp(mLocalPose[i].mScale.mValue, mLocalPose[i + 1].mScale.mValue, lerpPercent);
			P.mTrans.mValue = Vector3::Lerp(mLocalPose[i].mTrans.mValue, mLocalPose[i + 1].mTrans.mValue, lerpPercent);
			P.mRot.mValue = Quaternion::Slerp(mLocalPose[i].mRot.mValue, mLocalPose[i + 1].mRot.mValue, lerpPercent);
		}
	}


	float AnimationClip::GetClipStartTime() const
	{
		// Find smallest start time over all bones in this clip.
		float t = MathHelper::Infinity;
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			t = MathHelper::Min(t, mSamples[i].GetStartTime());
		}

		return t / mTicksPerSecond;
	}

	float AnimationClip::GetClipEndTime() const
	{
		// Find largest end time over all bones in this clip.
		float t = 0.0f;
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			t = MathHelper::Max(t, mSamples[i].GetEndTime());
		}

		return t / mTicksPerSecond;
	}

	float AnimationClip::GetDuration() const
	{
		return GetClipEndTime() - GetClipStartTime();
	}

	double AnimationClip::GetTickPerSecond() const
	{
		return mTicksPerSecond;
	}


	void AnimationClip::Interpolate(float t, std::vector<XMFLOAT4X4>& boneTransforms) const
	{
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			mSamples[i].Interpolate(t, boneTransforms[i]);
		}
	}

	void AnimationClip::Interpolate(float t, std::vector<Transform>& transforms) const
	{
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			mSamples[i].Interpolate(t, transforms[i]);
		}
	}

	void AnimationClip::GetTransformAt(int tick, std::vector<Transform>& transforms) const
	{
		for (size_t i = 0; i < mSamples.size(); i++)
		{
			transforms[i] = mSamples[i].mLocalPose[tick];
		}
	}
}
 
