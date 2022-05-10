#include "Animation.h"

using namespace DirectX;

namespace Animation
{
	BoneAnimationSample::BoneAnimationSample() {

	}


	float BoneAnimationSample::get_start_time_in_tick() const
	{
		return 0.0f;
	}

	float BoneAnimationSample::get_end_time_in_tick() const
	{
		return mLocalPose[mLocalPose.size() - 1].mTrans.mTimeTick;
	}

	int BoneAnimationSample::get_position_index(float tick) const
	{
		for (int i = 0; i < mLocalPose.size() - 1; i++)
		{
			if (tick < mLocalPose[i + 1].mTrans.mTimeTick)
				return i;
		}

		assert(false);
		return -1;
	}

	int BoneAnimationSample::get_rotation_index(float tick) const
	{
		for (int i = 0; i < mLocalPose.size() - 1; i++)
		{
			if (tick < mLocalPose[i + 1].mRot.mTimeTick)
				return i;
		}

		assert(false);
		return -1;

	}

	int BoneAnimationSample::get_scale_index(float tick) const
	{
		for (int i = 0; i < mLocalPose.size() - 1; i++)
		{
			if (tick < mLocalPose[i + 1].mScale.mTimeTick)
				return i;
		}

		assert(false);
		return -1;

	}

	float BoneAnimationSample::get_factor(float last, float next, float t) const
	{
		return (t - last) / (next - last);
	}

	void BoneAnimationSample::interpolate(float ratio, XMFLOAT4X4& M) const
	{
		float time_in_tick = ratio * (get_end_time_in_tick() - get_start_time_in_tick()) + get_start_time_in_tick();

		if (time_in_tick <= get_start_time_in_tick())
		{
			Vector3 S = mLocalPose.front().mScale.mValue;
			Vector3  P = mLocalPose.front().mTrans.mValue;
			Quaternion Q = mLocalPose.front().mRot.mValue;

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
		if (time_in_tick >= get_end_time_in_tick())
		{
			Vector3 S = mLocalPose.back().mScale.mValue;
			Vector3 P = mLocalPose.back().mTrans.mValue;
			Quaternion Q = mLocalPose.back().mRot.mValue;

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
		else
		{
			XMVECTOR S, P, Q;

			int scale_index = get_scale_index(time_in_tick);
			int position_index = get_position_index(time_in_tick);
			int rotation_index = get_rotation_index(time_in_tick);

			float scale_factor = get_factor(mLocalPose[scale_index].mScale.mTimeTick, mLocalPose[scale_index + 1].mScale.mTimeTick, time_in_tick);
			float position_factor = get_factor(mLocalPose[scale_index].mTrans.mTimeTick, mLocalPose[scale_index + 1].mTrans.mTimeTick, time_in_tick);
			float rotation_factor = get_factor(mLocalPose[scale_index].mRot.mTimeTick, mLocalPose[scale_index + 1].mRot.mTimeTick, time_in_tick);

			S = Vector3::Lerp(mLocalPose[scale_index].mScale.mValue, mLocalPose[scale_index + 1].mScale.mValue, scale_factor);
			P = Vector3::Lerp(mLocalPose[position_index].mTrans.mValue, mLocalPose[position_index + 1].mTrans.mValue, position_factor);
			Q = Quaternion::Slerp(mLocalPose[rotation_index].mRot.mValue, mLocalPose[rotation_index + 1].mRot.mValue, rotation_factor);

			XMStoreFloat4x4(&M, XMMatrixAffineTransformation(S, Vector3::Zero, Q, P));
		}
	}


	void BoneAnimationSample::interpolate(float ratio, Transform& P) const
	{
		float time_in_tick = ratio * (get_end_time_in_tick() - get_start_time_in_tick()) + get_start_time_in_tick();

		if (time_in_tick <= get_start_time_in_tick())
		{
			P.mScale.mValue = mLocalPose.front().mScale.mValue;
			P.mTrans.mValue = mLocalPose.front().mTrans.mValue;
			P.mRot.mValue = mLocalPose.front().mRot.mValue;
		}
		if (time_in_tick >= get_end_time_in_tick())
		{
			P.mScale.mValue = mLocalPose.back().mScale.mValue;
			P.mTrans.mValue = mLocalPose.back().mTrans.mValue;
			P.mRot.mValue = mLocalPose.back().mRot.mValue;
		}
		else
		{
			int scale_index = get_scale_index(time_in_tick);
			int position_index = get_position_index(time_in_tick);
			int rotation_index = get_rotation_index(time_in_tick);

			float scale_factor = get_factor(mLocalPose[scale_index].mScale.mTimeTick, mLocalPose[scale_index + 1].mScale.mTimeTick, time_in_tick);
			float position_factor = get_factor(mLocalPose[scale_index].mTrans.mTimeTick, mLocalPose[scale_index + 1].mTrans.mTimeTick, time_in_tick);
			float rotation_factor = get_factor(mLocalPose[scale_index].mRot.mTimeTick, mLocalPose[scale_index + 1].mRot.mTimeTick, time_in_tick);

			P.mScale.mValue = Vector3::Lerp(mLocalPose[scale_index].mScale.mValue, mLocalPose[scale_index + 1].mScale.mValue, scale_factor);
			P.mTrans.mValue = Vector3::Lerp(mLocalPose[position_index].mTrans.mValue, mLocalPose[position_index + 1].mTrans.mValue, position_factor);
			P.mRot.mValue = Quaternion::Slerp(mLocalPose[rotation_index].mRot.mValue, mLocalPose[rotation_index + 1].mRot.mValue, rotation_factor);
			P.mScale.mTimeTick = time_in_tick;
			P.mTrans.mTimeTick = time_in_tick;
			P.mRot.mTimeTick = time_in_tick;
		}
	}

	void BoneAnimationSample::convert_to_fps(float fps, float original_tick_per_second, float duration_tick)
	{
		float duration_in_second = duration_tick / original_tick_per_second;

		float dt = 1.0f / fps;
		std::vector<Transform> new_local_pose;

		for (float t = 0.0f, tick = 0.0f; t < duration_in_second; t += dt, tick++)
		{
			float ratio = t / duration_in_second;
			Transform transform;
			interpolate(ratio, transform);
			transform.mTrans.mTimeTick = tick;
			transform.mRot.mTimeTick = tick;
			transform.mScale.mTimeTick = tick;
			new_local_pose.push_back(transform);
		}

		mLocalPose = new_local_pose;
	}


	float AnimationClip::get_clip_start_time() const
	{
		// Find smallest start time over all bones in this clip.
		float t = FLT_MAX;
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			t = minf(t, mSamples[i].get_start_time_in_tick());
		}

		return t / mTicksPerSecond;
	}

	float AnimationClip::get_clip_end_time() const
	{
		// Find largest end time over all bones in this clip.
		float t = 0.0f;
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			t = maxf(t, mSamples[i].get_end_time_in_tick());
		}

		return t / mTicksPerSecond;
	}

	float AnimationClip::get_pose_count() const
	{
		int max = 0;
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			max = maxf(max, mSamples[i].mLocalPose.size());
		}

		return max;
	}

	double AnimationClip::get_tick_per_second() const
	{
		return mTicksPerSecond;
	}

	float AnimationClip::get_duration_in_second() const
	{
		return get_clip_end_time() - get_clip_start_time();
	}

	float AnimationClip::get_duration_in_tick() const
	{
		return get_duration_in_second() * mTicksPerSecond;
	}

	void AnimationClip::interpolate(float t, std::vector<XMFLOAT4X4>& boneTransforms) const
	{
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			mSamples[i].interpolate(t, boneTransforms[i]);
		}
	}

	void AnimationClip::interpolate(float t, std::vector<Transform>& transforms) const
	{
		for (UINT i = 0; i < mSamples.size(); ++i)
		{
			mSamples[i].interpolate(t, transforms[i]);
		}
	}

	void AnimationClip::convert_to_fps(float fps)
	{
		float duration_tick = get_duration_in_tick();

		for (auto& sample : mSamples)
		{
			sample.convert_to_fps(fps, mTicksPerSecond, duration_tick);
		}

		mTicksPerSecond = fps;
	}

	std::vector<Transform> AnimationClip::get_transform_at(int tick) const
	{
		std::vector<Transform> transforms;

		for (size_t i = 0; i < mSamples.size(); i++)
		{
			transforms.push_back(mSamples[i].mLocalPose[tick]);
		}

		return transforms;
	}
}

