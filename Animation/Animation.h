#pragma once
#include "AnimationKeyframe.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	///<summary>
	/// A BoneAnimation is defined by a list of keyframes.  For time
	/// values inbetween two keyframes, we interpolate between the
	/// two nearest keyframes that bound the time.  
	///
	/// We assume an animation always has two keyframes.
	///</summary>

	struct Transform
	{
		QuatKey mRot;
		VectorKey mTrans;
		VectorKey mScale;
	};

	struct BoneAnimationSample
	{
		std::string mName;

		std::vector<Transform> mLocalPose;

		float GetStartTime()const;

		float GetEndTime()const;

		void Interpolate(float t, DirectX::XMFLOAT4X4& M) const;

		void Interpolate(float t, Transform& P) const;
	};

	///<summary>
	/// Examples of AnimationClips are "Walk", "Run", "Attack", "Defend".
	/// An AnimationClip requires a BoneAnimation for every bone to form
	/// the animation clip.    
	///</summary>
	class AnimationClip
	{
	public:
		float GetClipStartTime() const;

		float GetClipEndTime() const;

		float GetDuration() const;

		double GetTickPerSecond() const;

		void Interpolate(float t, std::vector<DirectX::XMFLOAT4X4>& boneTransforms) const;

		void Interpolate(float t, std::vector<Transform>& transforms) const;

		void GetTransformAt(int tick, std::vector<Transform>& transforms) const;

		std::string mName;

		std::vector<BoneAnimationSample> mSamples;

		double mTicksPerSecond;
	};
}
