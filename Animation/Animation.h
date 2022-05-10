#pragma once
#include "AnimationKeyframe.h"
#include "Common.h"

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

	// The transform series of a joint
	struct BoneAnimationSample
	{
		std::string mName;

		std::vector<Transform> mLocalPose;

		BoneAnimationSample();

		float get_start_time_in_tick()const;

		float get_end_time_in_tick()const;

		int get_position_index(float tick) const;

		int get_rotation_index(float tick) const;

		int get_scale_index(float tick) const;

		float get_factor(float last, float next, float t) const;

		void interpolate(float t, DirectX::XMFLOAT4X4& M) const;

		void interpolate(float t, Transform& P) const;

		void convert_to_fps(float fps, float original_tick_per_second, float duration_tick);
	};

	///<summary>
	/// Examples of AnimationClips are "Walk", "Run", "Attack", "Defend".
	/// An AnimationClip requires a BoneAnimation for every bone to form
	/// the animation clip.    
	///</summary>
	class AnimationClip
	{
	public:
		float get_clip_start_time() const;

		float get_clip_end_time() const;

		float get_pose_count() const;

		double get_tick_per_second() const;

		float get_duration_in_second() const;

		float get_duration_in_tick() const;

		void interpolate(float t, std::vector<DirectX::XMFLOAT4X4>& boneTransforms) const;

		void interpolate(float t, std::vector<Transform>& transforms) const;

		void convert_to_fps(float fps);

		std::vector<Transform> get_transform_at(int tick) const;

		std::string mName;

		std::vector<BoneAnimationSample> mSamples;

		double mTicksPerSecond;
	};
}
