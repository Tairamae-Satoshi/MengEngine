#pragma once
#include"Animation.h"

namespace Animation
{
	struct SamplingJob
	{
		SamplingJob();

		bool Validate() const;

		// In a real project, you'd want to cache the result if there was a chance
		// that you were calling this several times with the same clipName at 
		// the same timePos.
		bool Run();

		float ratio;

		const AnimationClip* animation;

		std::vector<Transform> output;
	};
}