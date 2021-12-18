#include "SamplingJob.h"

namespace Animation
{
	SamplingJob::SamplingJob() : ratio(0.0f), animation(nullptr) {}

	bool SamplingJob::Validate() const
	{
		bool valid = true;
		if (!animation)
		{
			return false;
		}

		return true;
	}

	bool SamplingJob::Run()
	{
		if (!Validate())
		{
			return false;
		}

		int numJoint = animation->mSamples.size();
		output.resize(numJoint);
		for (UINT i = 0; i < animation->mSamples.size(); ++i)
		{
			animation->mSamples[i].Interpolate(ratio, output[i]);
		}

		return true;
	}
}