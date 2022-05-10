#pragma once
#include"AnimationDatabase.h"

namespace Animation
{
	//class Skeleton;

	struct LocalToModelJob
	{
		LocalToModelJob();

		bool Validate() const;

		bool Run(bool local = true, bool offset = true);

		const AnimationDatabase* skeleton;

		std::vector<Transform> input;

		std::vector<Matrix> output;
	};
}