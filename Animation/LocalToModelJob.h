#pragma once
#include"Skeleton.h"

namespace Animation
{
	//class Skeleton;

	struct LocalToModelJob
	{
		LocalToModelJob();

		bool Validate() const;

		bool Run(bool local = true, bool offset = true);

		const Skeleton* skeleton;

		std::vector<Transform> input;

		std::vector<Matrix> output;
	};
}