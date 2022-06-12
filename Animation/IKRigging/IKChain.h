#pragma once
#include "..//AnimationDatabase.h"

namespace Animation
{
	class IKChain
	{
	public:
		void AddBone(unsigned int id, float length)
		{
			this->joints.push_back(id);
			this->lengths.push_back(length);
			this->total_length += length;
			this->total_length_sqr = this->total_length * this->total_length;
		}

		

	private:
		std::vector<unsigned int> joints;

		std::vector<float> lengths;

		float total_length = 0.0f;

		float total_length_sqr = 0.0f;

		AnimationDatabase* skeleton;

	};
}