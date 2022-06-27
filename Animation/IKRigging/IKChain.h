#pragma once
#include "..//AnimationDatabase.h"

namespace Animation
{
	class IKChain
	{
	public:
		void AddBone(unsigned int id, float length);

		int GetFirstJoint() const;

		int GetLastJoint() const;

		void SetAlt(Vector3 fwd, Vector3 up, const std::vector<Transform>& tpose_w);

		void ComputeLen(const std::vector<Transform>& tpose_w);

		std::vector<int> joints;

		std::vector<float> lengths;

		unsigned int end_idx = -1;

		float total_length = 0.0f;

		float total_length_sqr = 0.0f;

		Vector3 alt_fwd;

		Vector3 alt_up;

		std::string ik_solver;
	};
}