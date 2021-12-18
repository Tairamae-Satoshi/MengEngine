#pragma once
#include "MotionAnalyzer.h"

namespace Animation
{
	class MotionAnalyzerBackwards
	{
	public:
		MotionAnalyzer orig;

		enum { kLeft, kRight, legSize };

		LegController* legC;

		const AnimationClip* animation;

		LegCycleData cycles[2];

		int sampleNum = 0;

		void Analyze();

		float mCycleDistance;

		Vector3 mCycleDirection;

		float mCycleDuration;

		float mCycleSpeed;


		void Analyze()
		{

		}
	};
}
