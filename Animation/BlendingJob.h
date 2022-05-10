#pragma once
#include"Animation.h"
#include"AnimationDatabase.h"

namespace Animation
{
	class PolarGradientBandInterpolator;

	struct BlendingJob
	{
	public:
		bool Run();

		struct Layer
		{
			Layer();

			const AnimationClip* animation;

			float weight;

			float K[4];

			int Nk; // Keytimes count: 0 ~ Nk-1

			int m; // T > K[m]

			float T = 0.0f; // Actual time

			float t = 0.0f; // Generic time

			float deltaT = 0.0f;

			float deltat = 0.0f;

			Vector3 velocity;
		};

		std::vector<Layer> layers;

		std::shared_ptr<PolarGradientBandInterpolator> interpolator;

		//std::vector<Layer> mAdditiveLayers;

		std::vector<Transform> output;

		float T = 0.0f;

		float deltaT = 0.0f;

		float t = 0.0f;

		float deltat = 0.0f;

		bool transition = false;

		//bool weight_transition = false;

		bool Validate();

		bool OnGui();

		float GetDuration();

	private:
		Quaternion CalculateReferenceQ(const std::vector<Quaternion>& qs);
	};


}

