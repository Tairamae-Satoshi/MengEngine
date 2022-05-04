#pragma once
#include "../pch.h"

// Reference: https://www.youtube.com/watch?v=jcpIrw38E-s

namespace Animation
{
	// Reference: https://www.youtube.com/watch?v=jcpIrw38E-s

	struct vec3d
	{

	};

	template<typename T>
	struct Array2D
	{
		int rows, cols;

		std::vector<T> data;

		Array2D(int r, int c) : rows(r), cols(c) { data.resize(rows * cols); }

		T& get(int r, int c) { return data[cols * r + c]; }

		const T& get(int r, int c) const { return data[cols * r + c]; }

	};

	struct AnimDataBase
	{
		std::vector<int> animStartIndices;
		std::vector<std::string> animNames;

		std::vector<vec3d> entityPosition;

		int PoseCount() { return 3224; }
	};

	struct RuntimeCharacterData
	{

	};

	struct Feature
	{
		virtual int Size() const = 0;

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimDataBase& animDatabase,
			int poseIndex
		) const = 0;

		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const = 0;

		float weight;
	};

	struct FeatureArray
	{
		std::vector<Feature*> features;

		std::vector<int> offsets;

		int totalDimCount;

		// After setting all features, call this:
		void ComputeOffset()
		{
			offsets.clear();

			int off = 0;
			for (const Feature* feature : features)
			{
				offsets.push_back(off);
				off += feature->Size();
			}
		}
	};

	void Build()
	{
		AnimDataBase animDatabase;
		// make this!

		FeatureArray featureArray;
		// ... too!


		featureArray.ComputeOffset();

		int pointCount = animDatabase.PoseCount();
		int dimCount = featureArray.totalDimCount;

		Array2D<float> matcherData(pointCount, dimCount);

		for (int featureIndex = 0; featureIndex < featureArray.features.size(); featureIndex++)
		{
			const Feature* feature = featureArray.features[featureIndex];

			for (int poseIndex = 0; poseIndex < pointCount; poseIndex++)
			{
				int featureDimOffset = featureArray.offsets[featureIndex];

				feature->EvaluateForAnimationPose(
					&matcherData.get(poseIndex, featureDimOffset),
					animDatabase,
					poseIndex
				);
			}
		}
	}

	template<typename T>
	struct Array2D
	{
		int rows, cols;

		std::vector<T> data;

		Array2D(int r, int c) : rows(r), cols(c) { data.resize(rows * cols); }
		
		T& get(int r, int c) { return data[cols * r + c]; }

		const T& get(int r, int c) const { return data[cols * r + c]; }

	};


	struct MotionMatchingJob
	{
	public:

		bool Run()
		{
			int dimCount = 50;
			int hours = 3;
			int fps = 30;
			int pointCount = hours * 60 * 60 * fps;

			Array2D<float> points(pointCount, dimCount);
		
			for (int poseIndex = 0; poseIndex < pointCount; poseIndex++)
			{
				for (int dimIndex = 0; dimIndex < dimCount; dimIndex++)
				{
					points.get(poseIndex, dimIndex) = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
				}
			}

			int answerWinnerPoseIndex = rand() % pointCount;

			std::vector<float> queryPoint;
			queryPoint.resize(dimCount);
			for (int dimIndex = 0; dimIndex < dimCount; dimIndex++)
			{
				queryPoint[dimIndex] = points.get(answerWinnerPoseIndex, dimIndex);
			}

			float smallestSquareDistanceToDate = FLT_MAX;
			int winnerPoseIndexFound = -1;

			for (int poseIndex = 0; poseIndex < pointCount; poseIndex++)
			{
				float squareDistance = 0.0f;

				for (int dimIndex = 0; dimIndex < dimCount; dimIndex++)
				{
					float distanceForThisDim = queryPoint[dimIndex] - points.get(poseIndex, dimIndex);
					squareDistance += distanceForThisDim * distanceForThisDim;
				}

				if (squareDistance < smallestSquareDistanceToDate)
				{
					smallestSquareDistanceToDate = squareDistance;
					winnerPoseIndexFound = poseIndex;
				}
			}


		}


	private:

	};
}


