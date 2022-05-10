#pragma once
#include "../pch.h"
#include "Spring.h"
#include "AnimationDatabase.h"
using namespace DirectX::SimpleMath;

// Reference: https://www.youtube.com/watch?v=jcpIrw38E-s

namespace Animation
{
	template<typename T>
	struct Array2D
	{
		int rows, cols;

		std::vector<T> data;

		Array2D() : rows(0), cols(0) { data.resize(rows * cols); }

		Array2D(int r, int c) : rows(r), cols(c) { data.resize(rows * cols); }

		T& get(int r, int c) { return data[cols * r + c]; }

		const T& get(int r, int c) const { return data[cols * r + c]; }

	};


	struct RuntimeCharacterData
	{
		Vector3 root_position;

		Quaternion root_rotation;

		std::vector<Vector3> trajectory_positions;

		std::vector<Quaternion> trajectory_rotations;
	};

	struct Feature
	{
		virtual int Size() const = 0;

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimationDatabase& animDatabase,
			int poseIndex
		) const = 0;

		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const = 0;

		float weight = 1.0f;
	};

	struct TrajectoryPositionFeature : Feature {
		virtual int Size() const override
		{
			return 6; // p1.x, p1.y, p2.x, p2.y, p3.x, p3.y 
		}

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimationDatabase& db,
			int poseIndex
		) const override
		{
			std::vector<Transform> transforms = db.GetTransformsAtPoseId(poseIndex);

			int t0 = db.ClampDatabaseTrajectoryIndex(poseIndex, 20);
			int t1 = db.ClampDatabaseTrajectoryIndex(poseIndex, 40);
			int t2 = db.ClampDatabaseTrajectoryIndex(poseIndex, 60);

			//{
			//	char out[100];
			//	//sprintf(out, "%d: %f, %f, %f", poseIndex, db.GetBonePosition(poseIndex, 0).x, db.GetBonePosition(poseIndex, 0).y, db.GetBonePosition(poseIndex, 0).z);
			//	sprintf(out, "t0 = %d, i = %d: %f, %f, %f | ", t0, poseIndex,
			//		db.GetBonePosition(t0, 0).z - db.GetBonePosition(poseIndex, 0).z,
			//		db.GetBonePosition(t1, 0).z - db.GetBonePosition(poseIndex, 0).z,
			//		db.GetBonePosition(t2, 0).z - db.GetBonePosition(poseIndex, 0).z);
			//	Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 196, out);

			//}
			//if (poseIndex > 400)
			//{
			//	int a = 2;
			//}
			Vector3 trajectoryPos0 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), db.GetBonePosition(t0, 0) - db.GetBonePosition(poseIndex, 0));
			Vector3 trajectoryPos1 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), db.GetBonePosition(t1, 0) - db.GetBonePosition(poseIndex, 0));
			Vector3 trajectoryPos2 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), db.GetBonePosition(t2, 0) - db.GetBonePosition(poseIndex, 0));

			// Need trasform the coordinate relative to simulation rotation		
			ResultLocation[0] = trajectoryPos0.x;
			ResultLocation[1] = trajectoryPos0.z;
			ResultLocation[2] = trajectoryPos1.x;
			ResultLocation[3] = trajectoryPos1.z;
			ResultLocation[4] = trajectoryPos2.x;
			ResultLocation[5] = trajectoryPos2.z;
		}



		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const override
		{
			Vector3 traj0 = quat_inv_mul_vec3(runtimeData.root_rotation, runtimeData.trajectory_positions[1] - runtimeData.root_position);
			Vector3 traj1 = quat_inv_mul_vec3(runtimeData.root_rotation, runtimeData.trajectory_positions[2] - runtimeData.root_position);
			Vector3 traj2 = quat_inv_mul_vec3(runtimeData.root_rotation, runtimeData.trajectory_positions[3] - runtimeData.root_position);

			int i = 0;
			ResultLocation[i++] = traj0.x;
			ResultLocation[i++] = traj0.z;
			ResultLocation[i++] = traj1.x;
			ResultLocation[i++] = traj1.z;
			ResultLocation[i++] = traj2.x;
			ResultLocation[i++] = traj2.z;
		}
	};

	struct TrajectoryDirectionFeature : Feature {
		virtual int Size() const override
		{
			return 6; // p1.x, p1.y, p2.x, p2.y, p3.x, p3.y 
		}

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimationDatabase& db,
			int poseIndex
		) const override
		{
			int t0 = db.ClampDatabaseTrajectoryIndex(poseIndex, 20);
			int t1 = db.ClampDatabaseTrajectoryIndex(poseIndex, 40);
			int t2 = db.ClampDatabaseTrajectoryIndex(poseIndex, 60);

			Vector3 trajectoryDir0 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), quat_mul_vec3(db.GetBoneRotation(t0, 0), Vector3(0, 0, 1)));
			Vector3 trajectoryDir1 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), quat_mul_vec3(db.GetBoneRotation(t1, 0), Vector3(0, 0, 1)));
			Vector3 trajectoryDir2 = quat_mul_vec3(db.GetBoneRotation(poseIndex, 0).Inversed(), quat_mul_vec3(db.GetBoneRotation(t2, 0), Vector3(0, 0, 1)));

			// Need trasform the coordinate relative to simulation rotation
			ResultLocation[0] = trajectoryDir0.x;
			ResultLocation[1] = trajectoryDir0.z;
			ResultLocation[2] = trajectoryDir1.x;
			ResultLocation[3] = trajectoryDir1.z;
			ResultLocation[4] = trajectoryDir2.x;
			ResultLocation[5] = trajectoryDir2.z;

		}

		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const override
		{
			Vector3 traj0 = quat_inv_mul_vec3(runtimeData.root_rotation, quat_mul_vec3(runtimeData.trajectory_rotations[1], Vector3(0, 0, 1)));
			Vector3 traj1 = quat_inv_mul_vec3(runtimeData.root_rotation, quat_mul_vec3(runtimeData.trajectory_rotations[2], Vector3(0, 0, 1)));
			Vector3 traj2 = quat_inv_mul_vec3(runtimeData.root_rotation, quat_mul_vec3(runtimeData.trajectory_rotations[3], Vector3(0, 0, 1)));

			int i = 0;
			ResultLocation[i++] = traj0.x;
			ResultLocation[i++] = traj0.z;
			ResultLocation[i++] = traj1.x;
			ResultLocation[i++] = traj1.z;
			ResultLocation[i++] = traj2.x;
			ResultLocation[i++] = traj2.z;

		}
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

			totalDimCount = off;
		}
	};



	struct MotionMatchingJob
	{
	public:
		MotionMatchingJob() {}

		void Build()
		{
			featureArray.features.push_back(&trajectoryPositionFeature);
			featureArray.features.push_back(&trajectoryDirectionFeature);

			featureArray.ComputeOffset();

			int pointCount = animDatabase->totalPoseCount;
			int dimCount = featureArray.totalDimCount;

			matcherData = Array2D<float>(pointCount, dimCount);
			featuresOffset.resize(dimCount);
			featuresScale.resize(dimCount);

			for (int featureIndex = 0; featureIndex < featureArray.features.size(); featureIndex++)
			{
				Feature* feature = featureArray.features[featureIndex];
				int featureDimOffset = featureArray.offsets[featureIndex];

				for (int poseIndex = 0; poseIndex < pointCount; poseIndex++)
				{

					feature->EvaluateForAnimationPose(
						&matcherData.get(poseIndex, featureDimOffset),
						*animDatabase,
						poseIndex
					);
				}

				NormalizeFeature(feature, featureDimOffset, 1.0f);
			}
		}


		bool Run(std::vector<float> queryPoint)
		{
			// Normalize Query
			std::vector<float> normalizedQuery;
			for (int i = 0; i < queryPoint.size(); i++)
			{
				normalizedQuery.push_back((queryPoint[i] - featuresOffset[i]) / featuresScale[i]);
			}

			// Search

			for (int poseIndex = 0; poseIndex < matcherData.rows; poseIndex++)
			{
				float currCost = 0.0f;

				for (int dimIndex = 0; dimIndex < matcherData.cols; dimIndex++)
				{
					float distanceForThisDim = normalizedQuery[dimIndex] - matcherData.get(poseIndex, dimIndex);
					currCost += distanceForThisDim * distanceForThisDim;
				}

				if (currCost < bestCost)
				{
					bestCost = currCost;
					bestIndex = poseIndex;
				}
			}

			return true;
		}

		Array2D<float> matcherData;

		std::vector<float> featuresOffset;

		std::vector<float> featuresScale;

		AnimationDatabase* animDatabase;

		TrajectoryPositionFeature trajectoryPositionFeature;

		TrajectoryDirectionFeature trajectoryDirectionFeature;

		FeatureArray featureArray;

		float bestCost = FLT_MAX;

		int bestIndex = -1;

		void NormalizeFeature(
			Feature* feature,
			const int offset,
			const float weight)
		{
			// First compute what is essentially the mean value for each feature dimension
			std::vector<float> vars;
			for (int j = 0; j < feature->Size(); j++)
			{
				featuresOffset[offset + j] = 0.0f;
				vars.push_back(0.0f);
			}

			for (int i = 0; i < matcherData.rows; i++)
			{
				for (int j = 0; j < feature->Size(); j++)
				{
					featuresOffset[offset + j] += matcherData.get(i, offset + j) / matcherData.rows;
				}
			}

			// Now compute the variance of each feature dimension
			for (int i = 0; i < matcherData.rows; i++)
			{
				for (int j = 0; j < feature->Size(); j++)
				{
					vars[j] += squaref(matcherData.get(i, offset + j) - featuresOffset[offset + j]) / matcherData.rows;
				}
			}

			// We compute the overall std of the feature as the average std across all dimensions
			float std = 0.0f;
			for (int j = 0; j < feature->Size(); j++)
			{
				std += sqrtf(vars[j]) / feature->Size();
			}

			// Features with no variation can have zero std which is
			// almost always a bug.
			assert(std > 0.0);

			// The scale of a feature is just the std divided by the weight
			for (int j = 0; j < feature->Size(); j++)
			{
				featuresScale[offset + j] = std / weight;
			}

			// Using the offset and scale we can then normalize the features
			for (int i = 0; i < matcherData.rows; i++)
			{
				for (int j = 0; j < feature->Size(); j++)
				{
					matcherData.get(i, offset + j) = (matcherData.get(i, offset + j) - featuresOffset[offset + j]) / featuresScale[offset + j];
				}
			}

		}
	};
}


