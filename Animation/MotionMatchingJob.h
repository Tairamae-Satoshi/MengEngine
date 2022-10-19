#pragma once
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

		std::vector<T> get_row(int r) const {
			std::vector<T> row;
			for (int i = 0; i < cols; i++)
			{
				row.push_back(data[cols * r + i]);
			}
			return row;
		}
	};


	struct RuntimeCharacterData
	{
		std::vector<Transform> transforms;

		std::vector<int> parent_index;

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

			int interval = 20;
			int t0 = db.ClampDatabaseTrajectoryIndex(poseIndex, interval);
			int t1 = db.ClampDatabaseTrajectoryIndex(poseIndex, 2 * interval);
			int t2 = db.ClampDatabaseTrajectoryIndex(poseIndex, 3 * interval);

			Quaternion base = Quaternion::CreateFromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), MathHelper::Pi);
			Vector3 trajectoryPos0 = quat_mul_vec3((/*db.GetBoneRotation(poseIndex, 0) **/ base).Inversed(), db.GetBonePosition(t0, 0) - db.GetBonePosition(poseIndex, 0));
			Vector3 trajectoryPos1 = quat_mul_vec3((/*db.GetBoneRotation(poseIndex, 0) **/ base).Inversed(), db.GetBonePosition(t1, 0) - db.GetBonePosition(poseIndex, 0));
			Vector3 trajectoryPos2 = quat_mul_vec3((/*db.GetBoneRotation(poseIndex, 0) **/ base).Inversed(), db.GetBonePosition(t2, 0) - db.GetBonePosition(poseIndex, 0));

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
			int interval = 20;
			int t0 = db.ClampDatabaseTrajectoryIndex(poseIndex, interval);
			int t1 = db.ClampDatabaseTrajectoryIndex(poseIndex, 2 * interval);
			int t2 = db.ClampDatabaseTrajectoryIndex(poseIndex, 3 * interval);

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

	void ForwardKinematics(
		Vector3& bone_position,
		Quaternion& bone_rotation,
		const std::vector<Transform>& bone_transforms,
		const std::vector<int>& bone_parents,
		const int bone);

	struct LeftFootPositionFeature :Feature{
		virtual int Size() const override
		{
			return 3; 
		}

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimationDatabase& db,
			int poseIndex
		) const override
		{

			int leftFootId = 46; // Hard coding for now
			// 46 50
			Vector3 bone_position;
			Quaternion bone_rotation;

			std::vector<Transform> transform = db.GetTransformsAtPoseId(poseIndex);
			ForwardKinematics(
				bone_position,
				bone_rotation,
				transform,
				db.GetParentIndex(),
				leftFootId);

			bone_position = quat_inv_mul_vec3(transform[0].mRot.mValue, bone_position - transform[0].mTrans.mValue);

			// Need trasform the coordinate relative to simulation rotation
			ResultLocation[0] = bone_position.x;
			ResultLocation[1] = bone_position.y;
			ResultLocation[2] = bone_position.z;

		}

		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const override
		{
			int leftFootId = 46; // Hard coding for now
			// 46 50
			Vector3 bone_position;
			Quaternion bone_rotation;

			ForwardKinematics(
				bone_position,
				bone_rotation,
				runtimeData.transforms,
				runtimeData.parent_index,
				leftFootId);

			bone_position = quat_inv_mul_vec3(runtimeData.transforms[0].mRot.mValue, bone_position - runtimeData.transforms[0].mTrans.mValue);

			// Need trasform the coordinate relative to simulation rotation
			ResultLocation[0] = bone_position.x;
			ResultLocation[1] = bone_position.y;
			ResultLocation[2] = bone_position.z;

		}
	};

	struct RightFootPositionFeature :Feature {
		virtual int Size() const override
		{
			return 3; 
		}

		virtual void EvaluateForAnimationPose(
			float* ResultLocation,
			const AnimationDatabase& db,
			int poseIndex
		) const override
		{

			int RightFootId = 50; // Hard coding for now
			// 46 50
			Vector3 bone_position;
			Quaternion bone_rotation;

			std::vector<Transform> transform = db.GetTransformsAtPoseId(poseIndex);
			ForwardKinematics(
				bone_position,
				bone_rotation,
				transform,
				db.GetParentIndex(),
				RightFootId);

			bone_position = quat_inv_mul_vec3(transform[0].mRot.mValue, bone_position - transform[0].mTrans.mValue);

			// Need trasform the coordinate relative to simulation rotation
			ResultLocation[0] = bone_position.x;
			ResultLocation[1] = bone_position.y;
			ResultLocation[2] = bone_position.z;

		}

		virtual void EvaluateForRuntimeGuy(
			float* ResultLocation,
			const RuntimeCharacterData& runtimeData
		) const override
		{
			int leftFootId = 50; // Hard coding for now
			// 46 50
			Vector3 bone_position;
			Quaternion bone_rotation;

			ForwardKinematics(
				bone_position,
				bone_rotation,
				runtimeData.transforms,
				runtimeData.parent_index,
				leftFootId);

			bone_position = quat_inv_mul_vec3(runtimeData.transforms[0].mRot.mValue, bone_position - runtimeData.transforms[0].mTrans.mValue);

			// Need trasform the coordinate relative to simulation rotation
			ResultLocation[0] = bone_position.x;
			ResultLocation[1] = bone_position.y;
			ResultLocation[2] = bone_position.z;

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
			featureArray.features.push_back(&leftFootPositionFeature);
			featureArray.features.push_back(&rightFootPositionFeature);

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

				if (poseIndex + 10 > animDatabase->rangeStops[poseIndex])
					continue;

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

		std::vector<float> DenormalizeFeature(
			const std::vector<float>& query)
		{
			std::vector<float> denormalizedQuery;
			for (int i = 0; i < query.size(); i++)
			{
				denormalizedQuery.push_back(query[i] * featuresScale[i] + featuresOffset[i]);
			}
			return denormalizedQuery;
		}

		Array2D<float> matcherData;

		std::vector<float> featuresOffset;

		std::vector<float> featuresScale;

		AnimationDatabase* animDatabase;

		LeftFootPositionFeature leftFootPositionFeature;

		RightFootPositionFeature rightFootPositionFeature;

		TrajectoryPositionFeature trajectoryPositionFeature;

		TrajectoryDirectionFeature trajectoryDirectionFeature;

		FeatureArray featureArray;

		float bestCost = FLT_MAX;

		int bestIndex = -1;

		
	};
}


