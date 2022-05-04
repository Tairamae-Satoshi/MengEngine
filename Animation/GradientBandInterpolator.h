#pragma once
#include "Interpolator.h"

using namespace DirectX;
using namespace DirectX::SimpleMath;

namespace Animation 
{
	class PolarGradientBandInterpolator: Interpolator
	{
	public:
		PolarGradientBandInterpolator(const std::vector<Vector2>& points) : Interpolator(points)
		{
			minVx = std::numeric_limits<float>::max();
			maxVx = std::numeric_limits<float>::min();
			minVy = std::numeric_limits<float>::max();
			maxVy = std::numeric_limits<float>::min();
			for (const auto& point : points)
			{
				minVx = std::min(minVx, point.x);
				maxVx = std::max(maxVx, point.x);
				minVy = std::min(minVy, point.y);
				maxVy = std::max(maxVy, point.y);
			}
		}

		virtual std::vector<float> Interpolate(Vector2 samplePoint, bool isRatio = false)
		{
			/*if (isRatio)
			{
				samplePoint.x = (samplePoint.x < 0 ? minVx: maxVx) * abs(samplePoint.x);
				samplePoint.y = (samplePoint.y < 0 ? minVy : maxVy) * abs(samplePoint.y);
			}*/
			v = samplePoint;
			const float kDirScale = 2.0f;
			float total_weight = 0.0f;
			float sample_mag = samplePoint.Length();
			std::vector<float> weights;
			weights.resize(mPoints.size());

			for (int i = 0; i < mPoints.size(); ++i)
			{
				Vector2 point_i = mPoints[i];
				float point_mag_i = point_i.Length();
				float weight = 1.0;

				for (int j = 0; j < mPoints.size(); ++j)
				{
					if (j == i)
						continue;

					Vector2 point_j = mPoints[j];
					float point_mag_j = point_j.Length();
					
					float ij_avg_mag = (point_mag_i + point_mag_j) * 0.5;

					// Calc angle and mag for i -> sample
					float mag_is = (sample_mag - point_mag_i) / ij_avg_mag;
					float angle_is = signedAngle(mPoints[i], samplePoint);

					// Calc angle and mag for i -> j
					float mag_ij = (point_mag_j - point_mag_i) / ij_avg_mag;
					float angle_ij = signedAngle(mPoints[i], mPoints[j]);

					// Cal vec for i -> sample
					Vector2 vec_is;
					vec_is.x = mag_is;
					vec_is.y = angle_is * kDirScale;

					// Cal vec for i -> j
					Vector2 vec_ij;
					vec_ij.x = mag_ij;
					vec_ij.y = angle_ij * kDirScale;

					// Cal weight
					float lensq_ij = vec_ij.Dot(vec_ij);
					float new_weight = vec_is.Dot(vec_ij) / lensq_ij;

					new_weight = 1.0 - new_weight;
					new_weight = std::clamp(new_weight, 0.0f, 1.0f);

					weight = std::min(weight, new_weight);
				}
				weights[i] = weight;
				
				total_weight += weight;
			}

			for (int i = 0; i < mPoints.size(); ++i)
			{
				weights[i] = weights[i] / total_weight;
			}

			return weights;
		}

		Vector2 v;

		float minVx;

		float maxVx;

		float minVy;

		float maxVy;

	private:

		float signedAngle(Vector2 a, Vector2 b)
		{
			return atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
		}
	};
}

