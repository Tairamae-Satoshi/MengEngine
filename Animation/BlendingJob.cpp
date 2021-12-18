#include "../pch.h"
#include "BlendingJob.h"
#include "SamplingJob.h"
#include "GradientBandInterpolator.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace Animation
{
	BlendingJob::Layer::Layer() :
		weight(1.0f), 
		t(0.0f)
	{}

	bool BlendingJob::Run()
	{
		Validate();
		UINT jointNum = layers[0].animation->mSamples.size();

		// Increment timewarping
		float sumOfWD = 0.0f;
		for (auto& layer : layers)
		{
			layer.m = floor(t * (layer.Nk - 1));
			float derivative = static_cast<float>(layer.Nk - 1) * (layer.K[layer.m + 1] - layer.K[layer.m]);
			sumOfWD += layer.weight * derivative;
		}
		deltat = deltaT / sumOfWD;
		float lastt = t;
		t += deltat;
		t = t - floor(t);
		transition = lastt > t ? true : false;

		// If move into next segment
		if (floor(t * (layers[0].Nk - 1))!= layers[0].m)
		{
			float deltat1 = 1.0f / static_cast<float>(layers[0].Nk - 1) * (layers[0].m + 1) - lastt;
			float deltaT1 = sumOfWD * deltat1;
			float deltaT2 = deltaT - deltaT1;
			float sumOfWD2 = 0.0f;

			for (auto& layer : layers)
			{
				int nextm = (layer.m + 1) % (layer.Nk - 1);
				float derivative = (layer.Nk - 1) * (layer.K[nextm + 1] - layer.K[nextm]);
				sumOfWD2 += layer.weight * derivative;
			}
			float deltat2 = deltaT2 / sumOfWD2;
			t = lastt + deltat1 + deltat2;
			t = t - floor(t);
		}

		for (auto& layer : layers){
			layer.m = floor(t * (layer.Nk - 1));
			layer.T = layer.K[layer.m] + (((layer.Nk - 1) * t) - layer.m) * (layer.K[layer.m + 1] - layer.K[layer.m]);
		}

		output.clear();
		output.resize(jointNum);

		std::vector<std::vector<Quaternion>> qs;
		qs.resize(jointNum);

		int numPass = 0;
		for (const auto& layer : layers)
		{
			if (layer.weight <= 0.0f){
				continue;
			}

			float weight = layer.weight;

			SamplingJob samplingJob;
			samplingJob.animation = layer.animation;
			samplingJob.ratio = layer.T / layer.animation->GetDuration();
			samplingJob.Run();

			const std::vector<Transform>& transforms = samplingJob.output;
			for (UINT i = 0; i < jointNum; i++)
			{
				qs[i].push_back(transforms[i].mRot.mValue);		
				
				if (numPass == 0){
					output[i].mRot.mValue = weight * transforms[i].mRot.mValue;
					output[i].mScale.mValue = weight * transforms[i].mScale.mValue;
					output[i].mTrans.mValue = weight * transforms[i].mTrans.mValue;
				}
				else{
					output[i].mRot.mValue += weight * transforms[i].mRot.mValue;
					output[i].mScale.mValue += weight * transforms[i].mScale.mValue;
					output[i].mTrans.mValue += weight * transforms[i].mTrans.mValue;
				}
			}
			numPass++;
		}

		// Log Quaternion Blending
		for (int i = 0; i < jointNum; ++i)
		{
			numPass = 0;
			Quaternion q_ref = CalculateReferenceQ(qs[i]);
			Vector4 v_avg = Vector4::Zero;
			for (int j = 0; j < layers.size(); ++j)
			{
				if (layers[j].weight <= 0)
					continue;

				Quaternion q_ref_inv_x_qj = q_ref.Inversed() * qs[i][numPass];

				v_avg += layers[j].weight * q_ref_inv_x_qj.Ln();				

				numPass++;
			}
			output[i].mRot.mValue = q_ref * Quaternion::Exp(v_avg);
		}

		// Normalize quaternion
		for (UINT i = 0; i < jointNum; i++)
		{
			output[i].mRot.mValue.Normalize();
		}

		return true;
	}

	bool BlendingJob::Validate()
	{
		float weightSum = 0.0f;
		for (const auto& layer : layers)
		{
			weightSum += layer.weight;
		}

		if (weightSum != 1.0f)
		{
			for (auto& layer : layers)
			{
				layer.weight /= weightSum;
			}
		}

		return true;
	}

	Quaternion BlendingJob::CalculateReferenceQ(const std::vector<Quaternion>& qs)
	{
		/*Eigen::Matrix4f sumOfqxqT = Eigen::Matrix4f::Zero();
		for (const auto& q : qs)
		{
			Eigen::Vector4f qi(q.x, q.y, q.z, q.w);
			sumOfqxqT += qi * qi.transpose();
		}

		Eigen::EigenSolver<Eigen::Matrix4f> solver(sumOfqxqT);
		Eigen::Matrix4f V = solver.eigenvectors().real();

		float minE = std::numeric_limits<float>::max();
		Quaternion q_ref;
		for (size_t i = 0; i < 4; i++)
		{
			float E = 0.0f;
			Quaternion q_eigen(V.col(i)[0], V.col(i)[1], V.col(i)[2], V.col(i)[3]);
			for (const auto& qi : qs) {
				float dot = qi.Dot(q_eigen);
				E += 1.0f - dot * dot;
			}
			if (E < minE)
			{
				minE = E;
				q_ref = q_eigen;
			}
		}*/
		Quaternion q_ref;

		int num_pass = 0;
		for (auto& qi : qs)
		{
			if (num_pass == 0)
			{
				q_ref = qi;
			}
			else
			{
				q_ref += qi;
			}
			num_pass++;
		}
		q_ref.Normalize();

		// Redirect the q_ref
		//float sum = 0.0f;
		//for (auto qi : qs)
		//{
		//	sum += qi.Dot(q_ref);
		//}
		//q_ref = sum >= 0 ? q_ref : -q_ref;

		return q_ref;
	}

	float BlendingJob::GetDuration()
	{
		float duration = 0.0f;
		for (const auto& layer : layers)
		{
			duration += layer.animation->GetDuration() * layer.weight;
		}
		return duration;
	}


	bool BlendingJob::OnGui()
	{
		char lable[64];
		static float k[2] = { 0, 0 };

		for (size_t i = 0; i < layers.size(); i++)
		{
			//std::sprintf(lable, "%s", mLayers[i].animation->mName);
			float w = layers[i].weight;
			if (ImGui::SliderFloat(layers[i].animation->mName.data(), &w, 0.0f, 1.0f))
			{
				//mLayers[1 - i].weight = 1.0f - mLayers[i].weight;
				return true;
			}
		}

		//weight_transition = false;
		float v[2] = { interpolator->v.x, interpolator->v.y };
		if (ImGui::SliderFloat("velocity_x", &v[0], interpolator->minVx, interpolator->maxVx)|| 
			ImGui::SliderFloat("velocity_y", &v[1], interpolator->minVy, interpolator->maxVy))
		{
			std::vector<float> weights = interpolator->Interpolate(Vector2(v[0], v[1]));
			for (int i = 0; i < layers.size(); ++i)
			{
				layers[i].weight = weights[i];
			}
			//weight_transition = true;
		}

		return false;
	}
}