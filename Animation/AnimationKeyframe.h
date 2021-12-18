#pragma once

///<summary>
/// A Keyframe defines the bone transformation at an instant in time.
///</summary>

#include "../pch.h"

struct VectorKey
{
	DirectX::SimpleMath::Vector3 mValue = DirectX::SimpleMath::Vector3(0.0f, 0.0f, 0.0f);
};

struct QuatKey
{
	DirectX::SimpleMath::Quaternion mValue = DirectX::SimpleMath::Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
};
