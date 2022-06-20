#pragma once
#include "SimpleMath.h"

class GraphicDebug
{
public:
	struct Line
	{
		DirectX::SimpleMath::Vector3 p1;

		DirectX::SimpleMath::Vector3 p2;

		DirectX::SimpleMath::Vector4 color;
	};

	std::vector<Line> lines;

	void DrawLine(const DirectX::SimpleMath::Vector3& p1, const DirectX::SimpleMath::Vector3& dir, float length, const DirectX::SimpleMath::Vector4& color);

	void DrawLine(const DirectX::SimpleMath::Vector3& p1, const DirectX::SimpleMath::Vector3& p2, const DirectX::SimpleMath::Vector4& color);

	void Update();
};