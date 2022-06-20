#include "GraphicDebug.h"
#include "SimpleMath.h"
using namespace DirectX::SimpleMath;

void GraphicDebug::DrawLine(const DirectX::SimpleMath::Vector3& p1, const DirectX::SimpleMath::Vector3& dir, float length, const DirectX::SimpleMath::Vector4& color){
	Line l;

	l.p1 = p1;
	l.p2 = p1 + dir * length;
	l.color = color;

	lines.push_back(std::move(l));
}

void GraphicDebug::DrawLine(const DirectX::SimpleMath::Vector3& p1, const DirectX::SimpleMath::Vector3& p2, const DirectX::SimpleMath::Vector4& color)
{
	Line l;

	l.p1 = p1;
	l.p2 = p2;
	l.color = color;

	lines.push_back(std::move(l));
}

void GraphicDebug::Update()
{
	lines.clear();
}