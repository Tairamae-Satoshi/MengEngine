//***************************************************************************************
// LightingUtil.hlsl by Frank Luna (C) 2015 All Rights Reserved.
//
// Contains API for shader lighting.
//***************************************************************************************

#define PI 3.141592653589793

//---------------------------------------------------------------------------------------
// Compute the distribution GGX (D = a^2 / PI * ((n dot h)^2 * (a^2 - 1) + 1)^2)
//---------------------------------------------------------------------------------------
float DistributionGGX(float3 N, float3 H, float roughness)
{
	float a = roughness * roughness;
	float a2 = a * a;
	float NdotH = max(dot(N, H), 0.0);
	float NdotH2 = NdotH * NdotH;

	float nom = a2;
	float denom = (NdotH2 * (a2 - 1.0) + 1.0);
	denom = PI * denom * denom;

	return nom / denom;
}

//---------------------------------------------------------------------------------------
// Compute the Geometry Schlick GGX (G_sub = (n dot v) / ((n dot v)(1 - k) + k))
//---------------------------------------------------------------------------------------
float GeometrySchlickGGX(float NdotV, float roughness)
{
	float r = (roughness + 1.0);
	float k = (r * r) / 8.0;

	float nom = NdotV;
	float denom = NdotV * (1.0 - k) + k;

	return nom / denom;
}

//---------------------------------------------------------------------------------------
// Compute the Geometry Smith (G(n, v, l, k) = G_sub(n, v, k) * G_sub(n, l, k))
//---------------------------------------------------------------------------------------
float GeometrySmith(float3 N, float3 V, float3 L, float roughness)
{
	float NdotV = max(dot(N, V), 0.0);
	float NdotL = max(dot(N, L), 0.0);
	float ggx1 = GeometrySchlickGGX(NdotV, roughness);
	float ggx2 = GeometrySchlickGGX(NdotL, roughness);

	return ggx1 * ggx2;
}

//---------------------------------------------------------------------------------------
// Compute the fresnel schlick (F = F_0 + (1 - F_0)(1 - (h dot v)^5))
//---------------------------------------------------------------------------------------
float3 FresnelSchlick(float cosTheta, float3 F0)
{
	return F0 + (1 - F0) * pow(1.0 - cosTheta, 5.0);
}
