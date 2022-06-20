// Include common HLSL code.
// Defaults for number of lights.
#ifndef NUM_DIR_LIGHTS
#define NUM_DIR_LIGHTS 1
#endif

#ifndef NUM_POINT_LIGHTS
#define NUM_POINT_LIGHTS 0
#endif

#ifndef NUM_SPOT_LIGHTS
#define NUM_SPOT_LIGHTS 0
#endif

// Shadow map related variables
#define NUM_SAMPLES 16
#define BLOCKER_SEARCH_NUM_SAMPLES NUM_SAMPLES
#define PCF_NUM_SAMPLES NUM_SAMPLES
#define NUM_RINGS 3

#define EPS 1e-3
#define PI 3.141592653589793
#define PI2 6.283185307179586

#define LIGHT_SIZE_UV 0.02 //  LIGHT_WORLD_SIZE / LIGHT_FRUSTUM_WIDTH = 1.0 / 50 = 0.02
#define NEAR_PLANE 0.1 // 1 / 100

// Include structures and functions for lighting.
#include "LightingUtil.hlsl"

// Constant data that varies per frame.
cbuffer cbPerObject
{
	float4x4 gWorld;
	float4x4 gObject;
	float4 gColor;
};

// Constant data that varies per material.
cbuffer cbPass
{
	float4x4 gView;
	float4x4 gInvView;
	float4x4 gProj;
	float4x4 gInvProj;
	float4x4 gViewProj;
	float4x4 gInvViewProj;
	float4x4 gViewProjTex;
	float4x4 gShadowTransform;
	float3 gEyePosW;
	float cbPerObjectPad1;
	float2 gRenderTargetSize;
	float2 gInvRenderTargetSize;
	float gNearZ;
	float gFarZ;
	float gTotalTime;
	float gDeltaTime;
};

// Static
cbuffer cbLight
{
	// Indices [0, NUM_DIR_LIGHTS) are directional lights;
	// indices [NUM_DIR_LIGHTS, NUM_DIR_LIGHTS+NUM_POINT_LIGHTS) are point lights;
	// indices [NUM_DIR_LIGHTS+NUM_POINT_LIGHTS, NUM_DIR_LIGHTS+NUM_POINT_LIGHT+NUM_SPOT_LIGHTS)
	// are spot lights for a maximum of MaxLights per object.

	float4 gAmbientLight;
	Light gLights[MaxLights];
}

cbuffer cbSkinned
{
	float4x4 gBoneTransforms[96];
};

cbuffer cbMaterial
{
	float4 BaseColorFactor;
	float4 EmissiveFactor;
	float  MetallicFactor;
	float  RoughnessFactor;
}

Texture2D BaseColorTex;
Texture2D MetallicRoughnessTex;
Texture2D EmissiveTex;

Texture2D ShadowMap;

SamplerState gsamPointWrap        : register(s0);
SamplerState gsamPointClamp       : register(s1);
SamplerState gsamLinearWrap       : register(s2);
SamplerState gsamLinearClamp      : register(s3);
SamplerState gsamAnisotropicWrap  : register(s4);
SamplerState gsamAnisotropicClamp : register(s5);
SamplerComparisonState gsamShadow : register(s6);

cbuffer POISSON_DISKS
{
	static float2 poissonDisk[16] = {
	 float2(-0.94201624, -0.39906216),
	 float2(0.94558609, -0.76890725),
	 float2(-0.094184101, -0.92938870),
	 float2(0.34495938, 0.29387760),
	 float2(-0.91588581, 0.45771432),
	 float2(-0.81544232, -0.87912464),
	 float2(-0.38277543, 0.27676845),
	 float2(0.97484398, 0.75648379),
	 float2(0.44323325, -0.97511554),
	 float2(0.53742981, -0.47373420),
	 float2(-0.26496911, -0.41893023),
	 float2(0.79197514, 0.19090188),
	 float2(-0.24188840, 0.99706507),
	 float2(-0.81409955, 0.91437590),
	 float2(0.19984126, 0.78641367),
	 float2(0.14383161, -0.14100790)
	};
};

//---------------------------------------------------------------------------------------
// Transforms a normal map sample to world space.
//---------------------------------------------------------------------------------------
float3 NormalSampleToWorldSpace(float3 normalMapSample, float3 unitNormalW, float3 tangentW)
{
	// Uncompress each component from [0,1] to [-1,1].
	float3 normalT = 2.0f*normalMapSample - 1.0f;

	// Build orthonormal basis.
	float3 N = unitNormalW;
	float3 T = normalize(tangentW - dot(tangentW, N)*N);
	float3 B = cross(N, T);

	float3x3 TBN = float3x3(T, B, N);

	// Transform from tangent space to world space.
	float3 bumpedNormalW = mul(normalT, TBN);

	return bumpedNormalW;
}

//---------------------------------------------------------------------------------------
// PCF for shadow mapping.
//---------------------------------------------------------------------------------------
//#define SMAP_SIZE = (2048.0f)
//#define SMAP_DX = (1.0f / SMAP_SIZE)
float PCF(Texture2D shadowMap, float4 shadowPosH)
{
	// Complete projection by doing division by w.
	shadowPosH.xyz /= shadowPosH.w;

	// Depth in NDC space.
	float depth = shadowPosH.z;

	uint width, height, numMips;
	shadowMap.GetDimensions(0, width, height, numMips);

	// Texel size.
	float dx = 1.0f / (float)width;

	float percentLit = 0.0f;
	const float2 offsets[9] =
	{
		float2(-dx,  -dx), float2(0.0f,  -dx), float2(dx,  -dx),
		float2(-dx, 0.0f), float2(0.0f, 0.0f), float2(dx, 0.0f),
		float2(-dx,  +dx), float2(0.0f,  +dx), float2(dx,  +dx)
	};

	[unroll]
	for (int i = 0; i < 9; ++i)
	{
		percentLit += shadowMap.SampleCmpLevelZero(gsamShadow,
			shadowPosH.xy + offsets[i], depth).r;
	}

	return percentLit / 9.0f;
}

//---------------------------------------------------------------------------------------
// Generate random from 0.0 to 1.0
//---------------------------------------------------------------------------------------
float rand_0to1(const float2 p)
{
	float2 K1 = float2(
		23.14069263277926, // e^pi (Gelfond's constant)
		2.665144142690225 // 2^sqrt(2) (Gelfond???Schneider constant)
		);

	return frac(cos(dot(p, K1)) * 12345.6789);
}

//---------------------------------------------------------------------------------------
// Generate poisson disk samples
//---------------------------------------------------------------------------------------
void PoissonDiskSamples(const float2 randomSeed)
{
	float ANGLE_STEP = PI2 * float(NUM_RINGS) / float(NUM_SAMPLES);
	float INV_NUM_SAMPLES = 1.0 / float(NUM_SAMPLES);

	float angle = rand_0to1(randomSeed) * PI2;
	float radius = INV_NUM_SAMPLES;
	float radiusStep = radius;

	[unroll]
	for (int i = 0; i < NUM_SAMPLES; ++i) {
		poissonDisk[i] = float2(cos(angle), sin(angle)) * pow(radius, 0.75);
		radius += radiusStep;
		angle += ANGLE_STEP;
	}
}

//---------------------------------------------------------------------------------------
// Find blocker in PCSS
//---------------------------------------------------------------------------------------
void FindBlocker(out float avgBlockerDepth,
	out float numBlockers,
	Texture2D shadowMap, float2 uv, float zReceiver)
{
	//This uses similar triangles to compute what
	//area of the shadow map we should search
	float searchWidth = LIGHT_SIZE_UV * (zReceiver - NEAR_PLANE) / zReceiver;
	float blockerSum = 0;
	numBlockers = 0;

	for (int i = 0; i < BLOCKER_SEARCH_NUM_SAMPLES; ++i)
	{
		float shadowMapDepth = shadowMap.SampleLevel(
			gsamPointWrap,
			uv + poissonDisk[i] * searchWidth,
			0).r;
		if (shadowMapDepth < zReceiver) {
			blockerSum += shadowMapDepth;
			numBlockers++;
		}
	}
	avgBlockerDepth = blockerSum / numBlockers;
}

//---------------------------------------------------------------------------------------
// Compute penumbra size
//---------------------------------------------------------------------------------------
float PenumbraSize(float zReceiver, float zBlocker)
{
	return (zReceiver - zBlocker) / zBlocker;
}

//---------------------------------------------------------------------------------------
// PCSS
//---------------------------------------------------------------------------------------
float PCSS(Texture2D shadowMap, float4 shadowPosH)
{
	// Complete projection by doing division by w.
	shadowPosH.xyz /= shadowPosH.w;
	PoissonDiskSamples(shadowPosH.xy);

	// STEP 1: avgblocker depth
	float avgBlockerDepth = 0.0f;
	float numBlockers = 0;
	FindBlocker(avgBlockerDepth, numBlockers, shadowMap, shadowPosH.xy, shadowPosH.z);
	if (numBlockers < 1)	//There are no occluders so early out (this saves filtering)
		return 1.0f;

	// STEP 2: penumbra size
	float filterRadiusUV = LIGHT_SIZE_UV * PenumbraSize(shadowPosH.z, avgBlockerDepth);

	// STEP 3: filtering
	float percentLit = 0.0f;

	[unroll]
	for (int i = 0; i < PCF_NUM_SAMPLES; ++i)
	{
		percentLit += shadowMap.SampleCmpLevelZero(gsamShadow,
			shadowPosH.xy + poissonDisk[i] * filterRadiusUV, shadowPosH.z).r;
	}

	//return shadowMap.SampleCmpLevelZero(gsamShadow,
		//shadowPosH.xy, shadowPosH.z).r;
	return percentLit / float(PCF_NUM_SAMPLES);
}
//---------------------------------------------------------------------------------------

struct VertexIn
{
	float3 PosL    : POSITION;
	float3 NormalL : NORMAL;
	float3 TangentL : TANGENT;
	float2 TexC    : TEXCOORD;
#ifdef SKINNED
	float3 BoneWeights : WEIGHTS;
	uint4 BoneIndices  : BONEINDICES;
#endif
};

struct VertexOut
{
	float4 PosH    : SV_POSITION;
	float4 ShadowPosH : POSITION0;
	float3 PosW    : POSITION1;
	float3 NormalW : NORMAL;
	float3 TangentW : TANGENT;
	float2 TexC    : TEXCOORD;
	float4 Color : POSITION2; // TODO
};

VertexOut VS(VertexIn vin)
{
	VertexOut vout = (VertexOut)0.0f;

#ifdef SKINNED
	float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	weights[0] = vin.BoneWeights.x;
	weights[1] = vin.BoneWeights.y;
	weights[2] = vin.BoneWeights.z;
	weights[3] = 1.0f - weights[0] - weights[1] - weights[2];

	float3 posL = float3(0.0f, 0.0f, 0.0f);
	float3 normalL = float3(0.0f, 0.0f, 0.0f);
	float3 tangentL = float3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < 4; ++i)
	{
		// Assume no nonuniform scaling when transforming normals, so 
		// that we do not have to use the inverse-transpose.

		posL += weights[i] * mul(float4(vin.PosL, 1.0f), gBoneTransforms[vin.BoneIndices[i]]).xyz;
		normalL += weights[i] * mul(vin.NormalL, (float3x3)gBoneTransforms[vin.BoneIndices[i]]);
		tangentL += weights[i] * mul(vin.TangentL.xyz, (float3x3)gBoneTransforms[vin.BoneIndices[i]]);
	}

	vin.PosL = posL;
	vin.NormalL = normalL;
	vin.TangentL.xyz = tangentL;
#endif

	// Transform to world space.
	float4 posW = mul(float4(vin.PosL, 1.0f), gWorld);
	vout.PosW = posW.xyz;
	vout.PosH = mul(posW, gViewProj);
	
	vout.NormalW = normalize(mul((float3x3)gObject, vin.NormalL));
	vout.TangentW = mul(vin.TangentL, (float3x3)gWorld);
	vout.TexC = vin.TexC;
	vout.Color = gColor;

	// Generate projective tex-coords to project shadow map onto scene.
	vout.ShadowPosH = mul(posW, gShadowTransform);

	return vout;
}


float4 PS(VertexOut pin) : SV_Target
{
	// Fetch the material data.
	float4	diffuseAlbedo = pin.Color/*float4(0.75f, 0.75f, 0.75f, 1.0f)*/;
#ifdef CHECKBOARD
	float total = floor(pin.PosW.x * 0.5f) + floor(pin.PosW.z * 0.5f);
	diffuseAlbedo = (total % 2.0f) == 0.0f ?
		float4(0.75f, 0.75f, 0.75f, 1.0f):
		float4(0.85f, 0.85f, 0.85f, 1.0f);
	float halfWidth = 0.1f;
	if (pin.PosW.x > 0 && pin.PosW.z > -halfWidth && pin.PosW.z <= halfWidth) diffuseAlbedo = float4(1.0f, 0.0f, 0.0f, 1.0f);
	if (pin.PosW.z > 0 && pin.PosW.x > -halfWidth && pin.PosW.x <= halfWidth) diffuseAlbedo = float4(0.0f, 0.0f, 1.0f, 1.0f);
	if (pin.PosW.x > -halfWidth && pin.PosW.x <= halfWidth && pin.PosW.z > -halfWidth && pin.PosW.z <= halfWidth) diffuseAlbedo = float4(0.0f, 1.0f, 0.0f, 1.0f);

#endif
	float4 metallic = MetallicFactor;
	float  roughness = 0.5;
	float3 fresnelR0 = float3(0.05f, 0.05f, 0.05f);

	// Interpolating normal can unnormalize it, so renormalize it.
	pin.NormalW = normalize(pin.NormalW);

	// Vector from point being lit to eye. 
	float3 toEyeW = normalize(float3(1.0f, 1.0f, 1.0f) - pin.PosW);

	// TODO: Shadow map
	// Only the first light casts a shadow.
	float3 shadowFactor = float3(1.0f, 1.0f, 1.0f);
	shadowFactor[0] = PCF(ShadowMap, pin.ShadowPosH);

	// Light terms.
	float4 ambient = gAmbientLight * diffuseAlbedo;

	const float shininess = (1.0f - roughness);
	Material mat = { diffuseAlbedo, fresnelR0, shininess };
	float4 directLight = ComputeLighting(gLights, mat, pin.PosW, pin.NormalW, toEyeW, shadowFactor);

	//float4 litColor = float4(1.0f, 1.0f, 1.0f, 1.0f);
	float4 litColor = ambient + directLight;

	// Common convention to take alpha from diffuse albedo.
	litColor.a = diffuseAlbedo.a;

	return litColor;
}


