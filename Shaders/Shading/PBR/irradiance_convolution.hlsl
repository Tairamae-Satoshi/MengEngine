// Include common HLSL code.
// Defaults for number of lights.
#ifndef NUM_DIR_LIGHTS
#define NUM_DIR_LIGHTS 0
#endif

#ifndef NUM_POINT_LIGHTS
#define NUM_POINT_LIGHTS 4
#endif

#ifndef NUM_SPOT_LIGHTS
#define NUM_SPOT_LIGHTS 0
#endif

#include "Common.hlsl"

#define PI 3.141592653589793

struct VertexOut
{
	float4 PosH : SV_POSITION;
	float3 PosL : POSITION;
};


float4 PS(VertexOut pin) : SV_Target
{
	// The world vector acts as the normal of a tangent surface
	// from the origin, aligned to WorldPos. Given this normal, calculate all
	// incoming radiance of the environment. The result of this radiance
	// is the radiance of light coming from -Normal direction, which is what
	// we use in the PBR shader to sample irradiance.
	float3 N = normalize(pin.PosL);

	float3 irradiance = float3(0.0, 0.0, 0.0);
	
	// tangent space calculation from origin point
	float3 up = float3(0.0, 1.0, 0.0);
	float3 right = normalize(cross(up, N));
	float3 up = normalize(cross(N, right));

	float sampleDelta = 0.025;
	float nrSamples = 0.0;
	for (float phi = 0.0; phi < 2.0 * PI; phi += sampleDelta)
	{
		for (float theta = 0.0; theta < 0.5 * PI; theta += sampleDelta)
		{
			// spherical to cartesian (in tangent space)
			float3 tangentSample = float3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
			// tangent space to world
			float3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * N;

			irradianc += 
		}
	}


}


