#pragma once

#include "../pch.h"
#include "Common.h"

using namespace DirectX::SimpleMath;

//--------------------------------------
static inline Quaternion quat_from_scaled_angle_axis(Vector3 v, float eps = 1e-8f)
{
	v = v * 0.5f;
	float halfangle = v.Length();

	if (halfangle < eps)
	{
		Quaternion result(v.x, v.y, v.z, 1.0f);
		result.Normalize();
		return result;
	}
	else
	{
		float c = cosf(halfangle);
		float s = sinf(halfangle) / halfangle;
		return Quaternion(s * v.x, s * v.y, s * v.z, c);
	}
}

static inline Vector3 quat_to_scaled_angle_axis(Quaternion q, float eps = 1e-8f)
{
	float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);

	if (length < eps)
	{
		return 2.0f * Vector3(q.x, q.y, q.z);
	}
	else
	{
		float halfangle = acosf(clampf(q.w, -1.0f, 1.0f));
		return 2.0f * halfangle * (Vector3(q.x, q.y, q.z) / length);
	}

}

static inline Quaternion quat_slerp_shortest_approx(Quaternion q, Quaternion p, float alpha)
{
	float ca = q.Dot(p);

	if (ca < 0.0f)
	{
		p = -p;
	}

	float d = fabsf(ca);
	float a = 1.0904f + d * (-3.2452f + d * (3.55645f - d * 1.43519f));
	float b = 0.848013f + d * (-1.06021f + d * 0.215638f);
	float k = a * (alpha - 0.5f) * (alpha - 0.5f) + b;
	float oalpha = alpha + alpha * (alpha - 0.5f) * (alpha - 1) * k;
	
	return Quaternion::Lerp(q, p, oalpha);
}

static inline Vector3 quat_mul_vec3(Quaternion q, Vector3 v)
{
	Vector3 t = 2.0f * Vector3(q.x, q.y, q.z).Cross(v);
	return v + q.w * t + Vector3(q.x, q.y, q.z).Cross(t);
}

static inline Vector3 quat_inv_mul_vec3(const Quaternion& q, const Vector3& v)
{
	return quat_mul_vec3(q.Inversed(), v);
}

static inline Quaternion quat_abs(Quaternion x)
{
	return x.w < 0.0 ? -x : x;
}


static inline float damper_implicit(float x, float g, float halflife, float dt, float eps = 1e-5f)
{
	return lerpf(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

static inline Vector3 damper_implicit(Vector3 x, Vector3 g, float halflife, float dt, float eps = 1e-5f)
{
	return 	Vector3::Lerp(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

static inline Quaternion damper_implicit(Quaternion x, Quaternion g, float halflife, float dt, float eps = 1e-5f)
{
	return quat_slerp_shortest_approx(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

static inline float damp_adjustment_implicit(float g, float halflife, float dt, float eps = 1e-5f)
{
	return g * (1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

static inline Vector3 damp_adjustment_implicit(Vector3 g, float halflife, float dt, float eps = 1e-5f)
{
	return g * (1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

static inline Quaternion damp_adjustment_implicit(Quaternion g, float halflife, float dt, float eps = 1e-5f)
{
	return quat_slerp_shortest_approx(Quaternion(), g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
}

//--------------------------------------

static inline float halflife_to_damping(float halflife, float eps = 1e-5f)
{
	return (4.0f * LN2f) / (halflife + eps);
}

static inline float damping_to_halflife(float damping, float eps = 1e-5f)
{
	return (4.0f * LN2f) / (damping + eps);
}

static inline float frequency_to_stiffness(float frequency)
{
	return squaref(2.0f * PIf * frequency);
}

static inline float stiffness_to_frequency(float stiffness)
{
	return sqrtf(stiffness) / (2.0f * PIf);
}

//--------------------------------------

static inline void simple_spring_damper_implicit(
	float& x,
	float& v,
	const float x_goal,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;
	float j0 = x - x_goal;
	float j1 = v + j0 * y;
	float eydt = fast_negexpf(y * dt);

	x = eydt * (j0 + j1 * dt) + x_goal;
	v = eydt * (v - j1 * y * dt);
}

static inline void simple_spring_damper_implicit(
	Vector3& x,
	Vector3& v,
	const Vector3 x_goal,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;
	Vector3 j0 = x - x_goal;
	Vector3 j1 = v + j0 * y;
	float eydt = fast_negexpf(y * dt);

	x = eydt * (j0 + j1 * dt) + x_goal;
	v = eydt * (v - j1 * y * dt);
}

static inline void simple_spring_damper_implicit(
	Quaternion& x,
	Vector3& v,
	const Quaternion& x_goal,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;

	Vector3 j0 = quat_to_scaled_angle_axis(quat_abs(x * x_goal.Inversed()));
	Vector3 j1 = v + j0 * y;

	float eydt = fast_negexpf(y * dt);

	x = quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * x_goal;
	v = eydt * (v - j1 * y * dt);
}

//--------------------------------------

static inline void decay_spring_damper_implicit(
	float& x,
	float& v,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;
	float j1 = v + x * y;
	float eydt = fast_negexpf(y * dt);

	x = eydt * (x + j1 * dt);
	v = eydt * (v - j1 * y * dt);
}

static inline void decay_spring_damper_implicit(
	Vector3& x,
	Vector3& v,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;
	Vector3 j1 = v + x * y;
	float eydt = fast_negexpf(y * dt);

	x = eydt * (x + j1 * dt);
	v = eydt * (v - j1 * y * dt);
}

static inline void decay_spring_damper_implicit(
	Quaternion& x,
	Vector3& v,
	const float halflife,
	const float dt)
{
	float y = halflife_to_damping(halflife) / 2.0f;

	Vector3 j0 = quat_to_scaled_angle_axis(x);
	Vector3 j1 = v + j0 * y;

	float eydt = fast_negexpf(y * dt);

	x = quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt));
	v = eydt * (v - j1 * y * dt);
}

//--------------------------------------

static inline void inertialize_transition(
	Vector3& off_x,
	Vector3& off_v,
	const Vector3 src_x,
	const Vector3 src_v,
	const Vector3 dst_x,
	const Vector3 dst_v)
{
	off_x = (src_x + off_x) - dst_x;
	off_v = (src_v + off_v) - dst_v;
}

static inline void inertialize_update(
	Vector3& out_x,
	Vector3& out_v,
	Vector3& off_x,
	Vector3& off_v,
	const Vector3 in_x,
	const Vector3 in_v,
	const float halflife,
	const float dt)
{
	decay_spring_damper_implicit(off_x, off_v, halflife, dt);
	out_x = in_x + off_x;
	out_v = in_v + off_v;
}

//static inline void inertialize_transition(
//	Quaternion& off_x,
//	Vector3& off_v,
//	const Quaternion src_x,
//	const Vector3 src_v,
//	const Quaternion dst_x,
//	const Vector3 dst_v)
//{
//	off_x = quat_abs(Quaternion_mul(Quaternion_mul(off_x, src_x), Quaternion_inv(dst_x)));
//	off_v = (off_v + src_v) - dst_v;
//}
//
//static inline void inertialize_update(
//	Quaternion& out_x,
//	Vector3& out_v,
//	Quaternion& off_x,
//	Vector3& off_v,
//	const Quaternion in_x,
//	const Vector3 in_v,
//	const float halflife,
//	const float dt)
//{
//	decay_spring_damper_implicit(off_x, off_v, halflife, dt);
//	out_x = Quaternion_mul(off_x, in_x);
//	out_v = off_v + in_v;
//}
