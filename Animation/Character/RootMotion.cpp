#include "RootMotion.h"

namespace Animation
{
	RootMotion::RootMotion() :
		forward(Vector3::UnitZ),
		up(Vector3::UnitY),
		position(Vector3::Zero),
		rotaion(Quaternion::Identity),
		transform(Matrix::Identity),
		last_transform(Matrix::Identity) {}

	void RootMotion::ApplyRootTransform(const Quaternion& q, const Vector3& t, bool _transition)
	{
		//Matrix motion_root_transform = Matrix::CreateAffineTransformation(Vector3::One, t, q);
		if (_transition) {
			//last_transform = transform;
			last_position = position;
			last_rotaion = rotaion;

			motion_begin_position = t;
			motion_begin_rotaion = q;
		}
		else {
		}
		position = last_position + t - motion_begin_position;
		rotaion = q /** motion_begin_rotaion.Inversed() * last_rotaion*/;


		//position = transform.Translation();
		//rotaion = Quaternion::CreateFromRotationMatrix(transform); 
	}
}