#include "pch.h"
#include "GameObject.h"
#include "MeshRenderer.h"

using namespace std;
using namespace DirectX;

GameObject::GameObject(GameObject* parent) :
	m_Parent(parent)
{
}

GameObject::~GameObject()
{

}

GameObject* GameObject::CreateChild()
{
	GameObject* child = new GameObject(this);
	m_Children.push_back(unique_ptr<GameObject>(child));
	return child;
}

void GameObject::DestroyChildren()
{
}

void GameObject::Translate(float x, float y, float z, Space relativeTo)
{
	m_TransformDirty = true;

	if (relativeTo == Space::Self)
	{
		m_Position += (m_Rotation * Vector3(x, y, z));
	}
	else if (relativeTo == Space::World)
	{
		_UpdateFromParent();

		if (m_Parent)
		{
			m_Position += Vector3::Transform(Vector3(x, y, z), m_Parent->m_DerivedRotation) / m_Parent->m_DerivedScale;
		}
		else
		{
			m_Position += Vector3(x, y, z);
		}
	}

	OnTransformUpdate(this);
	for (auto& child : m_Children)
	{
		SetChildTransformDirty(child.get());
	}
}

void GameObject::Rotate(float xAngle, float yAngle, float zAngle, Space relativeTo)
{
	m_TransformDirty = true;

	if (relativeTo == Space::Self)
	{
		m_Rotation = m_Rotation * Quaternion::CreateFromYawPitchRoll(XMConvertToRadians(xAngle), XMConvertToRadians(yAngle), XMConvertToRadians(zAngle));
	}
	else if (relativeTo == Space::World)
	{
		_UpdateFromParent();

		m_Rotation = m_Rotation * m_DerivedRotation * Quaternion::CreateFromYawPitchRoll(XMConvertToRadians(xAngle), XMConvertToRadians(yAngle), XMConvertToRadians(zAngle)) * m_DerivedRotation;
	}

	OnTransformUpdate(this);
	for (auto& child : m_Children)
	{
		SetChildTransformDirty(child.get());
	}
}

DirectX::XMFLOAT4X4 GameObject::LocalToWorldMatrix()
{
	_UpdateFromParent();

	return m_LocalToWorldMatrix;
}

void GameObject::_UpdateFromParent()
{
	if (m_TransformDirty)
	{
		m_TransformDirty = false;

		Vector3 parentScale(1.0f, 1.0f, 1.0f);
		Quaternion parentRotation;
		Vector3 parentPosition(0.0f, 0.0f, 0.0f);

		if (m_Parent)
		{
			m_Parent->_UpdateFromParent();
			parentScale = m_Parent->m_DerivedScale;
			parentRotation = m_Parent->m_DerivedRotation;
			parentPosition = m_Parent->m_DerivedPosition;
		}

		m_DerivedScale = parentScale * m_Scale;
		m_DerivedRotation = parentRotation * m_Rotation;
		// �ӽڵ��Position�����ڵ��Scale��Rotation�й�
		m_DerivedPosition = parentRotation * (parentScale * m_Position);
		m_DerivedPosition += parentPosition;

		// TODO:�Ż���ȥ����������ĳ˷���ֻ��Ҫ����Ԫ������һ������
		XMMATRIX scale = XMMatrixScalingFromVector(m_DerivedScale);
		XMMATRIX rotate = XMMatrixRotationQuaternion(m_DerivedRotation);
		XMMATRIX translation = XMMatrixTranslationFromVector(m_DerivedPosition);

		XMMATRIX localToWorldMatrix = scale * rotate * translation;
		XMStoreFloat4x4(&m_LocalToWorldMatrix, localToWorldMatrix);
	}
}

void GameObject::OnTransformUpdate(GameObject* gameObject)
{
	for (auto& component : gameObject->m_Components)
	{
		component->OnTransformUpdate();
	}

	for (auto& child : gameObject->m_Children)
	{
		OnTransformUpdate(child.get());
	}
}

void GameObject::SetChildTransformDirty(GameObject* gameObject)
{
	m_TransformDirty = true;

	for (auto& child : gameObject->m_Children)
	{
		SetChildTransformDirty(child.get());
	}
}