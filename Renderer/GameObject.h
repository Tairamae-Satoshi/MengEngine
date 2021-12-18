#pragma once
#include "SceneManager.h"
#include "Component.h"
#include "../Common/SimpleMath.h"

using namespace DirectX::SimpleMath;

enum class Space
{
	World,
	Self
};

class GameObject
{
public:
	GameObject(GameObject* parent);
	~GameObject();

	// �����ӽڵ�
	GameObject* CreateChild();
	// ɾ�������ӽڵ�
	void DestroyChildren();

	template<typename T> T* AddComponent();
	template<typename T> T* GetComponent();

	void Translate(float x, float y, float z, Space relativeTo = Space::Self);
	void Translate(DirectX::XMFLOAT3 translation, Space relativeTo = Space::Self);
	void Rotate(DirectX::XMFLOAT3 eulers, Space relativeTo = Space::Self);
	void Rotate(float xAngle, float yAngle, float zAngle, Space relativeTo = Space::Self);
	void Rotate(DirectX::XMFLOAT3 axis, float angle, Space relativeTo = Space::Self);

	void SetLocalPosition(Vector3 localPosition) { m_TransformDirty = true; m_Position = localPosition; OnTransformUpdate(this); }
	void SetLocalRotation(Quaternion localRotation) { m_TransformDirty = true; m_Rotation = localRotation; OnTransformUpdate(this); }
	void SetLocalScale(Vector3 localScale) { m_TransformDirty = true; m_Scale = localScale; OnTransformUpdate(this); }

	Vector3 LocalPosition() const { return m_Position; }
	Vector3 WorldPosition() const { return m_DerivedPosition; }
	Quaternion LocalRotation() const { return m_Rotation; }
	Quaternion WorldRotation() const { return m_DerivedRotation; }
	Vector3 LocalScale() const { return m_Scale; }
	Vector3 WorldScale() const { return m_DerivedScale; }
	Vector3 ForwardDir() { _UpdateFromParent(); return m_DerivedRotation * Vector3(0.0f, 0.0f, 1.0f); }

	DirectX::XMFLOAT4X4 LocalToWorldMatrix();

private:
	void _UpdateFromParent();
	void OnTransformUpdate(GameObject* gameObject);
	void SetChildTransformDirty(GameObject* gameObject);

	// ���ڵ�
	GameObject* m_Parent;
	// �ӽڵ�
	std::vector<std::unique_ptr<GameObject>> m_Children;

	// Transform
	DirectX::SimpleMath::Vector3 m_Position = { 0.0f,0.0f,0.0f };
	DirectX::SimpleMath::Quaternion m_Rotation;
	DirectX::SimpleMath::Vector3 m_Scale = { 1.0f,1.0f,1.0f };
	DirectX::SimpleMath::Vector3 m_DerivedPosition;
	DirectX::SimpleMath::Quaternion m_DerivedRotation;
	DirectX::SimpleMath::Vector3 m_DerivedScale;

	DirectX::XMFLOAT4X4 m_LocalToWorldMatrix;
	DirectX::XMFLOAT4X4 m_WorldToLocalMatrix;

	bool m_TransformDirty = true;
	// ÿ�������Constant Buffer����
	UINT m_ObjCBIndex = -1;

	// Components
	std::vector<std::unique_ptr<Component>> m_Components;
};

template<typename T>
inline T* GameObject::AddComponent()
{
	static_assert(std::is_base_of<Component, T>::value,
		"T must be classes derived from Component..");

	T* component = GetComponent<T>();

	if (component != nullptr)
	{
		LOG_WARNING("Warning: Already have Component");
		return component;
	}

	component = new T();
	component->SetGameObject(this);
	m_Components.push_back(std::unique_ptr<T>(component));

	component->OnAddToGameObject();

	return component;
}

template<typename T>
inline T* GameObject::GetComponent()
{
	for (int i = 0; i < m_Components.size(); ++i)
	{
		Component* baseComponent = m_Components[i].get();
		T* concreteComponent = dynamic_cast<T*>(baseComponent);
		if (concreteComponent != nullptr)
			return concreteComponent;
	}

	return nullptr;
}
