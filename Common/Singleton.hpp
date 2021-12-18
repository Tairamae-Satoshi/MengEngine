#pragma once

template <typename T>
class Singleton
{
public:
	Singleton()
	{
		assert(!ms_Singleton);
		ms_Singleton = static_cast<T*>(this);
	}

	~Singleton()
	{
		assert(ms_Singleton);
		ms_Singleton = nullptr;
	}

	Singleton(const Singleton<T>&) = delete;
	Singleton& operator=(const Singleton<T>&) = delete;

	static T& GetSingleton()
	{
		return *ms_Singleton;
	}

	static T* GetSinglePtr()
	{
		return ms_Singleton;
	}

protected:
	static T* ms_Singleton;
};

template<typename T> T* Singleton<T>::ms_Singleton = nullptr;