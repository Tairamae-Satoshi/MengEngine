#include "Utils.h"
#include"..//Vendor/GUI/imgui.h"

namespace Animation
{
	PlaybackController::PlaybackController()
		:time_ratio_(0.f),
		previous_time_ratio_(0.f),
		playback_speed_(1.f),
		play_(true),
		loop_(true),
		transition(false) {}

	void PlaybackController::Update(float duration, float dt)
	{
		float newTime = time_ratio_;

		if (play_)
		{
			delta_time_ = dt;
			newTime = time_ratio_ + dt * playback_speed_ / duration;
		}

		// Must be called even if time doesn't change, in order to update previous
		// frame time ratio. Uses set_time_ratio function in order to update
		// previous_time_ an wrap time value in the unit interval (depending on loop
		// mode).
		SetTimeRatio(newTime);
	}

	void PlaybackController::SetTimeRatio(float ratio) {
		// Mark whether the animation is over.
		transition = previous_time_ratio_ > time_ratio_? true: false;
		previous_time_ratio_ = time_ratio_;

		if (loop_)
		{
			// Wraps in the unit interval [0:1], even for negative values (the reason
			// for using floorf).
			time_ratio_ = ratio - floorf(ratio);
		}
		else
		{
			// Clamps in the unit interval [0:1].
			time_ratio_ = std::clamp(ratio, 0.0f, 1.0f);
		}
	}

	float PlaybackController::GetTimeRatio() const
	{
		return time_ratio_;
	}

	float PlaybackController::GetPreviousTimeRatio() const
	{
		return previous_time_ratio_;
	}

	bool PlaybackController::GetTransition() const
	{
		return transition;
	}


	void PlaybackController::Reset()
	{
		previous_time_ratio_ = time_ratio_ = 0.0f;
		playback_speed_ = 1.0f;
		play_ = true;
	}

	float PlaybackController::GetDeltaTime() const
	{
		return play_ ? delta_time_ : 0.0f;
	}


	bool PlaybackController::OnGui(bool enabled, bool allowSetTime)
	{
		bool timeChanged = false;

		ImGui::Text("Animation Control");
		if (ImGui::Button(play_ ? "Pause" : "Play")) 
		{
			play_ = !play_;
		}

		ImGui::Checkbox("Loop", &loop_);

		//char szLabel[64];
		float ratio = GetTimeRatio();
		if (ImGui::SliderFloat("Animation time", &ratio, 0.0f, 1.0f))
		{
			SetTimeRatio(ratio);
			play_ = false;
			timeChanged = true;
		}

		//std::sprintf(szLabel, "Playback speed: %.2f", mPlaybackSpeed);
		ImGui::SliderFloat("Playback speed", &playback_speed_, -5.0f, 5.0f);

		if (ImGui::Button("Reset playback speed"))
		{
			playback_speed_ = 1.0f;
		}

		return timeChanged;
	}
}