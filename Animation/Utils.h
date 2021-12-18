#pragma once
// Utility class that helps with controlling animation playback time. Time is
// computed every update according to the dt given by the caller, playback speed
// and "play" state.
// Internally time is stored as a ratio in unit interval [0,1], as expected by
// ozz runtime animation jobs.
// OnGui function allows to tweak controller parameters through the application Gui.

#include "SamplingJob.h"
#include "LocalToModelJob.h"
#include "LegController.h"

namespace Animation
{
	class PlaybackController {
	public:
		PlaybackController();

		void SetTimeRatio(float time);

		float GetTimeRatio() const;

		float GetPreviousTimeRatio() const;

		bool GetTransition() const;

		float SetPlaybackSpeed(float speed) { playback_speed_ = speed; }

		float GetPlaybackSpeed() const { return playback_speed_; }

		void SetLoop(bool loop) { loop_ = loop; }

		bool GetLoop() { return loop_; }

		void Update(float duration, float dt);

		void Reset();
	
		float GetDeltaTime() const;

		bool OnGui(bool enabled = true, bool allowSetTime = true);

	private:
		// Current animation time ratio, in the unit interval [0,1], where 0 is the
		// beginning of the animation, 1 is the end.
		float time_ratio_;

		// Time ratio of the previous update.
		float previous_time_ratio_;

		// Playback speed, can be negative in order to play the animation backward.
		float playback_speed_;

		// Animation play mode state: play/pause.
		bool play_;

		// Animation loop mode.
		bool loop_;

		// Delta time
		float delta_time_;

		// is this animation transition?
		bool transition;
	};
}

