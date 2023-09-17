
#pragma once

#include <memory>
#include <vector>

#include "base/sensor_object.h"
#include "base/track.h"

namespace perception {
namespace fusion {

class Scene {
 public:
  Scene();
  ~Scene();

  inline std::vector<TrackPtr>& GetForegroundTracks() { return foreground_tracks_; }

  inline const std::vector<TrackPtr>& GetForegroundTracks() const { return foreground_tracks_; }

  inline std::vector<TrackPtr>& GetBackgroundTracks() { return background_tracks_; }

  inline const std::vector<TrackPtr>& GetBackgroundTracks() const { return background_tracks_; }

  void AddForegroundTrack(TrackPtr track);
  void AddBackgroundTrack(TrackPtr track);

 protected:
  std::vector<TrackPtr> foreground_tracks_;
  std::vector<TrackPtr> background_tracks_;
};

typedef std::shared_ptr<Scene> ScenePtr;
typedef std::shared_ptr<const Scene> SceneConstPtr;

}  // namespace fusion
}  // namespace perception
