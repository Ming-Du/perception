#include "scene.h"

namespace perception {
namespace fusion {

Scene::Scene() {}

Scene::~Scene() {}

void Scene::AddForegroundTrack(TrackPtr track) {
  foreground_tracks_.push_back(track);
}

void Scene::AddBackgroundTrack(TrackPtr track) {
  background_tracks_.push_back(track);
}

}  // namespace fusion
}  // namespace perception
