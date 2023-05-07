# gazebo_worlds package
Constains Gazebo worlds, models and various utility scripts
### Worlds
- marsyard.world - Marsyard terrain based on model from `leo_gazebo_worlds`
- marsyard_artags.world - Marsyard terrain with added Aruco tag models

### Scripts
- `generate_markers.py` - Used to create Aruco tag models. Creates a copy of the model from `template` directory for each of marker patters in `materials/textures` directory. Usage:
```
cd src/gazebo_worlds/models
python generate_markers.py
```