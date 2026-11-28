# 📦 Assets

[⏫ API reference](README.md)

Header: `RobotIK/Robot/Assets/AssetResolver.hpp`.

```cpp
robotik::AssetResolver const assets = robotik::AssetResolver::standard();
std::filesystem::path const urdf = robotik::require(assets.model("robotik.robots.robot_6axis"));
```

| Method | Does |
|--------|------|
| `standard()` | Resolver with the standard search roots (`$ROBOTIK_DATA`, `data/packs`, install prefix) |
| `model(id)` | `Result` with the path of a model; an error when no pack provides it |
| `find(pack id)`, `require(pack id, version)` | A pack |
| `addRoot(path)`, `addPack(path)` | More places to search |
| `packs()`, `errors()` | What was found, and problems (conflicts) |
