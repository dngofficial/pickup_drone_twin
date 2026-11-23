# Provenance and licensing

The warehouse reference was inspected at commit `a2a53714bbb79777f3db4528322febf4baea5ee5`. Its MIT license requires retaining the Warehouse Twin contributors’ copyright and permission notice with copies or substantial portions of reused code. The exact license is included in `licenses/warehouse_drone_twin-MIT.txt`. The procedural drone is part of that code. The outdoor terrain replaces the AWS rack meshes and textures. Those assets have a separate MIT-0 license, which lacks MIT's notice retention condition. If any are later imported, keep their provenance and license with the asset manifest.

- Warehouse source and MIT license. https://github.com/dngofficial/warehouse_drone_twin/blob/a2a53714bbb79777f3db4528322febf4baea5ee5/LICENSE
- AWS rack manifest and license. https://github.com/dngofficial/warehouse_drone_twin/tree/a2a53714bbb79777f3db4528322febf4baea5ee5/third_party

The public Roboflow **hammer-bottle** project by **urc** lists 1,362 images, version 1, classes `bottle` and `Orange_hammer`, and CC BY 4.0. Credit urc, name and link the dataset and version, link the license, retain supplied notices and indicate label remapping and other modifications. Check the versioned dataset export's license and README after downloading with your key. Its hosted detector is labeled Roboflow 3.0 Object Detection (Fast). It provides dataset provenance for this project, which includes a YOLOv8 training pipeline. The Roboflow credential is supplied locally via `ROBOFLOW_API_KEY`.

Suggested credit: “Training data includes hammer-bottle by urc, Roboflow Universe, version 1, CC BY 4.0. `Orange_hammer` labels were renamed to `hammer`, polygon annotations were converted to enclosing boxes, and the dataset was mixed with synthetic imagery.”

- Dataset. https://universe.roboflow.com/urc-cvc4i/hammer-bottle-l3zmd-odqnx
- License. https://creativecommons.org/licenses/by/4.0/

Ultralytics offers AGPL-3.0 and Enterprise licensing and describes its trained models as AGPL-3.0 by default. This project uses AGPL-3.0-only as the default for new source to accommodate the integrated YOLO path, with upstream MIT notices retained. ONNX is a format conversion and does not change applicable licensing. Datasets, trained weights and credentials are excluded from the repository. Any later model release must track its own license and corresponding source obligations.

- Ultralytics licensing. https://www.ultralytics.com/license

The Mountaineers detection repository documents a camera-relative `/cam_object_pose` with its own `ObjectPoint` type. This project adopts a camera-relative topic with a standard `geometry_msgs/PoseWithCovarianceStamped` type and its own schema. Its primary detection array uses the station-anchored map frame. Mountaineers source code is outside the reuse plan.

- Mountaineers interface reference. https://github.com/team-mountaineers-urc-2025/object_detection_2025

The simulator uses procedural terrain, station and bins, plus the following free models. The originals are retained as bundled GLBs. Dimensions, orientation and materials are adapted at runtime. Collision shapes are derived from the fitted meshes.

| Bundled models | Creator and license | Source |
|---|---|---|
| `godot/assets/cc0/rock_largeA.glb`, `rock_largeB.glb`, `rock_largeC.glb` | Kenney, Nature Kit 2.1, CC0 1.0 | https://kenney.nl/assets/nature-kit |
| `godot/assets/cc0/water_bottle.glb` | Quaternius, Water Bottle, CC0 1.0 | https://poly.pizza/m/KpxDpidn1Z |
| `godot/assets/cc-by/hammer.glb` | J-Toastie, Hammer, CC BY 3.0 | https://poly.pizza/m/66FnMJl5fs |

**Hammer attribution:** Hammer by J-Toastie via Poly Pizza, [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/). This simulator rotates, rescales and recolors it and derives collision geometry from the modified presentation. The model remains covered by its asset license alongside the project's code license. See `asset_manifest.json` for paths, download sources and SHA-256 digests.

