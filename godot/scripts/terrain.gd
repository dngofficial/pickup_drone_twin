extends Node3D

const G = preload("res://scripts/geometry.gd")
const Layout = preload("res://scripts/scene_manifest.gd")
const GRID_STEP := 0.5

func build(manifest: Dictionary) -> void:
	var ground := SurfaceTool.new()
	ground.begin(Mesh.PRIMITIVE_TRIANGLES)
	for ix in range(-24, 88):
		for iy in range(-56, 56):
			var x := ix * GRID_STEP
			var y := iy * GRID_STEP
			for p: Vector2 in [Vector2(x,y), Vector2(x,y+GRID_STEP), Vector2(x+GRID_STEP,y), Vector2(x+GRID_STEP,y), Vector2(x,y+GRID_STEP), Vector2(x+GRID_STEP,y+GRID_STEP)]:
				var z := Layout.elevation(manifest, p.x, p.y)
				var shade := 0.11 * sin(p.x * 2.7 + p.y * 1.4) + 0.07 * cos(p.y * 4.2)
				ground.set_color(Color("a17b50").lerp(Color("d3ab78"), clampf(0.46 + shade + z * 0.33, 0, 1)))
				ground.add_vertex(G.to_godot(Vector3(p.x, p.y, z)))
	ground.index()
	ground.generate_normals()
	var terrain_mesh := ground.commit()
	var dirt := G.material(Color.WHITE)
	dirt.vertex_color_use_as_albedo = true
	var terrain := StaticBody3D.new()
	terrain.name = "Ground"
	terrain.collision_layer = G.TERRAIN
	terrain.collision_mask = 0
	add_child(terrain)
	G.mesh(terrain, terrain_mesh, Vector3.ZERO, dirt)
	G.attach_shape(terrain, terrain_mesh.create_trimesh_shape())
	for rock: Dictionary in manifest.rocks:
		add_rock(manifest, rock)

func add_rock(manifest: Dictionary, spec: Dictionary) -> void:
	var rock := StaticBody3D.new()
	var x := float(spec.xy[0])
	var y := float(spec.xy[1])
	var height := float(spec.height_m)
	var radius := float(spec.radius_m)
	rock.position = G.to_godot(Vector3(x, y, Layout.elevation(manifest, x, y) + height * 0.42))
	rock.rotation.y = float(spec.yaw)
	rock.collision_layer = G.TERRAIN
	rock.collision_mask = 0
	add_child(rock)
	var choice: String = ["A", "B", "C"][mini(int(float(spec.variation) * 3.0), 2)]
	var model := G.fitted_model(rock, "res://assets/cc0/rock_large%s.glb" % choice, Vector3(radius*1.55,height,radius*1.55), Vector3.ZERO)
	if model == null:
		return
	model.material_override = G.material(Color("65584d").lerp(Color("9e795a"), float(spec.variation)))
	G.model_hull(rock, model)
