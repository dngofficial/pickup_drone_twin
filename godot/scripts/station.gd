extends Node3D

const G = preload("res://scripts/geometry.gd")

func build() -> Array[Node3D]:
	var ink := G.material(Color("26333b"))
	var steel := G.material(Color("4d5e66"), 0.5)
	var yellow := G.material(Color("f4b44d"))
	G.solid_box(self, Vector3(0,-0.015,0), Vector3(2.2,0.068,2.2), ink)
	for s in [-1,1]:
		G.box(self, Vector3(s*.97,.025,0), Vector3(.055,.011,1.98), yellow)
		G.box(self, Vector3(0,.025,s*.97), Vector3(1.98,.011,.055), yellow)
		G.box(self, Vector3(s*.25,.026,0), Vector3(.075,.01,.62), yellow)
	G.box(self, Vector3(0,.026,0), Vector3(.55,.01,.075), yellow)
	var label := G.sign(self, "HOME", Vector3(0,.033,.72), Color.WHITE, .005)
	label.rotation.x = -PI/2
	var mast := Vector3(-1.38,0,.64)
	G.solid_box(self, mast + Vector3(0,.14,0), Vector3(.55,.28,.52), ink)
	G.solid_box(self, mast + Vector3(0,1.05,0), Vector3(.09,2.1,.09), steel)
	G.cylinder(self, mast + Vector3(0,2.16,0), .11,.14,yellow)
	var tags: Array[Node3D] = []
	var directions := [Vector3.FORWARD, Vector3.RIGHT, Vector3.BACK, Vector3.LEFT]
	for i in range(directions.size()):
		var normal: Vector3 = directions[i]
		var tag := make_tag(self, 10 + i, mast + Vector3(0,1.8,0) + normal * .19, normal, .40)
		tags.append(tag)
	return tags

static func make_tag(parent: Node3D, id: int, at: Vector3, normal: Vector3, width: float) -> Node3D:
	# Visible coded mock panel. The simulated detector reports geometric observations.
	var panel := Node3D.new()
	panel.name = "Tag%d" % id
	panel.position = at
	var up := Vector3.FORWARD if absf(normal.dot(Vector3.UP)) > .9 else Vector3.UP
	panel.basis = Basis.looking_at(-normal, up)
	panel.set_meta("tag_id", id)
	panel.set_meta("width", width)
	parent.add_child(panel)
	var texture := Image.create_empty(64, 64, false, Image.FORMAT_RGB8)
	var paper := Color("f5f3df")
	texture.fill(paper)
	texture.fill_rect(Rect2i(7,7,50,50), Color("121819"))
	var rng := RandomNumberGenerator.new()
	rng.seed = id * 193
	for row in range(5):
		for col in range(5):
			if rng.randi_range(0,1) == 1:
				texture.fill_rect(Rect2i(12+col*8,12+row*8,8,8), paper)
	var paint := G.material(Color.WHITE)
	paint.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	paint.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
	paint.albedo_texture = ImageTexture.create_from_image(texture)
	var face := QuadMesh.new()
	face.size = Vector2(width, width)
	G.mesh(panel, face, Vector3.ZERO, paint)
	return panel
