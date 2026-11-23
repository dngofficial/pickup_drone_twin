extends RefCounted

var rng := RandomNumberGenerator.new()

func sample(bodies: Array[RigidBody3D], camera: Node3D, sensors: Node, config: Dictionary, stamp: Dictionary, epoch: int) -> Dictionary:
	var info: Dictionary = camera.camera_info(stamp)
	var optical: Transform3D = camera.optical_pose * Transform3D(Basis(Vector3.RIGHT, PI), Vector3.ZERO)
	var inv := optical.affine_inverse()
	var detections: Array[Dictionary] = []
	var depths: Array = []
	for body in bodies:
		if body.freeze or rng.randf() < float(config.oracle_miss_rate):
			continue
		var grip: Marker3D = body.get_node("PickPoint")
		var pixels: Array[Vector2] = []
		var best_point := Vector3.ZERO
		var best_error := INF
		for offset: Vector3 in [Vector3.ZERO,Vector3(.024,0,0),Vector3(-.024,0,0),Vector3(0,.018,0),Vector3(0,0,.035),Vector3(0,0,-.035),Vector3(0,0,.067),Vector3(0,0,-.067)]:
			var target: Vector3 = grip.global_position + body.global_basis * offset
			var projected := inv * target
			if projected.z < .12 or projected.z > 12.0:
				continue
			var direction := (target - optical.origin).normalized()
			var hit: Dictionary = sensors.ray(optical.origin, target + direction * .09)
			if hit.is_empty() or hit.collider != body:
				continue
			var p: Vector3 = inv * hit.position
			var pixel := Vector2(info.k[0]*p.x/p.z + info.k[2], info.k[4]*p.y/p.z + info.k[5])
			if not Rect2(0,0,info.width,info.height).has_point(pixel):
				continue
			pixels.append(pixel)
			var error: float = hit.position.distance_to(grip.global_position)
			if error < best_error:
				best_error = error
				best_point = Vector3(pixel.x,pixel.y,p.z)
		if pixels.size() < ceili(8.0 * float(config.min_visible_fraction)):
			continue
		var low := pixels[0]
		var high := pixels[0]
		for pixel in pixels:
			low = low.min(pixel)
			high = high.max(pixel)
		var noise := float(config.oracle_box_noise_px)
		var center := (low + high) * .5 + Vector2(rng.randfn(0,noise),rng.randfn(0,noise))
		var pad: float = clampf(float(info.k[0]) * .06 / best_point.z, 3.0, 70.0)
		var extents := (high-low) + Vector2.ONE * pad
		var kind: String = body.get_meta("class_id")
		detections.append(detection(info.header,center,extents,kind,.94))
		depths.append([best_point.x,best_point.y,best_point.z + rng.randfn(0,.003)])
	if rng.randf() < float(config.oracle_false_positive_rate):
		var center := Vector2(rng.randf_range(10,info.width-10),rng.randf_range(10,info.height-10))
		detections.append(detection(info.header,center,Vector2(17,15),"bottle",.55))
		depths.append(null)
	return {"info":info,"boxes":{"header":info.header,"detections":detections},"depth":{"epoch":epoch,"stamp":stamp,"samples":depths}}

func detection(h: Dictionary, pixel: Vector2, size: Vector2, kind: String, score: float) -> Dictionary:
	return {"header":h,"bbox":{"center":{"position":{"x":pixel.x,"y":pixel.y},"theta":0},"size_x":size.x,"size_y":size.y},"results":[{"hypothesis":{"class_id":kind,"score":score}}],"id":""}
