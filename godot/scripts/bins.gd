extends Node3D

const G = preload("res://scripts/geometry.gd")
const Station = preload("res://scripts/station.gd")

var bins: Dictionary = {}
var tags: Array[Node3D] = []
var width: float
var rim: float
var settled: Dictionary = {}
var counted: Dictionary = {}
var counts := {"bin_a": 0, "bin_b": 0}

func build(manifest: Dictionary, config: Dictionary) -> void:
	width = float(config.bin_inner_width_m)
	rim = float(config.bin_rim_height_m)
	for id in ["bin_a", "bin_b"]:
		var is_a: bool = id == "bin_a"
		var location := G.to_godot(G.xyz(manifest[id + "_map_xyz"]))
		bins[id] = location
		var color := Color("2293a2") if is_a else Color("da782d")
		var paint := G.material(color)
		var border := G.material(color.lightened(.27))
		var outer := width + .16
		G.solid_box(self, location + Vector3(0,.03,0), Vector3(outer,.06,outer), G.material(Color("30383a")))
		for side in [-1,1]:
			var center: float = side * (width*.5 + .04)
			G.solid_box(self, location + Vector3(center,rim*.5,0), Vector3(.08,rim,outer), paint)
			G.solid_box(self, location + Vector3(0,rim*.5,center), Vector3(width,rim,.08), paint)
			G.box(self, location + Vector3(center,rim,0), Vector3(.095,.035,outer), border)
			G.box(self, location + Vector3(0,rim,center), Vector3(width,.035,.095), border)
		G.sign(self, "BIN A" if is_a else "BIN B", location + Vector3(0,rim+.27,outer*.5), Color.WHITE, .008).billboard = BaseMaterial3D.BILLBOARD_ENABLED
		tags.append(Station.make_tag(self, 20 if is_a else 21, location + Vector3(0,rim+.025,outer*.5+.18), Vector3.UP, .30))

func deliveries(bodies: Array[RigidBody3D], dt: float, epoch: int) -> Array[Dictionary]:
	var events: Array[Dictionary] = []
	for body in bodies:
		var token: String = body.get_meta("released_token", "")
		if token.is_empty() or counted.has(token) or body.freeze:
			continue
		var destination := ""
		if body.linear_velocity.length() <= .09 and body.angular_velocity.length() <= .25:
			for id: String in bins:
				var relative: Vector3 = body.global_position - bins[id]
				if absf(relative.x) < width*.5-.06 and absf(relative.z) < width*.5-.06 and relative.y >= .04 and relative.y < rim:
					destination = id
					break
		if destination.is_empty():
			settled.erase(token)
			continue
		var last: Array = settled.get(token, [destination, 0.0])
		var elapsed := (float(last[1]) if last[0] == destination else 0.0) + dt
		settled[token] = [destination, elapsed]
		if elapsed < .35:
			continue
		counted[token] = true
		counts[destination] += 1
		settled.erase(token)
		events.append({"event_id": "%d-%s" % [epoch, token], "payload_token": token, "bin_id": destination})
	return events
