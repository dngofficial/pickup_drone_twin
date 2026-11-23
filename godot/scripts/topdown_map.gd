extends Control

signal selected(id: String)
var telemetry: Dictionary = {}
var trajectory: Array[Vector2] = []
var bounds := Rect2(-4,-12,32,24)

func update(value: Dictionary) -> void:
	if value.get("status",{}).get("epoch") != telemetry.get("status",{}).get("epoch"):
		trajectory.clear()
	telemetry = value
	var pose: Array = value.pose
	var p := Vector2(pose[0],pose[1])
	if trajectory.is_empty() or trajectory[-1].distance_to(p) > .12:
		trajectory.append(p)
		if trajectory.size()>600: trajectory.pop_front()
	queue_redraw()

func pixel(p: Vector2) -> Vector2:
	var area := Rect2(12,34,size.x-24,size.y-45)
	return area.position + Vector2((p.x-bounds.position.x)/bounds.size.x,1.0-(p.y-bounds.position.y)/bounds.size.y)*area.size

func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO,size),Color("15252dec"))
	draw_rect(Rect2(Vector2.ZERO,size),Color("3e5a5d"),false)
	var font := ThemeDB.fallback_font
	draw_string(font,Vector2(12,23),"SITE MAP  /  SLAM + COVERAGE",HORIZONTAL_ALIGNMENT_LEFT,-1,14,Color("a1b8b6"))
	for x in range(-4,29,4):
		draw_line(pixel(Vector2(x,-12)),pixel(Vector2(x,12)),Color("2f4246"))
	for y in range(-12,13,4):
		draw_line(pixel(Vector2(-4,y)),pixel(Vector2(28,y)),Color("2f4246"))
	for p: Array in telemetry.get("coverage",[]):
		draw_rect(Rect2(pixel(Vector2(p[0],p[1]))-Vector2(2.5,2.5),Vector2(5,5)),Color("46785a77"))
	for p: Array in telemetry.get("map",[]):
		if bounds.has_point(Vector2(p[0],p[1])):
			draw_circle(pixel(Vector2(p[0],p[1])),1.0,Color("95a89c8a"))
	if trajectory.size()>1:
		var line := PackedVector2Array()
		for p: Vector2 in trajectory: line.append(pixel(p))
		draw_polyline(line,Color("ecad57"),1.5,true)
	var pose: Array = telemetry.get("pose",[0,0,0,0])
	var here := pixel(Vector2(pose[0],pose[1]))
	var last := here
	for p: Array in telemetry.get("path",[]):
		var next := pixel(Vector2(p[0],p[1]))
		draw_line(last,next,Color("61dbc3"),2.0,true)
		last = next
	draw_circle(here,4.0,Color("e7b665"))
	for label: String in ["bin_a","bin_b"]:
		var b: Array = telemetry.get("bins",{}).get(label,[-2.0,2.5 if label=="bin_a" else -2.5,0.0])
		var at := pixel(Vector2(b[0],b[1]))
		draw_circle(at,5.,Color("45cbcd") if label=="bin_a" else Color("f2a650"))
		draw_string(font,at+Vector2(6,4),label.right(1).to_upper(),HORIZONTAL_ALIGNMENT_LEFT,-1,12,Color.WHITE)
	draw_string(font,pixel(Vector2.ZERO)+Vector2(7,4),"Home",HORIZONTAL_ALIGNMENT_LEFT,-1,12,Color.WHITE)
	for track: Array in telemetry.get("tracks",[]):
		var at := pixel(Vector2(track[3],track[4]))
		var color: Color = [Color("e2ad5e"),Color("7ee0a7"),Color("ffff89"),Color("75caff"),Color("6d7078"),Color("a6cda3")][int(track[2])]
		draw_circle(at,3.5,color)
		if track[2] in [1,2]:
			draw_string(font,at+Vector2(5,0),str(track[0]),HORIZONTAL_ALIGNMENT_LEFT,-1,11,color)

func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed and event.button_index==MOUSE_BUTTON_LEFT:
		var nearest := 15.0
		var target := ""
		for track: Array in telemetry.get("tracks",[]):
			var distance := pixel(Vector2(track[3],track[4])).distance_to(event.position)
			if track[2] in [1,2] and distance<nearest:
				target = str(track[0])
				nearest = distance
		if not target.is_empty(): selected.emit(target)
