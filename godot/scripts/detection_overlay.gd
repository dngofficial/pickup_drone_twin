extends Control

var boxes: Array = []

func _draw() -> void:
	var scale_px := size / Vector2(640,360)
	var font := ThemeDB.fallback_font
	for item: Array in boxes:
		var box: Array = item[2]
		var left := Vector2(box[0]-box[2]*.5,box[1]-box[3]*.5) * scale_px
		var rect := Rect2(left,Vector2(box[2],box[3])*scale_px).intersection(Rect2(Vector2.ZERO,size))
		if rect.has_area():
			var color := Color("77f3c5") if item[0]=="bottle" else Color("ffb45d")
			draw_rect(rect,color,false,1.4)
			draw_string(font,Vector2(rect.position.x,maxf(12,rect.position.y-3)),"%s %.2f" % [item[0],item[1]],HORIZONTAL_ALIGNMENT_LEFT,-1,12,color)
