shader_type spatial;
render_mode unshaded, fog_disabled, depth_draw_never, depth_test_disabled, cull_disabled;

uniform sampler2D depth_texture : hint_depth_texture, repeat_disable, filter_nearest;

void vertex() {
	POSITION = vec4(VERTEX.xy, 1.0, 1.0);
}

void fragment() {
	float depth = texture(depth_texture, SCREEN_UV).r;
#if CURRENT_RENDERER == RENDERER_COMPATIBILITY
	vec3 ndc = vec3(SCREEN_UV, depth) * 2.0 - 1.0;
#else
	vec3 ndc = vec3(SCREEN_UV * 2.0 - 1.0, depth);
#endif
	vec4 view = INV_PROJECTION_MATRIX * vec4(ndc, 1.0);
	float z = -view.z / view.w;
	float mm = (depth > 0.0 && z > 0.0 && z < 65.535) ? round(z * 1000.0) : 0.0;
	// Put 6+6+4 bits away from channel bucket boundaries. This tolerates
	// byte rounding introduced by the compatibility renderer's color pass.
	vec3 digits = vec3(mod(mm, 64.0), mod(floor(mm / 64.0), 64.0), floor(mm / 4096.0));
	vec3 code = (digits * vec3(4.0, 4.0, 16.0) + vec3(3.0, 3.0, 8.0)) / 255.0;
	ALBEDO = code;
	// Sample the opaque depth buffer before this transparent pass executes.
	ALPHA = 1.0;
}
