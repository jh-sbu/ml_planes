#import bevy_pbr::forward_io::VertexOutput
#import bevy_pbr::mesh_view_bindings::view

fn grid_line(world_xz: vec2<f32>, spacing: f32) -> f32 {
    let uv = world_xz / spacing;
    let fw = max(fwidth(uv), vec2<f32>(0.001));
    let d = abs(fract(uv - 0.5) - 0.5) / fw;
    return 1.0 - clamp(min(d.x, d.y), 0.0, 1.0);
}

fn axis_line(coord: f32) -> f32 {
    let fw = max(fwidth(coord), 0.001);
    let d = abs(coord) / fw;
    return 1.0 - clamp(d, 0.0, 1.0);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let world_xz = in.world_position.xz;
    let dist = length(world_xz - view.world_position.xz);
    let fade = 1.0 - smoothstep(800.0, 3000.0, dist);

    let minor = grid_line(world_xz, 10.0);
    let major = grid_line(world_xz, 100.0);

    let base       = vec3<f32>(0.05, 0.12, 0.05);
    let minor_col  = vec3<f32>(0.10, 0.35, 0.10);
    let major_col  = vec3<f32>(0.20, 0.60, 0.20);
    let x_axis_col = vec3<f32>(0.0, 0.45, 0.70);  // x = 0, blue (colorblind-safe)
    let z_axis_col = vec3<f32>(0.90, 0.60, 0.0);  // z = 0, orange (colorblind-safe)

    var color = base;
    color = mix(color, minor_col, minor);
    color = mix(color, major_col, major);
    color = mix(color, x_axis_col, axis_line(world_xz.x));  // line where x = 0
    color = mix(color, z_axis_col, axis_line(world_xz.y));  // line where z = 0
    color = mix(base, color, fade);

    return vec4<f32>(color, 1.0);
}
