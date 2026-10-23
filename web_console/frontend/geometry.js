/* Occupancy-map image rows point down; ROS map coordinates point up. Origin may rotate. */
globalThis.G1MapGeometry = Object.freeze({
  worldToPixel(pose, map) {
    const [ox, oy, yaw] = map.origin, c = Math.cos(yaw), s = Math.sin(yaw);
    const dx = pose.x - ox, dy = pose.y - oy;
    return [(c * dx + s * dy) / map.resolution, map.height - (-s * dx + c * dy) / map.resolution];
  },
  pixelToWorld(pixel, map) {
    const [ox, oy, yaw] = map.origin, c = Math.cos(yaw), s = Math.sin(yaw);
    const x = pixel[0] * map.resolution, y = (map.height - pixel[1]) * map.resolution;
    return { x: ox + c * x - s * y, y: oy + s * x + c * y };
  },
});
