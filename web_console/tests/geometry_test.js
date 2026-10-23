/* Verify ROS/image axes, resolution and rotated origins with known coordinates. */
const assert = require('node:assert/strict');
require('../frontend/geometry.js');
const { worldToPixel, pixelToWorld } = globalThis.G1MapGeometry;
function near(actual, expected) { assert.ok(Math.abs(actual - expected) < 1e-8, `${actual} != ${expected}`); }
const lab = { width: 731, height: 1031, resolution: 0.05, origin: [-8.5, -26.5, 0] };
assert.deepEqual(worldToPixel({ x: -8.5, y: -26.5 }, lab), [0, 1031]);
assert.deepEqual(pixelToWorld([0, 0], lab), { x: -8.5, y: 25.050000000000004 });
const door = worldToPixel({ x: 1.5, y: -21.5 }, lab);
near(door[0], 200); near(door[1], 931);
const rotated = { width: 100, height: 80, resolution: 0.1, origin: [3, -4, Math.PI / 2] };
const pixel = worldToPixel({ x: 1, y: -3 }, rotated);
near(pixel[0], 10); near(pixel[1], 60);
for (const map of [lab, rotated]) {
  for (const point of [{ x: 9.781014110738525, y: 13.265217821184399 }, { x: 0, y: 0 }]) {
    const result = pixelToWorld(worldToPixel(point, map), map);
    near(result.x, point.x); near(result.y, point.y);
  }
}
console.log('Map origin, axes, resolution and rotation checks passed');
