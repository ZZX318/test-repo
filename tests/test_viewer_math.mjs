import assert from "node:assert/strict";
import {createCamera, projectPoint, orbitDrag} from "../uav_planner/web/viewer_math.mjs";

const viewport = {width:1920, height:1080};
const base = createCamera({origin:[0,0,0], maximum:[32,32,14]});
const low = projectPoint([16,16,0], base, viewport);
const high = projectPoint([16,16,13], base, viewport);
assert.ok(Math.abs(low[1]-high[1]) > 20, "z must change screen y");
const tilted = {...base, pitch:1.2};
const p1 = projectPoint([16,16,5], {...base,pitch:.2}, viewport);
const p2 = projectPoint([16,16,5], tilted, viewport);
assert.ok(Math.hypot(p1[0]-p2[0],p1[1]-p2[1]) > 5, "pitch must move pixels");
const behind = {...base, target:[16,16,7], distance:5};
assert.equal(projectPoint([16,16,100], behind, viewport), null);
const yawed = orbitDrag(base, 40, 0);
const py = projectPoint([4,16,4], yawed, viewport);
const px = projectPoint([4,16,4], base, viewport);
assert.ok(Math.abs(py[0]-px[0]) > 1, "yaw must move x");
for (const pitch of [.02, 1.50]) {
  const p = projectPoint([4,16,4], {...base,pitch}, viewport);
  assert.ok(p.every(Number.isFinite), "boundary pitch must stay finite");
}
console.log("viewer math: ok");
