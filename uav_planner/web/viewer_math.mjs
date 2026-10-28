export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm = (a) => { const n = Math.hypot(...a) || 1; return a.map(v => v / n); };

export function createCamera(bounds = {origin:[0,0,0], maximum:[32,32,14]}) {
  const o = bounds.origin, m = bounds.maximum;
  return {yaw:.70, pitch:.55, distance:62,
    target:[(o[0]+m[0])/2,(o[1]+m[1])/2,o[2]+(m[2]-o[2])*.52],
    fov:48*Math.PI/180, near:.5, ortho:false};
}

export function projectPoint(point, cam, viewport = {width:1920,height:1080}) {
  const cp=Math.cos(cam.pitch), sp=Math.sin(cam.pitch);
  const eye=[cam.target[0]+cam.distance*cp*Math.sin(cam.yaw),
    cam.target[1]-cam.distance*cp*Math.cos(cam.yaw),cam.target[2]+cam.distance*sp];
  const f=norm(sub(cam.target,eye)), r=norm(cross(f,[0,0,1])), u=cross(r,f);
  const d=sub(point,eye), depth=dot(d,f);
  if(depth<cam.near)return null;
  const focal=(viewport.height/2)/Math.tan(cam.fov/2);
  const scale=cam.ortho?Math.min(viewport.width,viewport.height)/cam.distance:focal/depth;
  return [viewport.width/2+dot(d,r)*scale,viewport.height/2-dot(d,u)*scale,depth];
}

export function orbitDrag(cam, dx, dy) {
  return {...cam, yaw:cam.yaw+dx*.008, pitch:clamp(cam.pitch+dy*.008,.02,1.50)};
}
