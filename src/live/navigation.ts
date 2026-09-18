export type Point = [number, number];
export type Arena = { arena: number; clearance: number; obstacles: { x: number; y: number; hx: number; hy: number; height: number }[] };
export const distance = (a: number[], b: number[]) => Math.hypot(a[0]-b[0],a[1]-b[1]);

export function clear(point: number[], arena: Arena, margin = arena.clearance) {
  if (Math.max(Math.abs(point[0]),Math.abs(point[1])) > arena.arena-margin) return false;
  return arena.obstacles.every(o => Math.hypot(Math.max(Math.abs(point[0]-o.x)-o.hx,0),Math.max(Math.abs(point[1]-o.y)-o.hy,0)) > margin);
}

export function segmentClear(a: number[], b: number[], arena: Arena, margin = arena.clearance) {
  const samples = Math.max(1,Math.ceil(distance(a,b)/.025));
  for (let i=0; i<=samples; i++) if (!clear([a[0]+(b[0]-a[0])*i/samples,a[1]+(b[1]-a[1])*i/samples],arena,margin)) return false;
  return true;
}

export function planPath(start: Point, goal: Point, arena: Arena): Point[] {
  if (!goal.every(Number.isFinite) || !clear(start,arena) || !clear(goal,arena)) throw new Error('Choose clear floor away from obstacles and walls.');
  if (segmentClear(start,goal,arena)) return [start,goal];
  const cell = (p: Point): Point => [Math.round((p[0]+arena.arena)/.1),Math.round((p[1]+arena.arena)/.1)];
  const point = (c: Point): Point => [c[0]*.1-arena.arena,c[1]*.1-arena.arena];
  const key = (c: Point) => c.join(',');
  const first=cell(start), last=cell(goal);
  if (!segmentClear(start,point(first),arena) || !segmentClear(point(last),goal,arena)) throw new Error('Move the destination slightly away from the clearance boundary.');
  const open = [{cell:first,priority:0}];
  const costs = new Map([[key(first),0]]), parent = new Map<string,Point>();
  while (open.length) {
    open.sort((a,b) => a.priority-b.priority);
    const current = open.shift()!.cell;
    if (key(current) === key(last)) break;
    for (let dx=-1; dx<=1; dx++) for (let dy=-1; dy<=1; dy++) {
      if (!dx && !dy) continue;
      const next: Point = [current[0]+dx,current[1]+dy];
      if (!segmentClear(point(current),point(next),arena)) continue;
      const cost = costs.get(key(current))! + Math.hypot(dx,dy)*.1;
      if (cost < (costs.get(key(next)) ?? Infinity)) {
        costs.set(key(next),cost); parent.set(key(next),current);
        open.push({cell:next,priority:cost+distance(point(next),point(last))});
      }
    }
  }
  if (!costs.has(key(last))) throw new Error('No route with enough room for all four feet.');
  const cells = [last];
  while (key(cells.at(-1)!) !== key(first)) cells.push(parent.get(key(cells.at(-1)!))!);
  const points: Point[] = [start,...cells.reverse().map(point),goal];
  const result = [start];
  for (let i=0; i<points.length-1;) {
    let j=points.length-1;
    while (j>i+1 && !segmentClear(points[i],points[j],arena)) j--;
    result.push(points[j]); i=j;
  }
  return result;
}
