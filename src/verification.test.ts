import { describe, expect, it } from "vitest";
import * as math from "mathjs";
import { DofID, LinearStaticSolver, LoadCase } from ".";
import { Beam2D } from "./Beam2D";
import { BeamConcentratedLoad } from "./BeamConcentratedLoad";
import { BeamElementTrapezoidalEdgeLoad } from "./BeamElementTrapezoidalEdgeLoad";
import { BeamElementUniformEdgeLoad } from "./BeamElementUniformEdgeLoad";
import { BeamTemperatureLoad } from "./BeamTemperatureLoad";
import { Node } from "./Node";

/**
 * Shared helpers for the verification tests: building small models, reading results as plain
 * numbers, and the global resultants of every kind of load.
 *
 * Conventions, as the solver uses them: x to the right, z down, y = z x x into the page, so a
 * moment about y of a force (Fx, Fz) at (x, z) is z * Fx - x * Fz.
 */
const E = 210e9;
const G = 81e9;
const ALPHA = 12e-6;

/** An IPE 200-like section; `k` decides whether shear deforms (1) or not (1e32). */
const section = (k: number) => ({ a: 2.85e-3, iy: 1.943e-5, h: 0.2, k });

const newSolver = (k = 1e32) => {
  const solver = new LinearStaticSolver();
  solver.domain.createMaterial("m", { e: E, g: G, alpha: ALPHA, d: 7850 });
  solver.domain.createCrossSection("cs", section(k));
  return solver;
};

/** Turns a node's support axes by `degrees`, as EduBeam does for a rotated support. */
const turnNode = (node: Node, degrees: number) => {
  const r = (degrees * Math.PI) / 180;
  node.updateLcs({ locx: [Math.cos(r), 0, Math.sin(r)], locy: [0, 1, 0] });
};

const num = (value: unknown): number => {
  if (typeof value === "number") return value;
  const m = value as math.Matrix;
  if (m && typeof m.get === "function") return m.get([0]) as number;
  if (Array.isArray(value)) return num(value[0]);
  throw new TypeError("Not a number: " + String(value));
};

/** All three global displacement components of a node, zero where it has none. */
const displacements = (solver: LinearStaticSolver, label: string): [number, number, number] => {
  const node = solver.domain.getNode(label);
  const lc = solver.loadCases[0];
  const dofs = solver.getNodeDofIDs(label);
  const rn = dofs.map((d) => num(node.getUnknowns(lc, [d])));
  const t = node.getTransformationMtrx(dofs) as math.Matrix;
  const rg = (node.hasLcs() ? (math.multiply(t, rn) as math.Matrix).toArray() : rn) as number[];
  const at = (d: DofID) => {
    const i = dofs.indexOf(d);
    return i === -1 ? 0 : rg[i];
  };
  return [at(DofID.Dx), at(DofID.Dz), at(DofID.Ry)];
};

/** A node's reactions in the global axes, [Rx, Rz, Ry], zero where it has no support. */
const reactions = (solver: LinearStaticSolver, label: string): [number, number, number] => {
  const node = solver.domain.getNode(label);
  const lc = solver.loadCases[0];
  if (node.bcs.size === 0) return [0, 0, 0];
  const r = node.getReactions(lc, true);
  const values = r.values as unknown as math.Matrix | number[];
  const read = (i: number) => (Array.isArray(values) ? num(values[i]) : ((values as math.Matrix).get([i]) as number));
  const at = (d: DofID) => {
    const i = (r.dofs as number[]).indexOf(d);
    return i === -1 ? 0 : read(i);
  };
  return [at(DofID.Dx), at(DofID.Dz), at(DofID.Ry)];
};

/** A global force and moment about the origin. */
type Resultant = { fx: number; fz: number; my: number };

const momentOf = (x: number, z: number, fx: number, fz: number) => z * fx - x * fz;

/** The global resultant of every applied load in the load case, nodal and element. */
const appliedResultant = (solver: LinearStaticSolver, lc: LoadCase = solver.loadCases[0]): Resultant => {
  const sum: Resultant = { fx: 0, fz: 0, my: 0 };
  const add = (x: number, z: number, fx: number, fz: number, my = 0) => {
    sum.fx += fx;
    sum.fz += fz;
    sum.my += momentOf(x, z, fx, fz) + my;
  };

  for (const load of lc.nodalLoadList) {
    const [x, , z] = solver.domain.getNode(load.target).coords;
    const v = load.values as Record<number, number>;
    add(x, z, v[DofID.Dx] ?? 0, v[DofID.Dz] ?? 0, v[DofID.Ry] ?? 0);
  }

  for (const load of lc.elementLoadList) {
    const el = solver.domain.getElement(load.target) as Beam2D;
    const [x1, , z1] = solver.domain.getNode(el.nodes[0]).coords;
    const geo = el.computeGeo();
    const c = geo.dx / geo.l;
    const s = geo.dz / geo.l;
    const at = (d: number) => [x1 + c * d, z1 + s * d] as const;

    if (load instanceof BeamElementUniformEdgeLoad) {
      const g = load.getGlobalIntensities();
      const [x, z] = at(geo.l / 2);
      add(x, z, g.fx * geo.l, g.fz * geo.l);
    } else if (load instanceof BeamElementTrapezoidalEdgeLoad) {
      const g = load.getGlobalIntensities();
      // A trapezoid is a uniform part and a triangle rising to the end
      const l = geo.l;
      const [xu, zu] = at(l / 2);
      add(xu, zu, g.start.fx * l, g.start.fz * l);
      const [xt, zt] = at((2 * l) / 3);
      add(xt, zt, ((g.end.fx - g.start.fx) * l) / 2, ((g.end.fz - g.start.fz) * l) / 2);
    } else if (load instanceof BeamConcentratedLoad) {
      const g = load.getGlobalIntensities();
      const [x, z] = at(load.values[3]);
      add(x, z, g.fx, g.fz, g.my);
    }
    // A temperature change applies no load of its own
  }
  return sum;
};

/** The global resultant of all support reactions. */
const reactionResultant = (solver: LinearStaticSolver): Resultant => {
  const sum: Resultant = { fx: 0, fz: 0, my: 0 };
  for (const [label, node] of solver.domain.nodes) {
    const [rx, rz, ry] = reactions(solver, label);
    const [x, , z] = node.coords;
    sum.fx += rx;
    sum.fz += rz;
    sum.my += momentOf(x, z, rx, rz) + ry;
  }
  return sum;
};

/** Internal forces of an element at distance x from its first node. */
const internal = (solver: LinearStaticSolver, label: string, x: number) => {
  const el = solver.domain.getElement(label) as Beam2D;
  const lc = solver.loadCases[0];
  return { N: el.computeNormalForceAt(lc, x), V: el.computeShearForceAt(lc, x), M: el.computeBendingMomentAt(lc, x) };
};

/** `actual` equal to `expected` within a tolerance relative to `scale` (the size of the problem). */
const near = (actual: number, expected: number, scale: number, rel = 1e-7) =>
  Math.abs(actual - expected) <= rel * Math.max(Math.abs(scale), 1e-300);

/**
 * Cuts element `label` into `n` equal pieces (plus a cut at every point load on it), with each of
 * its loads carried over exactly: a uniform load on every piece, a trapezoid in matching slices,
 * a point load as a nodal load at its cut, a temperature change on every piece. Its hinges go to
 * the end pieces. Nodal results of the cut model are exact, which makes it the reference for the
 * original element's diagrams and deflected shape between its ends.
 */
const subdivide = (solver: LinearStaticSolver, label: string, n: number) => {
  const d = solver.domain;
  const lc = solver.loadCases[0];
  const el = d.getElement(label) as Beam2D;
  const [n1, n2] = el.nodes;
  const c1 = d.getNode(n1).coords;
  const c2 = d.getNode(n2).coords;
  const l = el.computeGeo().l;
  const own = lc.elementLoadList.filter((x) => x.target === label);

  const cuts = new Set<number>([0, l]);
  for (let i = 1; i < n; i++) cuts.add((i * l) / n);
  for (const load of own) if (load instanceof BeamConcentratedLoad) cuts.add(load.values[3]);
  const at = [...cuts].sort((p, q) => p - q);

  const nodeAt = at.map((p, i) => {
    if (i === 0) return n1;
    if (i === at.length - 1) return n2;
    const name = `${label}~${i}`;
    const t = p / l;
    d.createNode(name, [c1[0] + (c2[0] - c1[0]) * t, 0, c1[2] + (c2[2] - c1[2]) * t]);
    return name;
  });

  // Read while the element still exists: a load in its axes needs its geometry
  const pointGlobal = new Map(
    own
      .filter((x) => x instanceof BeamConcentratedLoad)
      .map((x) => [x, (x as BeamConcentratedLoad).getGlobalIntensities()])
  );

  d.elements.delete(label);
  const pieces = at.slice(0, -1).map((p, i) => {
    const name = `${label}#${i}`;
    const last = i === at.length - 2;
    d.createBeam2D(name, [nodeAt[i], nodeAt[i + 1]], el.mat, el.cs, [i === 0 && el.hinges[0], last && el.hinges[1]]);
    return { name, from: p, to: at[i + 1] };
  });

  lc.elementLoadList = lc.elementLoadList.filter((x) => x.target !== label);
  for (const load of own) {
    if (load instanceof BeamElementUniformEdgeLoad) {
      for (const p of pieces) lc.createBeamElementUniformEdgeLoad(p.name, [...load.values], load.lcs);
    } else if (load instanceof BeamElementTrapezoidalEdgeLoad) {
      const v = (pos: number, i: 0 | 1) => load.startValues[i] + ((load.endValues[i] - load.startValues[i]) * pos) / l;
      for (const p of pieces)
        lc.createBeamElementTrapezoidalEdgeLoad(
          p.name,
          [v(p.from, 0), v(p.from, 1)],
          [v(p.to, 0), v(p.to, 1)],
          load.lcs
        );
    } else if (load instanceof BeamConcentratedLoad) {
      const g = pointGlobal.get(load)!;
      lc.createNodalLoad(nodeAt[at.indexOf(load.values[3])], { [DofID.Dx]: g.fx, [DofID.Dz]: g.fz, [DofID.Ry]: g.my });
    } else if (load instanceof BeamTemperatureLoad) {
      for (const p of pieces) lc.createBeamTemperatureLoad(p.name, [...load.values]);
    }
  }

  return { at, nodeAt, pieces, l };
};

/**
 * A gable frame with something of everything: a roller turned -20 degrees, a pin turned 30, a
 * clamped strut, inclined rafters, hinges, a cantilever, and every kind of load the solver has.
 * Every part of it is held in more than one way, so every load reaches every member.
 */
type LoadKind =
  | "nodal"
  | "udl-global"
  | "udl-local"
  | "trapezoid-global"
  | "trapezoid-local"
  | "point-global"
  | "point-local"
  | "moment"
  | "temperature"
  | "settlement";

const LOADS: LoadKind[] = [
  "nodal",
  "udl-global",
  "udl-local",
  "trapezoid-global",
  "trapezoid-local",
  "point-global",
  "point-local",
  "moment",
  "temperature",
  "settlement",
];

const frame = (loads: LoadKind[], k = 1e32, solve = true): LinearStaticSolver => {
  const solver = newSolver(k);
  const d = solver.domain;
  const a = d.createNode("A", [0, 0, 0], [DofID.Dz]);
  turnNode(a, -20);
  d.createNode("B", [0, 0, -4]);
  d.createNode("C", [3, 0, -5.5]);
  d.createNode("D", [6, 0, -4]);
  const e = d.createNode("E", [6.5, 0, 0], [DofID.Dx, DofID.Dz]);
  turnNode(e, 30);
  d.createNode("F", [8, 0, -4]);
  d.createNode("G", [-1, 0, 0], [DofID.Dx, DofID.Dz, DofID.Ry]);

  d.createBeam2D("AB", ["A", "B"], "m", "cs");
  d.createBeam2D("BC", ["B", "C"], "m", "cs");
  d.createBeam2D("CD", ["C", "D"], "m", "cs");
  d.createBeam2D("DE", ["D", "E"], "m", "cs", [true, false]);
  d.createBeam2D("DF", ["D", "F"], "m", "cs");
  d.createBeam2D("GB", ["G", "B"], "m", "cs", [false, true]);

  const lc = solver.loadCases[0];
  for (const kind of loads) {
    if (kind === "nodal") lc.createNodalLoad("B", { [DofID.Dx]: 12e3, [DofID.Dz]: 5e3, [DofID.Ry]: 7e3 });
    if (kind === "udl-global") lc.createBeamElementUniformEdgeLoad("BC", [2e3, 9e3], false);
    if (kind === "udl-local") lc.createBeamElementUniformEdgeLoad("CD", [1.5e3, 8e3], true);
    if (kind === "trapezoid-global") lc.createBeamElementTrapezoidalEdgeLoad("DF", [1e3, 2e3], [-3e3, 11e3], false);
    if (kind === "trapezoid-local") lc.createBeamElementTrapezoidalEdgeLoad("DE", [4e3, -1e3], [0, 6e3], true);
    if (kind === "point-global") lc.createBeamConcentratedLoad("AB", [6e3, 3e3, 0, 1.3], false);
    if (kind === "point-local") lc.createBeamConcentratedLoad("CD", [-2e3, 15e3, 0, 1.1], true);
    if (kind === "moment") lc.createBeamConcentratedLoad("DF", [0, 0, 9e3, 0.7], true);
    if (kind === "temperature") lc.createBeamTemperatureLoad("CD", [25, -15, 0]);
    if (kind === "settlement") lc.createPrescribedDisplacement("G", { [DofID.Dz]: 0.01, [DofID.Ry]: 0.002 });
  }

  if (solve) solver.solve();
  return solver;
};

const SCALE = 1e5; // forces of the order of 10 kN acting over a few metres

describe("global equilibrium: the reactions balance the loads", () => {
  for (const k of [1e32, 1]) {
    for (const kind of LOADS) {
      it(`${kind}, ${k === 1 ? "with" : "without"} shear deformation`, () => {
        const solver = frame([kind], k);
        const applied = appliedResultant(solver);
        const support = reactionResultant(solver);

        expect(near(applied.fx + support.fx, 0, SCALE)).toBe(true);
        expect(near(applied.fz + support.fz, 0, SCALE)).toBe(true);
        expect(near(applied.my + support.my, 0, SCALE)).toBe(true);
      });
    }

    it(`everything at once, ${k === 1 ? "with" : "without"} shear deformation`, () => {
      const solver = frame(LOADS, k);
      const applied = appliedResultant(solver);
      const support = reactionResultant(solver);

      expect(near(applied.fx + support.fx, 0, SCALE)).toBe(true);
      expect(near(applied.fz + support.fz, 0, SCALE)).toBe(true);
      expect(near(applied.my + support.my, 0, SCALE)).toBe(true);
    });
  }
});

describe("each member: the diagrams end at the member's end forces", () => {
  for (const kind of LOADS) {
    it(kind, () => {
      const solver = frame([kind]);
      const lc = solver.loadCases[0];
      for (const [label, el] of solver.domain.elements) {
        const beam = el as Beam2D;
        const l = beam.computeGeo().l;
        const f = beam.computeEndForces(lc);
        const end = internal(solver, label, l);
        const start = internal(solver, label, 0);

        expect(near(start.N, -f.get([0]), SCALE), `${label} N(0)`).toBe(true);
        expect(near(start.V, -f.get([1]), SCALE), `${label} V(0)`).toBe(true);
        expect(near(start.M, -f.get([2]), SCALE), `${label} M(0)`).toBe(true);
        expect(near(end.N, f.get([3]), SCALE), `${label} N(l)`).toBe(true);
        expect(near(end.V, f.get([4]), SCALE), `${label} V(l)`).toBe(true);
        expect(near(end.M, f.get([5]), SCALE), `${label} M(l)`).toBe(true);
      }
    });
  }
});

/** The member each load kind is applied to, so the cut model refines where the load is. */
const LOADED: Record<LoadKind, string> = {
  nodal: "BC",
  "udl-global": "BC",
  "udl-local": "CD",
  "trapezoid-global": "DF",
  "trapezoid-local": "DE",
  "point-global": "AB",
  "point-local": "CD",
  moment: "DF",
  temperature: "CD",
  settlement: "BC",
};

const PIECES = 16;

describe("between the ends: a member agrees with the same member cut into short pieces", () => {
  for (const k of [1e32, 1]) {
    for (const kind of LOADS) {
      const label = LOADED[kind];
      const shear = k === 1 ? "with" : "without";

      it(`${kind} on ${label}, ${shear} shear deformation`, () => {
        const original = frame([kind], k);
        const cut = frame([kind], k, false);
        const { at, nodeAt, pieces, l } = subdivide(cut, label, PIECES);
        cut.solve();
        const lc = original.loadCases[0];
        const beam = original.domain.getElement(label) as Beam2D;

        // Nodal displacements: the same at every node the two models share
        const dScale = Math.max(
          ...["A", "B", "C", "D", "E", "F", "G"].flatMap((n) => displacements(cut, n).slice(0, 2).map(Math.abs))
        );
        for (const n of ["A", "B", "C", "D", "E", "F", "G"]) {
          const a = displacements(original, n);
          const b = displacements(cut, n);
          expect(near(a[0], b[0], dScale, 1e-6), `${n} ux`).toBe(true);
          expect(near(a[1], b[1], dScale, 1e-6), `${n} uz`).toBe(true);
        }

        // Diagrams in the middle of every piece
        const fScale = Math.max(
          1,
          ...pieces.flatMap((p) => Object.values(internal(cut, p.name, (p.to - p.from) / 2)).map(Math.abs))
        );
        for (const p of pieces) {
          const x = (p.from + p.to) / 2;
          const a = internal(original, label, x);
          const b = internal(cut, p.name, x - p.from);
          expect(near(a.N, b.N, fScale, 1e-6), `N at ${x.toFixed(3)}: ${a.N} vs ${b.N}`).toBe(true);
          expect(near(a.V, b.V, fScale, 1e-6), `V at ${x.toFixed(3)}: ${a.V} vs ${b.V}`).toBe(true);
          expect(near(a.M, b.M, fScale, 1e-6), `M at ${x.toFixed(3)}: ${a.M} vs ${b.M}`).toBe(true);
        }

        // Deflected shape at the uniform cuts
        const defl = beam.computeGlobalDefl(lc, PIECES);
        const wScale = Math.max(...nodeAt.flatMap((n) => displacements(cut, n).slice(0, 2).map(Math.abs)));
        for (let i = 0; i <= PIECES; i++) {
          const j = at.findIndex((p) => Math.abs(p - (i * l) / PIECES) < 1e-12);
          const [ux, uz] = displacements(cut, nodeAt[j]);
          expect(
            near(defl.u[i], ux, wScale, 1e-6),
            `deflection ux at ${(i / PIECES).toFixed(3)}: ${defl.u[i]} vs ${ux}`
          ).toBe(true);
          expect(
            near(defl.w[i], uz, wScale, 1e-6),
            `deflection uz at ${(i / PIECES).toFixed(3)}: ${defl.w[i]} vs ${uz}`
          ).toBe(true);
        }
      });
    }
  }
});

/** Turns a point (x, z) about the origin by `r` radians. */
const turn = (x: number, z: number, r: number): [number, number] => [
  x * Math.cos(r) - z * Math.sin(r),
  x * Math.sin(r) + z * Math.cos(r),
];

describe("turning the whole structure, its supports and its loads changes no internal force", () => {
  for (const degrees of [17, 90, 145, -60]) {
    it(`${degrees} degrees`, () => {
      const r = (degrees * Math.PI) / 180;
      const straight = frame(LOADS);
      const turned = frame(LOADS, 1e32, false);

      for (const [label, node] of turned.domain.nodes) {
        const [x, , z] = straight.domain.getNode(label).coords;
        const [xt, zt] = turn(x, z, r);
        node.coords = [xt, 0, zt];
        // Supports turn with the structure: a node without axes of its own gets them now
        const lcs = straight.domain.getNode(label).lcs;
        const ex = lcs ? [lcs[0][0], lcs[0][2]] : [1, 0];
        const [ux, uz] = turn(ex[0], ex[1], r);
        if (node.bcs.size > 0) node.updateLcs({ locx: [ux, 0, uz], locy: [0, 1, 0] });
      }
      const lc = turned.loadCases[0];
      for (const load of lc.nodalLoadList) {
        const v = load.values as Record<number, number>;
        const [fx, fz] = turn(v[DofID.Dx] ?? 0, v[DofID.Dz] ?? 0, r);
        load.values = { [DofID.Dx]: fx, [DofID.Dz]: fz, [DofID.Ry]: v[DofID.Ry] ?? 0 };
      }
      for (const load of lc.elementLoadList) {
        const l = load as unknown as { lcs?: boolean; values?: number[]; startValues?: number[]; endValues?: number[] };
        if (l.lcs !== false) continue; // in the member's axes, which turn with it anyway
        const turnPair = (v: number[]) => {
          const [fx, fz] = turn(v[0], v[1], r);
          v[0] = fx;
          v[1] = fz;
        };
        if (l.startValues && l.endValues) {
          turnPair(l.startValues);
          turnPair(l.endValues);
        } else if (l.values) turnPair(l.values);
      }
      turned.solve();

      for (const [label, el] of straight.domain.elements) {
        const l = (el as Beam2D).computeGeo().l;
        for (const x of [0, 0.37 * l, 0.5 * l, l]) {
          const a = internal(straight, label, x);
          const b = internal(turned, label, x);
          expect(near(a.N, b.N, SCALE), `${label} N at ${x}`).toBe(true);
          expect(near(a.V, b.V, SCALE), `${label} V at ${x}`).toBe(true);
          expect(near(a.M, b.M, SCALE), `${label} M at ${x}`).toBe(true);
        }
      }
    });
  }
});

describe("a link: a member hinged at both ends, loaded along its span", () => {
  // Two cantilevers 3 m long hold a 4 m link between their tips, under 10 kN/m and a 6 kN point
  // load 1 m from its start. The link is simply supported on the tips: its end reactions are
  // 10*4/2 + 6*3/4 = 24.5 kN and 20 + 6/4 = 21.5 kN, and each goes into its cantilever.
  const build = (k: number) => {
    const solver = newSolver(k);
    const d = solver.domain;
    d.createNode("A", [0, 0, 0], [DofID.Dx, DofID.Dz, DofID.Ry]);
    d.createNode("B", [3, 0, 0]);
    d.createNode("C", [7, 0, 0]);
    d.createNode("D", [10, 0, 0], [DofID.Dx, DofID.Dz, DofID.Ry]);
    d.createBeam2D("AB", ["A", "B"], "m", "cs");
    d.createBeam2D("BC", ["B", "C"], "m", "cs", [true, true]);
    d.createBeam2D("CD", ["C", "D"], "m", "cs");
    solver.loadCases[0].createBeamElementUniformEdgeLoad("BC", [0, 10e3], false);
    solver.loadCases[0].createBeamConcentratedLoad("BC", [0, 6e3, 0, 1], false);
    solver.solve();
    return solver;
  };

  for (const k of [1e32, 1]) {
    it(k === 1 ? "with shear deformation" : "without shear deformation", () => {
      const s = build(k);
      const scale = 1e5;
      // Ends of the link carry no moment
      expect(near(internal(s, "BC", 0).M, 0, scale)).toBe(true);
      expect(near(internal(s, "BC", 4).M, 0, scale)).toBe(true);
      // Shear at its ends: the end reactions
      expect(Math.abs(internal(s, "BC", 0).V)).toBeCloseTo(24.5e3, 3);
      expect(Math.abs(internal(s, "BC", 4).V)).toBeCloseTo(21.5e3, 3);
      // Moment under the point load: 24.5*1 - 10*1^2/2 = 19.5 kNm
      expect(Math.abs(internal(s, "BC", 1).M)).toBeCloseTo(19.5e3, 3);
      // Fixing moments of the cantilevers
      expect(Math.abs(internal(s, "AB", 0).M)).toBeCloseTo(24.5e3 * 3, 2);
      expect(Math.abs(internal(s, "CD", 3).M)).toBeCloseTo(21.5e3 * 3, 2);
    });
  }
});

describe("a point load at the very end of its member", () => {
  for (const where of ["start", "end"] as const) {
    it(`at the ${where} is a nodal load there`, () => {
      const onMember = newSolver();
      const asNodal = newSolver();
      for (const s of [onMember, asNodal]) {
        s.domain.createNode("A", [0, 0, 0], [DofID.Dx, DofID.Dz, DofID.Ry]);
        s.domain.createNode("B", [4, 0, -1]);
        s.domain.createNode("C", [7, 0, -1], [DofID.Dz]);
        s.domain.createBeam2D("AB", ["A", "B"], "m", "cs");
        s.domain.createBeam2D("BC", ["B", "C"], "m", "cs");
      }
      const l = (onMember.domain.getElement("AB") as Beam2D).computeGeo().l;
      onMember.loadCases[0].createBeamConcentratedLoad("AB", [3e3, 8e3, 2e3, where === "start" ? 0 : l], false);
      asNodal.loadCases[0].createNodalLoad(where === "start" ? "A" : "B", {
        [DofID.Dx]: 3e3,
        [DofID.Dz]: 8e3,
        [DofID.Ry]: 2e3,
      });
      onMember.solve();
      asNodal.solve();

      for (const n of ["A", "B", "C"]) {
        const a = displacements(onMember, n);
        const b = displacements(asNodal, n);
        for (let i = 0; i < 3; i++) expect(near(a[i], b[i], 1e-2), `${n}[${i}]`).toBe(true);
      }
    });
  }
});

/**
 * The hand calculations of EduBeam's guide (docs/en/guide/verification.md), on an IPE 200 with
 * shear deformation (k = 1). Deflections are read at nodes, as the guide says to.
 */
describe("the closed-form cases of the EduBeam guide", () => {
  const beam = (l: number, supports: [DofID[], DofID[]], k = 1, mid = true) => {
    const s = newSolver(k);
    s.domain.createNode("1", [0, 0, 0], supports[0]);
    if (mid) s.domain.createNode("m", [l / 2, 0, 0]);
    s.domain.createNode("2", [l, 0, 0], supports[1]);
    if (mid) {
      s.domain.createBeam2D("a", ["1", "m"], "m", "cs");
      s.domain.createBeam2D("b", ["m", "2"], "m", "cs");
    } else s.domain.createBeam2D("a", ["1", "2"], "m", "cs");
    return s;
  };
  const udl = (s: LinearStaticSolver, q: number) => {
    for (const label of s.domain.elements.keys()) s.loadCases[0].createBeamElementUniformEdgeLoad(label, [0, q], false);
  };

  it("simply supported beam, uniform load", () => {
    const s = beam(6, [[DofID.Dx, DofID.Dz], [DofID.Dz]]);
    udl(s, 12e3);
    s.solve();
    expect(-reactions(s, "1")[1] / 1e3).toBeCloseTo(36, 6);
    expect(internal(s, "a", 3).M / 1e3).toBeCloseTo(54, 6);
    expect(displacements(s, "1")[2]).toBeCloseTo(-0.02647, 5);
    expect(displacements(s, "m")[1] * 1e3).toBeCloseTo(49.86, 2);
  });

  it("cantilever, tip load", () => {
    const s = beam(4, [[DofID.Dx, DofID.Dz, DofID.Ry], []], 1, false);
    s.loadCases[0].createNodalLoad("2", { [DofID.Dz]: 18e3 });
    s.solve();
    expect(-reactions(s, "1")[1] / 1e3).toBeCloseTo(18, 6);
    expect(Math.abs(reactions(s, "1")[2]) / 1e3).toBeCloseTo(72, 6);
    expect(Math.abs(displacements(s, "2")[2])).toBeCloseTo(0.03529, 5);
    expect(displacements(s, "2")[1] * 1e3).toBeCloseTo(94.42, 2);
  });

  it("fixed-fixed beam, uniform load", () => {
    const all = [DofID.Dx, DofID.Dz, DofID.Ry];
    const s = beam(6, [all, all]);
    udl(s, 12e3);
    s.solve();
    expect(internal(s, "a", 0).M / 1e3).toBeCloseTo(-36, 6);
    expect(internal(s, "a", 3).M / 1e3).toBeCloseTo(18, 6);
    expect(displacements(s, "m")[1] * 1e3).toBeCloseTo(10.16, 2);
  });

  it("propped cantilever, uniform load", () => {
    const s = beam(6, [[DofID.Dx, DofID.Dz, DofID.Ry], [DofID.Dz]], 1e32, false);
    udl(s, 12e3);
    s.solve();
    expect(-reactions(s, "1")[1] / 1e3).toBeCloseTo(45, 6);
    expect(-reactions(s, "2")[1] / 1e3).toBeCloseTo(27, 6);
    expect(internal(s, "a", 0).M / 1e3).toBeCloseTo(-54, 6);
    expect(internal(s, "a", 3.75).M / 1e3).toBeCloseTo((9 * 12 * 36) / 128, 6);
  });

  it("two-bar truss", () => {
    const s = newSolver(1);
    s.domain.createNode("1", [0, 0, 0], [DofID.Dx, DofID.Dz]);
    s.domain.createNode("2", [2, 0, -2]);
    s.domain.createNode("3", [4, 0, 0], [DofID.Dx, DofID.Dz]);
    s.domain.createBeam2D("a", ["1", "2"], "m", "cs", [true, true]);
    s.domain.createBeam2D("b", ["2", "3"], "m", "cs", [true, true]);
    s.loadCases[0].createNodalLoad("2", { [DofID.Dz]: 20e3 });
    s.solve();
    expect(internal(s, "a", 1).N / 1e3).toBeCloseTo(-14.1421, 3);
    expect(internal(s, "b", 1).N / 1e3).toBeCloseTo(-14.1421, 3);
    expect(reactions(s, "1").map((r) => r / 1e3)).toEqual([expect.closeTo(10, 6), expect.closeTo(-10, 6), 0]);
  });

  // EduBeam's temperature load: [uniform change, bottom minus top, 0]
  it("temperature gradient, free to curve: no forces, the span rises", () => {
    const s = beam(8, [[DofID.Dx, DofID.Dz], [DofID.Dz]]);
    for (const label of s.domain.elements.keys()) s.loadCases[0].createBeamTemperatureLoad(label, [0, -10, 0]);
    s.solve();
    expect(Math.abs(internal(s, "a", 2).M)).toBeLessThan(1e-6);
    expect(Math.abs(reactions(s, "1")[1])).toBeLessThan(1e-6);
    expect(displacements(s, "m")[1] * 1e3).toBeCloseTo(-4.8, 6);
  });

  it("temperature gradient, ends held from turning: a constant moment EI kappa", () => {
    const s = beam(8, [
      [DofID.Dx, DofID.Dz, DofID.Ry],
      [DofID.Dz, DofID.Ry],
    ]);
    for (const label of s.domain.elements.keys()) s.loadCases[0].createBeamTemperatureLoad(label, [0, -10, 0]);
    s.solve();
    const m = 210e9 * 1.943e-5 * 6e-4;
    for (const x of [0, 2, 4]) expect(Math.abs(internal(s, "a", x).M)).toBeCloseTo(m, 3);
    expect(Math.abs(displacements(s, "m")[1])).toBeLessThan(1e-12);
  });

  it("prescribed settlement of a propped cantilever", () => {
    const s = beam(6, [[DofID.Dx, DofID.Dz, DofID.Ry], [DofID.Dz]], 1e32, false);
    s.loadCases[0].createPrescribedDisplacement("2", { [DofID.Dz]: 0.01 });
    s.solve();
    const ei = 210e9 * 1.943e-5;
    const r = (3 * ei * 0.01) / 6 ** 3;
    expect(Math.abs(reactions(s, "2")[1])).toBeCloseTo(r, 3);
    expect(Math.abs(reactions(s, "1")[2])).toBeCloseTo(r * 6, 2);
  });
});

/**
 * A distributed load is the sum of point loads along the member, so the point load's solution -
 * checked above against members cut at the load, which is exact - integrated over the member must
 * give the distributed loads' fixed-end forces and deflected shape. On a short, deep member, where
 * shear deformation matters most.
 */
describe("distributed loads are point loads summed along the member", () => {
  // 10-point Gauss-Legendre on [0, 1]
  const nodes = [
    0.013046735741414, 0.067468316655507, 0.160295215850488, 0.283302302935376, 0.425562830509184, 0.574437169490816,
    0.716697697064624, 0.839704784149512, 0.932531683344493, 0.986953264258586,
  ];
  const weights = [
    0.033335672154344, 0.074725674575291, 0.109543181257991, 0.134633359654998, 0.147762112357376, 0.147762112357376,
    0.134633359654998, 0.109543181257991, 0.074725674575291, 0.033335672154344,
  ];

  for (const k of [1e32, 1]) {
    it(k === 1 ? "with shear deformation" : "without shear deformation", () => {
      const s = newSolver(k);
      s.domain.createNode("1", [0, 0, 0]);
      s.domain.createNode("2", [1.2, 0, -0.3]);
      const beam = s.domain.createBeam2D("e", ["1", "2"], "m", "cs");
      const l = beam.computeGeo().l;
      if (k === 1) expect(beam.computeShearFactor()).toBeGreaterThan(0.1);

      const q0 = [2e3, 7e3];
      const q1 = [-1e3, 13e3];
      const at = (t: number, i: 0 | 1) => q0[i] + (q1[i] - q0[i]) * t;
      const trapezoid = new BeamElementTrapezoidalEdgeLoad("e", s.domain, [q0[0], q0[1]], [q1[0], q1[1]], true);
      const uniform = new BeamElementUniformEdgeLoad("e", s.domain, [q0[0], q0[1]], true);

      // Sum the point loads, one per Gauss point. A point load's deflection has a kink where it
      // stands, so the deflection at x is integrated in two parts, either side of x.
      const point = (t: number, weight: number, q: (t: number, i: 0 | 1) => number) =>
        new BeamConcentratedLoad("e", s.domain, [q(t, 0) * l * weight, q(t, 1) * l * weight, 0, t * l], true);
      const sum = (q: (t: number, i: 0 | 1) => number) => {
        const f = [0, 0, 0, 0, 0, 0];
        nodes.forEach((t, g) =>
          point(t, weights[g], q)
            .getLoadVectorForClampedBeam()
            .forEach((v, i) => (f[i] += v))
        );
        const w = Array(11).fill(0);
        const u = Array(11).fill(0);
        for (let j = 0; j <= 10; j++) {
          const x = j / 10;
          for (const [from, to] of [
            [0, x],
            [x, 1],
          ]) {
            if (to <= from) continue;
            nodes.forEach((t, g) => {
              const c = point(from + (to - from) * t, weights[g] * (to - from), q).computeBeamDeflectionContrib(x);
              w[j] += c.w;
              u[j] += c.u;
            });
          }
        }
        return { f, w, u };
      };

      for (const [load, q] of [
        [trapezoid, at],
        [uniform, (_t: number, i: 0 | 1) => q0[i]],
      ] as const) {
        const expected = sum(q);
        const f = load.getLoadVectorForClampedBeam();
        const fScale = Math.max(...expected.f.map(Math.abs));
        f.forEach((v, i) =>
          expect(near(v, expected.f[i], fScale, 1e-9), `end force ${i}: ${v} vs ${expected.f[i]}`).toBe(true)
        );

        const wScale = Math.max(...expected.w.map(Math.abs), ...expected.u.map(Math.abs));
        for (let j = 0; j <= 10; j++) {
          const c = load.computeBeamDeflectionContrib(j / 10);
          expect(near(c.w, expected.w[j], wScale, 1e-6), `w at ${j / 10}: ${c.w} vs ${expected.w[j]}`).toBe(true);
          expect(near(c.u, expected.u[j], wScale, 1e-6), `u at ${j / 10}: ${c.u} vs ${expected.u[j]}`).toBe(true);
        }
      }
    });
  }
});
