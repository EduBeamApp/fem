import { describe, expect, test } from "vitest";
import { LinearStaticSolver } from "./LinearStaticSolver";
import { Beam2D } from "./Beam2D";
import { DofID } from ".";

/**
 * A nodal load is given in the global c.s., whatever the axes of the node it acts on.
 *
 * A simply supported beam, pinned at node 1 and on a roller at node 2, with a horizontal force at
 * the roller. Turned by any angle but zero, the roller cannot take a horizontal force - the vertical
 * part of its reaction would leave the beam out of moment equilibrium - so the pin takes it all and
 * the beam is in tension.
 */

const L = 4;
const P = 10e3;

const beam = (degrees: number) => {
  const solver = new LinearStaticSolver();
  const domain = solver.domain;
  const radians = (degrees * Math.PI) / 180;

  domain.createNode(1, [0, 0, 0], [DofID.Dx, DofID.Dz]);
  domain.createNode(2, [L, 0, 0], [DofID.Dx]);
  if (degrees) domain.getNode(2).updateLcs({ locx: [Math.cos(radians), 0, Math.sin(radians)], locy: [0, 1, 0] });
  domain.createBeam2D("1", [1, 2], 1, 1);
  domain.createMaterial("1", { e: 30e9, g: 12.5e9, alpha: 1e-5, d: 2500 });
  domain.createCrossSection("1", { a: 0.06, iy: 4.5e-4, iz: 1, dyz: 1, h: 0.3, k: 1e32, j: 1 });
  solver.loadCases[0].createNodalLoad(2, { [DofID.Dx]: P });
  solver.solve();

  return solver;
};

const reactions = (solver: LinearStaticSolver, node: number) => {
  const values = solver.domain.getNode(node).getReactions(solver.loadCases[0], true).values;

  return (Array.isArray(values) ? values : values.toArray()) as number[];
};

const normalForce = (solver: LinearStaticSolver) =>
  (solver.domain.getElement("1") as Beam2D).computeNormalForce(solver.loadCases[0], 1).N[0];

describe("a horizontal nodal load on a rotated roller", () => {
  test("is taken by the roller when it is not rotated", () => {
    const solver = beam(0);

    expect(reactions(solver, 2)[0]).toBeCloseTo(-P, 6);
    expect(normalForce(solver)).toBeCloseTo(0, 6);
  });

  test.each([30, 60, -45])(
    "goes to the pin and stretches the beam when the roller is turned by %i degrees",
    (degrees) => {
      const solver = beam(degrees);

      reactions(solver, 2).forEach((r) => expect(r).toBeCloseTo(0, 6));
      expect(reactions(solver, 1)[0]).toBeCloseTo(-P, 6);
      expect(normalForce(solver)).toBeCloseTo(P, 6);
    }
  );
});
