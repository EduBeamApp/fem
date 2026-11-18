import type * as math from "mathjs";

/** The parts of a hinged member's stiffness that static condensation works with. */
export interface Condensation {
  /** Indices of the end forces that remain. */
  a: number[];
  /** Indices of the released ones (the moments at hinges). */
  b: number[];
  kaa: math.Matrix;
  kab: math.Matrix;
  kbb: math.Matrix;
}

/**
 * The condensed parts of `Beam2D.computeLocalStiffnessMtrx(true)`. For a member with a hinge they are
 * always there; their absence would be a bug, so it throws rather than computing with undefined.
 */
export function condensation(rec: {
  a?: number[];
  b?: number[];
  kaa?: math.Matrix;
  kab?: math.Matrix;
  kbb?: math.Matrix;
}): Condensation {
  const { a, b, kaa, kab, kbb } = rec;
  if (!a || !b || !kaa || !kab || !kbb) {
    throw new Error("Static condensation needs a member with a hinge");
  }
  return { a, b, kaa, kab, kbb };
}
