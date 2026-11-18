import * as math from "mathjs";
import { condensation } from "./condensation";
import { Beam2D } from "./Beam2D";
import { BeamElementLoad } from "./BeamElementLoad";
import { Domain } from "./Domain";
import { LabelType } from ".";

/**
 * Implementation of Beam2d concentrated load
 * Beam2D lives in the XZ plane, so the only supported point moment is My.
 */
export class BeamConcentratedLoad extends BeamElementLoad {
  values: number[]; // Fx, Fz, My, distance x
  lcs: boolean;

  constructor(elem: LabelType, domain: Domain, values: number[], lcs: boolean) {
    super(elem, domain);
    this.values = values;
    this.lcs = lcs;
  }
  change(elem: LabelType, values: number[], lcs: boolean) {
    this.target = elem.toString();
    this.values = values;
    this.lcs = lcs;
  }
  getGlobalIntensities() {
    const fx = this.values[0];
    const fz = this.values[1];
    const my = this.values[2];
    if (this.lcs) {
      const geo = this.domain.getElement(this.target).computeGeo();
      const cos = geo.dx / geo.l;
      const sin = geo.dz / geo.l;
      return { fx: fx * cos - fz * sin, fz: fx * sin + fz * cos, my };
    } else {
      return { fx, fz, my };
    }
  }
  getLocalIntensities() {
    const fx = this.values[0];
    const fz = this.values[1];
    const my = this.values[2];
    const geo = this.domain.getElement(this.target).computeGeo();
    const l = geo.l;
    const dx = geo.dx;
    const dz = geo.dz;
    const cos = dx / l;
    const sin = dz / l;
    if (!this.lcs) {
      return {
        fx: fx * cos + fz * sin,
        fz: -fx * sin + fz * cos,
        my,
      };
    } else {
      return { fx, fz, my };
    }
  }

  getLoadVectorForClampedBeam(): Array<number> {
    const geo = this.domain.getElement(this.target).computeGeo();
    const f = this.getLocalIntensities();
    const fx = f.fx;
    const fz = f.fz;
    const my = f.my;
    const l = geo.l;

    const a = this.values[3];
    const b = l - a;
    // Exact for a member that deforms in shear too (Timoshenko); Phi = 0 gives the Euler-Bernoulli values
    const phi = (this.domain.getElement(this.target) as Beam2D).computeShearFactor();
    const d = 1 + phi;
    const l2 = l * l;
    const l3 = l2 * l;

    return [
      (-b / l) * fx,
      (-fz * b * (phi * l2 + 3 * a * b + b * b)) / (d * l3) - (6 * a * b * my) / (d * l3),
      (fz * a * b * (phi * l + 2 * b)) / (2 * d * l2) - (my * b * (phi * l - 2 * a + b)) / (d * l2),
      (-a / l) * fx,
      (-fz * a * (phi * l2 + a * a + 3 * a * b)) / (d * l3) + (6 * a * b * my) / (d * l3),
      (-fz * a * b * (phi * l + 2 * a)) / (2 * d * l2) - (my * a * (phi * l + a - 2 * b)) / (d * l2),
    ];
  }

  getLocationArray(): number[] {
    return this.domain.getElement(this.target).getLocationArray();
  }

  getLoadVector(): number[] {
    const elem = <Beam2D>this.domain.getElement(this.target);
    const t = elem.computeT();
    const f = this.getLoadVectorForClampedBeam();
    if (elem.hasHinges()) {
      const { a, b, kab, kbb } = condensation(elem.computeLocalStiffnessMtrx(true));
      let ans = [0, 0, 0, 0, 0, 0];

      const h1 = math.multiply(kab, math.inv(kbb));
      if (b.length == 1) {
        const flv = f[b[0]];
        for (let i = 0; i < a.length; i++) {
          ans[a[i]] = f[a[i]] - h1.get([i, 0]) * flv;
        }

        return math.multiply(math.multiply(math.transpose(t), ans), -1.0).toArray() as number[];
      } else {
        const help = math.subtract(math.subset(f, math.index(a)), math.multiply(h1, math.subset(f, math.index(b))));
        ans = math.subset(ans, math.index(a), help);
        return math.multiply(math.multiply(math.transpose(t), ans), -1.0).toArray() as number[];
      }
    } else {
      return math.multiply(math.multiply(math.transpose(t), f), -1.0).toArray() as number[];
    }
  }

  computeBeamDeflectionContrib(xl: number): { u: number; w: number } {
    const f = this.getLocalIntensities();
    const elem = this.domain.elements.get(this.target)!;
    const geo = elem.computeGeo();
    const l = geo.l;

    const Fxloc = f.fx;
    const Fzloc = f.fz;
    const Myloc = f.my;

    const e = this.domain.getElement(this.target).getMaterial().e;
    const area = this.domain.getElement(this.target).getCS().a;
    const iy = this.domain.getElement(this.target).getCS().iy;

    const a = this.values[3];
    const EI = e * iy;
    const EA = e * area;
    const x = xl * l;
    const phi = (elem as Beam2D).computeShearFactor();
    const P = Fzloc;
    const C = Myloc;

    let u = 0;
    if (x < a) u += (((l - a) / l) * Fxloc * x) / EA;
    else u += (((l - a) / l) * Fxloc * a) / EA - ((a / l) * Fxloc * (x - a)) / EA;

    // Held at both ends, with shear deformation (exact Timoshenko solutions, derived with sympy),
    // either side of the load: the force P across the member, then the couple C
    const d = EI * l ** 3 * (1 + phi);
    let w: number;
    if (x <= a) {
      w =
        (-P *
          x *
          (a - l) *
          (phi * phi * l ** 4 -
            2 * phi * a * a * l * l +
            phi * a * l ** 3 +
            3 * phi * a * l * l * x +
            phi * l ** 4 -
            2 * phi * l * l * x * x -
            6 * a * a * l * x +
            4 * a * a * x * x +
            6 * a * l * l * x -
            2 * a * l * x * x -
            2 * l * l * x * x)) /
          (12 * d) +
        (C * x * (a - l) * (-phi * a * l * l + phi * l * l * x - 3 * a * l * x + 2 * a * x * x + l * l * x)) / (2 * d);
    } else {
      w =
        (-P *
          a *
          (x - l) *
          (phi * phi * l ** 4 -
            2 * phi * a * a * l * l +
            3 * phi * a * l * l * x +
            phi * l ** 4 +
            phi * l ** 3 * x -
            2 * phi * l * l * x * x -
            2 * a * a * l * l -
            2 * a * a * l * x +
            4 * a * a * x * x +
            6 * a * l * l * x -
            6 * a * l * x * x)) /
          (12 * d) +
        (C *
          a *
          (x - l) *
          (-phi * a * l * l +
            phi * l * l * x -
            a * l * l -
            a * l * x +
            2 * a * x * x +
            2 * l * l * x -
            2 * l * x * x)) /
          (2 * d);
    }

    return { u: u, w: w };
  }
  computeBeamNContrib(x: number): number {
    const f = this.getLocalIntensities();
    const a = this.values[3];
    return x < a ? 0 : -f.fx;
  }
  computeBeamVContrib(x: number): number {
    const f = this.getLocalIntensities();
    const a = this.values[3];
    return x < a ? 0 : -f.fz;
  }
  computeBeamMContrib(x: number): number {
    const f = this.getLocalIntensities();
    const a = this.values[3];
    return x < a ? 0 : -f.fz * (x - a) - f.my;
  }
}
