import * as math from "mathjs";
import { Beam2D } from "./Beam2D";
import { BeamElementLoad } from "./BeamElementLoad";
import { Domain } from "./Domain";
import { LabelType } from ".";

/**
 * Implementation of Beam2d trapezoidal (linearly varying) load
 */
export class BeamElementTrapezoidalEdgeLoad extends BeamElementLoad {
    startValues: [number, number]; // fx, fz at element start
    endValues: [number, number]; // fx, fz at element end
    lcs: boolean; // true if values provided in element local c.s.

    constructor(elem: LabelType, domain: Domain, startValues: [number, number], endValues: [number, number], lcs: boolean) {
        super(elem, domain);
        this.startValues = startValues;
        this.endValues = endValues;
        this.lcs = lcs;
    }

    change(elem: LabelType, startValues: [number, number], endValues: [number, number], lcs: boolean) {
        this.target = elem.toString();
        this.startValues = startValues;
        this.endValues = endValues;
        this.lcs = lcs;
    }

    getGlobalIntensities() {
        const start = { fx: this.startValues[0], fz: this.startValues[1] };
        const end = { fx: this.endValues[0], fz: this.endValues[1] };
        if (this.lcs) {
            const geo = this.domain.getElement(this.target).computeGeo();
            const cos = geo.dx / geo.l;
            const sin = geo.dz / geo.l;
            return {
                start: { fx: start.fx * cos - start.fz * sin, fz: start.fx * sin + start.fz * cos },
                end: { fx: end.fx * cos - end.fz * sin, fz: end.fx * sin + end.fz * cos },
            };
        }
        return { start, end };
    }

    getLocalIntensities() {
        const start = { fx: this.startValues[0], fz: this.startValues[1] };
        const end = { fx: this.endValues[0], fz: this.endValues[1] };
        if (!this.lcs) {
            const geo = this.domain.getElement(this.target).computeGeo();
            const cos = geo.dx / geo.l;
            const sin = geo.dz / geo.l;
            return {
                start: { fx: start.fx * cos + start.fz * sin, fz: -start.fx * sin + start.fz * cos },
                end: { fx: end.fx * cos + end.fz * sin, fz: -end.fx * sin + end.fz * cos },
            };
        }
        return { start, end };
    }

    // load vector in local c.s.
    getLoadVectorForClampedBeam(): Array<number> {
        const geo = this.domain.getElement(this.target).computeGeo();
        const l = geo.l;
        const intensities = this.getLocalIntensities();
        const fx0 = intensities.start.fx;
        const fx1 = intensities.end.fx;
        const fz0 = intensities.start.fz;
        const fz1 = intensities.end.fz;

        const fx1Node = (-l / 6) * (2 * fx0 + fx1);
        const fx2Node = (-l / 6) * (fx0 + 2 * fx1);

        // A uniform part fz0, whose fixed-end forces do not depend on shear deformation, and a part
        // rising linearly to dz at the end, whose do (Phi = 0 gives the Euler-Bernoulli values)
        const phi = (this.domain.getElement(this.target) as Beam2D).computeShearFactor();
        const dz = fz1 - fz0;
        const d = 1 + phi;

        const fz1Node = (-l / 2) * fz0 - (dz * l * (10 * phi + 9)) / (60 * d);
        const fz2Node = (-l / 2) * fz0 - (dz * l * (20 * phi + 21)) / (60 * d);

        const m1Node = (fz0 * l * l) / 12 + (dz * l * l * (5 * phi + 4)) / (120 * d);
        const m2Node = (-fz0 * l * l) / 12 - (dz * l * l * (5 * phi + 6)) / (120 * d);

        return [fx1Node, fz1Node, m1Node, fx2Node, fz2Node, m2Node];
    }

    getLocationArray(): number[] {
        return this.domain.getElement(this.target).getLocationArray();
    }

    getLoadVector(): number[] {
        const elem = this.domain.getElement(this.target) as Beam2D;
        const t = elem.computeT();
        const f = this.getLoadVectorForClampedBeam();
        if (elem.hasHinges()) {
            const stiffrec = elem.computeLocalStiffnessMtrx(true);
            let ans = [0, 0, 0, 0, 0, 0];

            const h1 = math.multiply(stiffrec.kab, math.inv(stiffrec.kbb));

            if (stiffrec.b.length == 1) {
                const flv = f[stiffrec.b[0]];
                for (let i = 0; i < stiffrec.a.length; i++) {
                    ans[stiffrec.a[i]] = f[stiffrec.a[i]] - h1.get([i, 0]) * flv;
                }
                return math.multiply(math.multiply(math.transpose(t), ans), -1.0).toArray() as number[];
            } else {
                const help = math.subtract(
                    math.subset(f, math.index(stiffrec.a)),
                    math.multiply(h1, math.subset(f, math.index(stiffrec.b)))
                );
                ans = math.subset(ans, math.index(stiffrec.a), help);
                return math.multiply(math.multiply(math.transpose(t), ans), -1.0).toArray() as number[];
            }
        } else {
            return math.multiply(math.multiply(math.transpose(t), f), -1.0).toArray() as number[];
        }
    }

    computeBeamDeflectionContrib(xl: number): { u: number; w: number } {
        const elem = this.domain.getElement(this.target) as Beam2D;
        const geo = elem.computeGeo();
        const l = geo.l;
        const intensities = this.getLocalIntensities();
        const fz0 = intensities.start.fz;
        const fz1 = intensities.end.fz;
        const deltaFz = fz1 - fz0;
        const xi = xl; // normalized position along the element

        // Held at both ends, with shear deformation (exact Timoshenko solutions, derived with sympy):
        // the uniform part fz0 and the part rising linearly to deltaFz at the end
        const phi = elem.computeShearFactor();
        const EI = elem.getMaterial().e * elem.getCS().iy;
        const x = xi * l;
        const uniform = (fz0 * l ** 4 * xi * (1 - xi) * (phi + xi - xi * xi)) / (24 * EI);
        const linear =
          (deltaFz * x * (x - l) *
            (-10 * phi * phi * l ** 3 - 10 * phi * phi * l * l * x - 9 * phi * l ** 3 - 24 * phi * l * l * x +
              6 * phi * l * x * x + 6 * phi * x ** 3 - 12 * l * l * x + 6 * l * x * x + 6 * x ** 3)) /
          (720 * EI * l * (1 + phi));
        const w = uniform + linear;

        // The bar held at both ends under its axial load: EA u'' = -fx(x), u(0) = u(l) = 0, for the
        // uniform part fx0 and the part rising linearly to fx1 - fx0 at the end
        const fx0 = intensities.start.fx;
        const deltaFx = intensities.end.fx - fx0;
        const EA = elem.getMaterial().e * elem.getCS().a;
        const u = ((fx0 * x * (l - x)) / 2 + (deltaFx * x * (l * l - x * x)) / (6 * l)) / EA;

        return { u, w };
    }

    computeBeamNContrib(x: number): number {
        const intensities = this.getLocalIntensities();
        const geo = this.domain.getElement(this.target).computeGeo();
        const l = geo.l;
        const fx0 = intensities.start.fx;
        const fx1 = intensities.end.fx;
        const deltaFx = fx1 - fx0;
        return -(fx0 * x + 0.5 * deltaFx * (x * x) / l);
    }

    computeBeamVContrib(x: number): number {
        const intensities = this.getLocalIntensities();
        const geo = this.domain.getElement(this.target).computeGeo();
        const l = geo.l;
        const fz0 = intensities.start.fz;
        const fz1 = intensities.end.fz;
        const deltaFz = fz1 - fz0;
        return -(fz0 * x + 0.5 * deltaFz * (x * x) / l);
    }

    computeBeamMContrib(x: number): number {
        const intensities = this.getLocalIntensities();
        const geo = this.domain.getElement(this.target).computeGeo();
        const l = geo.l;
        const fz0 = intensities.start.fz;
        const fz1 = intensities.end.fz;
        const deltaFz = fz1 - fz0;
        return -(0.5 * fz0 * x * x + (deltaFz / (6 * l)) * x * x * x);
    }
}
