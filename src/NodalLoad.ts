import * as math from "mathjs";
import { Domain } from "./Domain";
import { Load } from "./Load";
import { EnumDictionary, DofID, LabelType } from ".";

/**
 * Implementation of concentrated nodal load
 */
export class NodalLoad extends Load {
  values: EnumDictionary<DofID, number>;

  constructor(node: LabelType, domain: Domain, values: EnumDictionary<DofID, number> = {}) {
    super(node, domain);
    this.values = values;
  }
  change(node: number, values: EnumDictionary<DofID, number>) {
    this.target = node.toString();
    this.values = values;
  }
  /**
   * The load vector, in the axes the node's unknowns are in.
   *
   * The values are given in the global c.s. A node with a local c.s. (a rotated support) has its
   * unknowns in that one, so the load is turned into it, f_n = T^T f_g - left as it is, a horizontal
   * force would act along the rotated support and be taken by it whole.
   */
  getLoadVector(): number[] {
    const dofs = this.domain.solver.getNodeDofIDs(this.target);
    const ans = Array<number>();
    for (const idof of dofs) {
      if (idof in this.values) {
        ans.push(this.values[idof]);
      } else {
        ans.push(0.0);
      }
    }

    const node = this.domain.getNode(this.target);
    if (!node.hasLcs()) return ans;

    const t = node.getTransformationMtrx(dofs) as math.Matrix;
    return (math.multiply(math.transpose(t), ans) as math.Matrix).toArray() as number[];
  }
  getLocationArray(): number[] {
    return this.domain.solver.getNodeLocationArray(this.target, this.domain.solver.getNodeDofIDs(this.target));
  }
}
