import * as math from "mathjs";
import { Domain } from "./Domain";
import { Load } from "./Load";
import { EnumDictionary, DofID, LabelType } from ".";

/**
 * Implementation of concentrated nodal load
 */
export class NodalLoad extends Load {
  values: EnumDictionary<DofID, number>;
  /** True when the values are in the node's own c.s. (a rotated support's axes) rather than the global one. */
  lcs: boolean;

  constructor(node: LabelType, domain: Domain, values: EnumDictionary<DofID, number> = {}, lcs = false) {
    super(node, domain);
    this.values = values;
    this.lcs = lcs;
  }
  change(node: number, values: EnumDictionary<DofID, number>, lcs = this.lcs) {
    this.target = node.toString();
    this.values = values;
    this.lcs = lcs;
  }
  /**
   * The load vector, in the axes the node's unknowns are in.
   *
   * A node with a local c.s. (a rotated support) has its unknowns in that one. Values given in the
   * global c.s. are turned into it, f_n = T^T f_g - left as they are, a horizontal force would act
   * along the rotated support and be taken by it whole. Values given in the node's c.s. already are.
   */
  getLoadVector(): number[] {
    const dofs = this.domain.solver.getNodeDofIDs(this.target);
    const ans = Array<number>();
    for (const idof of dofs) {
      if (idof in this.values) {
        ans.push(this.values[idof as DofID]!);
      } else {
        ans.push(0.0);
      }
    }

    const node = this.domain.getNode(this.target);
    if (this.lcs || !node.hasLcs()) return ans;

    const t = node.getTransformationMtrx(dofs) as math.Matrix;
    return (math.multiply(math.transpose(t), ans) as math.Matrix).toArray() as number[];
  }
  getLocationArray(): number[] {
    return this.domain.solver.getNodeLocationArray(this.target, this.domain.solver.getNodeDofIDs(this.target));
  }
}
