import { Beam2D } from "./Beam2D";
import { CrossSection, CrossSectionParameters } from "./CrossSection";
import { Material, MaterialParameters } from "./Material";
import { Solver } from "./Solver";
import { DofID, LabelType } from ".";
import { Element } from "./Element";
import { Node } from "./Node";

/**
 * Class representing problem domain
 */
export class Domain {
  solver: Solver;
  nodes = new Map<string, Node>();
  elements = new Map<string, Element>();
  materials = new Map<string, Material>();
  crossSections = new Map<string, CrossSection>();

  /**
   * Constructor
   */
  constructor(solver: Solver) {
    this.solver = solver;
  }
  getNode(id: LabelType): Node {
    const _id = id.toString();
    const found = this.nodes.get(_id);
    if (found) {
      return found;
    } else {
      throw new RangeError(`Node ${id} does not exist`);
    }
  }

  getElement(id: LabelType): Element {
    const _id = id.toString();
    const found = this.elements.get(_id);
    if (found) {
      return found;
    } else {
      throw new RangeError(`Element ${id} does not exist`);
    }
  }

  getMaterial(id: LabelType): Material {
    const _id = id.toString();
    const found = this.materials.get(_id);
    if (found) {
      return found;
    } else {
      throw new RangeError(`Material ${id} does not exist`);
    }
  }
  getCS(id: LabelType): CrossSection {
    const _id = id.toString();
    const found = this.crossSections.get(_id);
    if (found) {
      return found;
    } else {
      throw new RangeError(`Cross section ${id} does not exist`);
    }
  }

  // class factory
  createNode(label: LabelType, coords: number[] = [0, 0, 0], bcs: Array<DofID> = []) {
    const ans = new Node(label, this, coords, bcs);
    this.nodes.set(label.toString(), ans);
    return ans;
  }
  createBeam2D(
    label: LabelType,
    nodes: Array<LabelType>,
    mat: LabelType,
    cs: LabelType,
    hinges: [boolean, boolean] = [false, false]
  ) {
    const ans = new Beam2D(label, this, nodes, mat, cs, hinges);

    this.elements.set(label.toString(), ans);
    return ans;
  }
  createMaterial(label: LabelType, params: MaterialParameters = {}) {
    const ans = new Material(label, params);
    this.materials.set(label.toString(), ans);
    return ans;
  }
  createCrossSection(label: LabelType, params: CrossSectionParameters = {}) {
    const ans = new CrossSection(label, params);
    this.crossSections.set(label.toString(), ans);
    return ans;
  }
}
