/** The part of luqr 0.3 (an untyped package) that EigenValueDynamicSolver uses. */
declare module "luqr" {
  export const luqr: {
    /** LDLᵀ decomposition of a symmetric matrix; nothing when it fails. */
    decomposeLDL(matrix: import("mathjs").MathArray): { L: number[][]; d: number[] } | null | undefined;
  };
}
