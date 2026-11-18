# Changelog

## 0.3.0 (unreleased)

The first release on npm, and under a new name: `ts-fem` is now **`@edubeam/fem`**. Change your
imports from `'ts-fem'` to `'@edubeam/fem'`; the API is otherwise the same.

- Nodal loads on a node with rotated axes can be given in the node's own axes.
- Exact Timoshenko fixed-end forces and deflected shapes, checked against a verification suite of
  closed-form cases.
- `dist` is built on install and on publish instead of being committed.
- Type declarations are listed first in `exports`, so TypeScript finds them under every resolution mode.

## 0.2.3 and earlier

Installed from GitHub as `ts-fem`. Highlights: trapezoidal loads, concentrated moments on beams,
prescribed displacements, temperature loads, reactions.
