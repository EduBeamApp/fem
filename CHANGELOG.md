# Changelog

## 0.3.2 (unreleased)

- The build is named after the package: `dist/fem.js` and `dist/fem.umd.cjs` instead of `ts-fem.*`, and a
  `<script>` tag defines `window.EduBeamFem` instead of `window["ts-fem"]`. Imports of `@edubeam/fem` are
  unchanged.

## 0.3.1 (2026-11-19)

- The solvers no longer write timings or eigenvalue diagnostics to the console; a library should stay
  quiet, and a page that solves many models (or renders them on a server) was flooded with
  "Solution took … [ms]".
- An element that refers to a missing node throws a `RangeError` naming both, without first printing to the
  console, and looking up a missing node, element, material or cross section says "Node 3 does not exist"
  (it read "Node label 3 does not exists").

## 0.3.0 (2026-11-18)

The first release on npm, and under a new name: `ts-fem` is now **`@edubeam/fem`**. Change your
imports from `'ts-fem'` to `'@edubeam/fem'`; the API is otherwise the same.

- Nodal loads on a node with rotated axes can be given in the node's own axes.
- Exact Timoshenko fixed-end forces and deflected shapes, checked against a verification suite of
  closed-form cases.
- `dist` is built on install and on publish instead of being committed.
- Type declarations are listed first in `exports` and import each other with explicit `.js` paths,
  so TypeScript finds them under `moduleResolution: node16`/`nodenext` as well as in bundlers.
- mathjs is no longer bundled into `dist` (384 kB down to 49 kB); it is installed as a dependency.
- `sideEffects: false`, so bundlers can drop what a project doesn't use.
- Typed more precisely, with no change at run time: `Node.lcs` is `number[][] | undefined`, as it always was in
  practice, and `Node.hasLcs()` narrows it, so `if (node.hasLcs()) node.lcs[0]` still compiles.

## 0.2.3 and earlier

Installed from GitHub as `ts-fem`. Highlights: trapezoidal loads, concentrated moments on beams,
prescribed displacements, temperature loads, reactions.
