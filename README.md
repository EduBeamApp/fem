# @edubeam/fem

A finite element solver for plane frames and trusses, written in TypeScript. It runs in the browser
and in Node, and it is the solver behind [EduBeam](https://www.edubeam.app): every diagram,
reaction and deflection the app shows comes from it.

Documentation: <https://www.edubeam.app/solver>

Formerly published from GitHub as `ts-fem`; import from `@edubeam/fem` instead.

## Install

```bash
npm install @edubeam/fem
```

## A simply supported beam

```ts
import { DofID, LinearStaticSolver, type Beam2D } from '@edubeam/fem'

const solver = new LinearStaticSolver()
const { domain } = solver

// SI units throughout: m, N, Pa. Steel and an IPE 200.
domain.createMaterial(1, { e: 210e9, g: 80.77e9, alpha: 1.2e-5, d: 7850 })
domain.createCrossSection(1, { a: 28.5e-4, iy: 1943e-8, iz: 142e-8, dyz: 0, h: 0.2, k: 5 / 6, j: 6.98e-8 })

// Nodes at [x, y, z] in metres, with the DOFs they hold fixed
domain.createNode('a', [0, 0, 0], [DofID.Dx, DofID.Dz]) // pin
domain.createNode('b', [6, 0, 0], [DofID.Dz]) // roller
domain.createBeam2D('1', ['a', 'b'], 1, 1)

// 12 kN/m along local z, which points down on a beam drawn left to right
solver.loadCases[0].createBeamElementUniformEdgeLoad('1', [0, 12_000], true)
solver.solve()

const lc = solver.loadCases[0]
const beam = domain.elements.get('1') as Beam2D
beam.computeBendingMomentAt(lc, 3) // 54000 N·m, qL²/8
beam.computeShearForceAt(lc, 0) // 36000 N, qL/2
```

The model lies in the x–z plane with z pointing down; sagging moments are positive; positions
along a member are in metres from its first node.

## What's in it

- **Analyses:** `LinearStaticSolver` (load cases) and `EigenValueDynamicSolver` (natural
  frequencies and mode shapes).
- **Elements:** `Beam2D`, a two-node Timoshenko beam with optional end hinges.
- **Loads:** nodal forces and moments, point and line loads along members (uniform and
  trapezoidal), temperature, prescribed displacements.
- **Results:** N, V and M anywhere along a member, deflections, end forces, nodal displacements
  and reactions.

The full list is at <https://www.edubeam.app/solver/reference>.

## Authors

Bořek Patzák (borek.patzak@gmail.com), with Jan Voříšek and contributors.

## License

GPL-3.0
