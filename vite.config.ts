// vite.config.ts
import { resolve } from "path";
/// <reference types="vitest" />
import { defineConfig } from "vite";
import dts from "vite-plugin-dts";
// https://vitejs.dev/guide/build.html#library-mode
export default defineConfig({
  build: {
    lib: {
      entry: resolve(__dirname, "src/index.ts"),
      name: "ts-fem",
      fileName: "ts-fem",
    },
    rollupOptions: {
      // mathjs is a dependency, installed beside the package: bundling it as well would ship it twice
      external: ["mathjs"],
      output: { globals: { mathjs: "math" } },
    },
  },
  // The tests sit beside the sources; their types are no part of the package
  plugins: [dts({ exclude: ["**/*.test.ts"] })],
  test: {
    coverage: {
      provider: "v8",
      include: ["src/**/*.ts"],
      exclude: ["src/**/*.test.ts"],
      reporter: ["text", "html"],
      // The levels reached at 0.3.0, rounded down: coverage may grow, not shrink.
      thresholds: { statements: 79, branches: 93, functions: 78, lines: 79 },
    },
  },
});
