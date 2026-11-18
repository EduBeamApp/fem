// vite.config.ts
import { resolve } from "path";
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
});
