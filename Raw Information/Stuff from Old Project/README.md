# Archived Stewart simulator

This is the previous interactive p5-based simulator, preserved separately from the active optimizer at the repository root. It is not loaded by the optimizer and does not provide the optimizer's workspace or cycle-demand calculations.

From the repository root run `python -m http.server 8000`, then open:

http://localhost:8000/Raw%20Information/Stuff%20from%20Old%20Project/index.html

The page loads `p5.min.js`, `quaternion.min.js` and `stewart.min.js` from this directory. It provides an interactive platform canvas, manual pose/layout controls and animation. Its legacy constraints and coordinate conventions are independent of the current optimizer; compatibility with the current optimizer's home-height and metadata fields has not been established.

To smoke-test visually, confirm the canvas renders, change a translation slider and start/pause animation. Check the browser console for errors. The script URLs can also be checked directly over the same HTTP server. This archive is not a calibrated hardware simulation.

## Rebuild the browser bundle

The checked-in `stewart.min.js` is generated from `stewart.js` using the versions locked in this directory's `package-lock.json`. The source's custom-layout initializer must be present in the browser bundle. To rebuild, run `npm ci` and `npm run build` from this directory, then commit the generated bundle with source changes. Build dependencies are esbuild, quaternion, bezier-js and Ajv; they are not required to serve either page. The browser entry exposes the Stewart constructor expected by the archived UI.
