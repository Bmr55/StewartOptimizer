// Compatibility adapter: preserves the original exportBest() download behavior.
// Headless callers should import src/optimization/optimizer.js, whose exportBest() returns JSON.
import { Optimizer as CoreOptimizer } from './src/optimization/optimizer.js';
import { download } from './src/ui/download.js';

export class Optimizer extends CoreOptimizer {
  exportBest(format = 'json') {
    const data = super.exportBest(format);
    if (data !== undefined) this.download(data, 'optimized_layout.json', 'application/json');
  }

  download(data, filename, mime) { download(data, filename, mime); }
}
