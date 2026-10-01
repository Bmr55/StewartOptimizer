// Minimal DOM stand-in for the simulator panels: elements are looked up by id,
// remember their handlers and can be marked as the focused element.
export function createFakeDocument() {
  const byId = new Map();
  class FakeElement {
    constructor(tagName) {
      this.tagName = tagName.toUpperCase();
      this.children = [];
      this.handlers = {};
      this._value = '';
      this._id = null;
      this.textContent = '';
      this.className = '';
      this.hidden = false;
      this.classList = { toggle() {}, add() {}, remove() {}, contains() { return false; } };
    }
    get id() { return this._id; }
    set id(id) { this._id = id; byId.set(id, this); }
    get value() { return this._value; }
    set value(value) { this._value = String(value); }
    appendChild(child) { this.children.push(child); return child; }
    append(...nodes) { this.children.push(...nodes); }
    replaceChildren(...nodes) { this.children = nodes; }
    addEventListener(type, handler) { this.handlers[type] = handler; }
    setAttribute() {}
    dispatch(type, event = {}) { this.handlers[type]?.({ target: this, ...event }); }
  }
  // Document-level listeners (the view's keydown) are kept so tests can fire them.
  const documentHandlers = {};
  return {
    activeElement: null,
    documentHandlers,
    addEventListener(type, handler) { documentHandlers[type] = handler; },
    removeEventListener(type) { delete documentHandlers[type]; },
    dispatch(type, event = {}) { documentHandlers[type]?.(event); },
    createElement: tag => new FakeElement(tag),
    getElementById(id) {
      if (!byId.has(id)) new FakeElement('div').id = id;
      return byId.get(id);
    },
  };
}

// WebGL2 stand-in for the renderer: it records program and draw counts, the
// last value of each uniform (looked up by name), every buffer upload, and each
// draw with the data last uploaded before it. `setLost` fakes a lost context.
export function createFakeGL() {
  const calls = { createProgram: 0, deleteProgram: 0, drawArrays: 0, draws: [], uploads: [], uniforms: {} };
  let lost = false;
  let bound = null;
  let nextId = 0;
  const noop = () => {};
  const setUniform = (name, ...values) => { calls.uniforms[name] = values.length === 1 ? values[0] : values; };
  const draw = entry => calls.draws.push({ ...entry, data: calls.uploads.at(-1)?.data ?? [] });
  return {
    calls, setLost(value) { lost = value; },
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4, DEPTH_TEST: 5,
    ARRAY_BUFFER: 6, DYNAMIC_DRAW: 7, FLOAT: 8, LINES: 9, POINTS: 10, TRIANGLES: 11, STATIC_DRAW: 12,
    COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32,
    createShader: () => ({}), shaderSource: noop, compileShader: noop, getShaderParameter: () => true,
    createProgram() { calls.createProgram++; return { id: nextId++ }; }, attachShader: noop, linkProgram: noop,
    getProgramParameter: () => true, createBuffer: () => ({ id: nextId++ }), getUniformLocation: (program, name) => name,
    createVertexArray: () => ({ id: nextId++ }), bindVertexArray: noop, deleteVertexArray: noop, vertexAttribDivisor: noop,
    enable: noop, clearColor: noop, viewport: noop, clear: noop, useProgram: noop,
    bindBuffer(target, buffer) { bound = buffer; },
    bufferData(target, data) { calls.uploads.push({ buffer: bound, data: Array.from(data) }); },
    vertexAttribPointer: noop, enableVertexAttribArray: noop,
    uniform1f: setUniform, uniform2f: setUniform, uniform3f: setUniform,
    uniformMatrix4fv(name, transpose, data) { setUniform(name, Array.from(data)); },
    drawArrays(primitive, first, count) {
      calls.drawArrays++;
      draw({ primitive, count, size: calls.uniforms.uPointSize });
    },
    drawArraysInstanced(primitive, first, count, instances) { draw({ primitive, count, instances }); },
    deleteBuffer: noop, deleteProgram() { calls.deleteProgram++; }, isContextLost: () => lost, getError: () => 0,
  };
}
