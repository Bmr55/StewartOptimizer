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
    dispatch(type) { this.handlers[type]?.({ target: this }); }
  }
  return {
    activeElement: null,
    addEventListener() {},
    removeEventListener() {},
    createElement: tag => new FakeElement(tag),
    getElementById(id) {
      if (!byId.has(id)) new FakeElement('div').id = id;
      return byId.get(id);
    },
  };
}
