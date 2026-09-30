/**
 * @license Stewart v1.2.0 8/18/2025
 * https://raw.org/research/inverse-kinematics-of-a-stewart-platform/
 *
 * Copyright (c) 2025, Robert Eisele (https://raw.org/)
 * Licensed under the MIT license.
 **/

import Quaternion from 'quaternion';
import { Bezier } from 'bezier-js';
import Ajv from 'ajv';
import layoutSchema from '../research/original/Stewart_Layout_Schema.json' with { type: 'json' };
/* !simple-compilation */

const ajv = new Ajv();
const validateLayout = ajv.compile(layoutSchema);

const TAU = 2 * Math.PI;

function getHexPlate(r_i, r_o, rot) {
  const ret = [];
  const a_2 = (2 * r_i - r_o) / Math.sqrt(3);
  for (let i = 0; i < 6; i++) {
    const phi = (i - i % 2) / 3 * Math.PI + rot;
    const ap = a_2 * ((i & 1) ? -1 : 1);

    ret.push({
      x: r_o * Math.cos(phi) + ap * Math.sin(phi),
      y: r_o * Math.sin(phi) - ap * Math.cos(phi)
    });
  }
  return ret;
}

function parseSVGPath(str) {

  const p = str.match(/[a-z]|[-+]?([0-9]*\.[0-9]+|[0-9]+)/ig) || [];

  const COMMANDS = "MmZzLlHhVvCcSsQqTtAa";
  const UPPERCASE = "MZLHVCSQTA";

  const segments = [];

  const cur = { x: 0, y: 0 };
  let start = null;
  let cmd = null;
  let prevCmd = null;
  let isRelative = false;

  while (p.length > 0) {

    if (COMMANDS.indexOf(p[0]) !== -1) {
      prevCmd = cmd;
      cmd = p.shift();
      isRelative = UPPERCASE.indexOf(cmd) === -1;
      cmd = cmd.toUpperCase();
    } else {
      if (cmd === null) {
        throw new Error("Invalid implicit command");
      }
      prevCmd = cmd; // For S and T
    }

    switch (cmd) {

      case 'M': {
        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          cur.x += x;
          cur.y += y;
        } else {
          cur.x = x;
          cur.y = y;
        }

        segments.push({ cmd: "move", x: cur.x, y: cur.y });

        // Reset start position
        start = { x: cur.x, y: cur.y };

        // Implicitly treat the initial move as a line command
        cmd = 'L';
        break;
      }

      case 'L': {
        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x += cur.x;
          y += cur.y;
        }

        segments.push({ cmd: "line", x1: cur.x, y1: cur.y, x2: x, y2: y });

        cur.x = x;
        cur.y = y;
        break;
      }

      case 'H': {
        let x = +p.shift();

        if (isRelative) {
          x += cur.x;
        }

        segments.push({ cmd: "line", x1: cur.x, y1: cur.y, x2: x, y2: cur.y });

        cur.x = x;
        break;
      }

      case 'V': {
        let y = +p.shift();

        if (isRelative) {
          y += cur.y;
        }

        segments.push({ cmd: "line", x1: cur.x, y1: cur.y, x2: cur.x, y2: y });

        cur.y = y;
        break;
      }

      case 'Z': {
        if (start) {
          segments.push({ cmd: "line", x1: cur.x, y1: cur.y, x2: start.x, y2: start.y });
          cur.x = start.x;
          cur.y = start.y;
        }
        start = null;
        cmd = null; // No implicit commands after path close
        break;
      }

      case 'C': {

        let x1 = +p.shift();
        let y1 = +p.shift();

        let x2 = +p.shift();
        let y2 = +p.shift();

        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x1 += cur.x;
          y1 += cur.y;

          x2 += cur.x;
          y2 += cur.y;

          x += cur.x;
          y += cur.y;
        }

        segments.push({
          cmd: "cubic",
          x0: cur.x, y0: cur.y, // Start
          x1: x1, y1: y1, // Control 1
          x2: x2, y2: y2, // Control 2
          x3: x, y3: y, // End
          bezier: new Bezier(cur.x, cur.y, x1, y1, x2, y2, x, y)
        });

        cur.x = x;
        cur.y = y;
        break;
      }

      case 'S': {

        // First control point is the reflection of the previous command.
        let x1, y1;
        if (prevCmd !== 'C' && prevCmd !== 'S') {
          // If prev command was not C or S, assume first control point is coincident with current point
          x1 = cur.x;
          y1 = cur.y;
        } else {
          // The first control point is assumed to be the reflection of the second control point of the previous command relative to current point
          x1 = cur.x + cur.x - segments[segments.length - 1].x2;
          y1 = cur.y + cur.y - segments[segments.length - 1].y2;
        }

        let x2 = +p.shift();
        let y2 = +p.shift();

        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x2 += cur.x;
          y2 += cur.y;

          x += cur.x;
          y += cur.y;
        }

        segments.push({
          cmd: "cubic",
          x0: cur.x, y0: cur.y, // Start
          x1: x1, y1: y1, // Control 1
          x2: x2, y2: y2, // Control 2
          x3: x, y3: y, // End
          bezier: new Bezier(cur.x, cur.y, x1, y1, x2, y2, x, y)
        });

        cur.x = x;
        cur.y = y;
        break;
      }

      case 'Q': {

        let x1 = +p.shift();
        let y1 = +p.shift();

        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x1 += cur.x;
          y1 += cur.y;

          x += cur.x;
          y += cur.y;
        }

        // Quadratic Bezier
        segments.push({
          cmd: "quadratic",
          x0: cur.x, y0: cur.y, // Start
          x1: x1, y1: y1, // Control 1
          x2: x, y2: y, // End
          bezier: new Bezier(cur.x, cur.y, x1, y1, x, y)
        });

        cur.x = x;
        cur.y = y;
        break;
      }

      case 'T': {

        // Control point is the reflection of the previous command.
        let x1, y1;
        if (prevCmd !== 'Q' && prevCmd !== 'T') {
          // If prev command was not C or S, assume first control point is coincident with current point
          x1 = cur.x;
          y1 = cur.y;
        } else {
          // The first control point is assumed to be the reflection of the second control point of the previous command relative to current point
          x1 = cur.x + cur.x - segments[segments.length - 1].x1;
          y1 = cur.y + cur.y - segments[segments.length - 1].y1;
        }

        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x += cur.x;
          y += cur.y;
        }

        segments.push({
          cmd: "quadratic",
          x0: cur.x, y0: cur.y, // Start
          x1: x1, y1: y1, // Control 1
          x2: x, y2: y, // End
          bezier: new Bezier(cur.x, cur.y, x1, y1, x, y)
        });

        cur.x = x;
        cur.y = y;
        break;
      }

      case 'A': {

        const rx = +p.shift();
        const ry = +p.shift();

        const axisRotation = +p.shift();
        const largeArcFlag = +p.shift();
        const sweepFlag = +p.shift();

        let x = +p.shift();
        let y = +p.shift();

        if (isRelative) {
          x += cur.x;
          y += cur.y;
        }

        segments.push({
          cmd: "arc",

          rx: rx, ry: ry, // Radius

          axisRotation: axisRotation,
          largeArcFlag: largeArcFlag,
          sweepFlag: sweepFlag,

          x: x, y: y // End
        });

        cur.x = x;
        cur.y = y;
        break;
      }

      default:
        throw new Error('Invalid SVG command ' + cmd);
    }
  }
  return segments;
}

function StewartAnimation(platform) {
  this.platform = platform;
  this.orientation = Quaternion.ONE;
  this.translation = [0, 0, 0];
  // Tracks which legs are outside the servo range or invalid
  this.outOfRangeLegs = [];

  this.start('wobble');
}

StewartAnimation.SVG = function (svg, box) {

  const PERSEC = 0.05; // 5units per sec
  const L = 0;
  const H = 0 - 10;

  const SCREEN_SIZE = 80; // 80x80

  const cur = { x: box.width / 2, y: box.height / 2, z: L };
  const ret = [];
  let s = null;

  function move(x, y, z) {

    const relX = (x - box.x) / box.width * SCREEN_SIZE - SCREEN_SIZE / 2;
    const relY = (y - box.y) / box.height * SCREEN_SIZE - SCREEN_SIZE / 2;

    const relCurX = (cur.x - box.x) / box.width * SCREEN_SIZE - SCREEN_SIZE / 2;
    const relCurY = (cur.y - box.y) / box.height * SCREEN_SIZE - SCREEN_SIZE / 2;

    ret.push({ orig: s.cmd, x: relX, y: relY, z: z, t: Math.hypot(relX - relCurX, relY - relCurY, z - cur.z) / PERSEC });

    cur.x = x;
    cur.y = y;
    cur.z = z;
  }

  const seg = parseSVGPath(svg);

  for (let i = 0; i < seg.length; i++) {

    s = seg[i];

    switch (s.cmd) {
      case 'move':
        move(cur.x, cur.y, H);
        move(s.x, s.y, H);
        move(s.x, s.y, L);
        break;
      case 'line':
        move(s.x2, s.y2, L);
        break;
      case 'quadratic':
      case 'cubic':
        const b = s.bezier.getLUT();

        for (let j = 0; j < b.length; j++) {
          move(b[j].x, b[j].y, L);
        }
        break;
      case 'arc':

        // https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
        const x1 = cur.x;
        const y1 = cur.y;

        const x2 = s.x;
        const y2 = s.y;

        const axisRotation = s.axisRotation;
        const largeArcFlag = s.largeArcFlag;
        const sweepFlag = s.sweepFlag;

        const rx = s.rx;
        const ry = s.ry;

        // Step 1: x1', y1'
        const x1_ = Math.cos(axisRotation) * (x1 - x2) / 2.0 + Math.sin(axisRotation) * (y1 - y2) / 2.0;
        const y1_ = -Math.sin(axisRotation) * (x1 - x2) / 2.0 + Math.cos(axisRotation) * (y1 - y2) / 2.0;

        // Step 2: cx', cy'
        const s_ = (largeArcFlag === sweepFlag ? -1 : 1) * Math.sqrt((rx * rx * ry * ry - rx * rx * y1_ * y1_ - ry * ry * x1_ * x1_) / (rx * rx * y1_ * y1_ + ry * ry * x1_ * x1_));

        const cx_ = s_ * rx * y1_ / ry;
        const cy_ = s_ * -ry * x1_ / rx;

        // Step 3: cx, cy
        const cx = (x1 + x2) / 2.0 + Math.cos(axisRotation) * cx_ - Math.sin(axisRotation) * cy_;
        const cy = (y1 + y2) / 2.0 + Math.sin(axisRotation) * cx_ + Math.cos(axisRotation) * cy_;


        // Step 4:

        const angleBetween = function (ux, uy, vx, vy) {

          const cosPhi = (ux * vx + uy * vy) / Math.sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));

          return (ux * vy < uy * vx ? -1 : 1) * Math.acos(cosPhi);
        };

        // initial angle
        const theta1 = angleBetween(
          1, 0,
          (x1_ - cx_) / rx, (y1_ - cy_) / ry);

        // angle delta
        let thetad = angleBetween(
          (x1_ - cx_) / rx, (y1_ - cy_) / ry,
          (-x1_ - cx_) / rx, (-y1_ - cy_) / ry);

        if (sweepFlag === 0 && thetad > 0) {
          thetad -= TAU;
        } else if (sweepFlag === 1 && thetad < 0) {
          thetad += TAU;
        }

        const steps = Math.ceil(Math.abs(thetad * Math.max(rx, ry)) / 2); // every two degree
        for (let j = 0; j <= steps; j++) {
          const phi = theta1 + thetad * (j / steps);

          const x = rx * Math.cos(phi);
          const y = ry * Math.sin(phi);

          const x_ = x * Math.cos(axisRotation) - y * Math.sin(axisRotation);
          const y_ = x * Math.sin(axisRotation) + y * Math.cos(axisRotation);

          move(cx + x_, cy + y_, L);
        }
    }
  }
  return StewartAnimation.Interpolate(ret);
};

StewartAnimation.Interpolate = function (data) {

  let duration = 0;
  for (let i = 1; i < data.length; i++) {
    duration += data[i].t;
  }

  return {
    duration: duration,
    pathVisible: true,
    next: null,
    fn: function (pct) {

      this.orientation = Quaternion.ONE;

      let pctStart = 0;

      for (let i = 1; i < data.length; i++) {

        const p = data[i];

        const pctEnd = pctStart + p.t / duration;

        if (pctStart <= pct && pct < pctEnd) {

          const scale = (pct - pctStart) / (pctEnd - pctStart);

          const prev = i === 0 ? data[0] : data[i - 1];

          this.translation[0] = prev.x + (p.x - prev.x) * scale;
          this.translation[1] = prev.y + (p.y - prev.y) * scale;
          this.translation[2] = prev.z + (p.z - prev.z) * scale;

          return;
        }
        pctStart = pctEnd;
      }

      // Set to last element in chain
      this.translation[0] = data[data.length - 1].x;
      this.translation[1] = data[data.length - 1].y;
      this.translation[2] = data[data.length - 1].z;
    },

  };
};

StewartAnimation.prototype = {
  cur: null,
  next: null,
  startTime: 0,
  platform: null,
  translation: null,
  orientation: null,
  pathVisible: true,
  toggleVisiblePath: function () {
    this.pathVisible = !this.pathVisible;
  },
  drawPath: function (p) {

    if (!this.pathVisible || !this.cur.pathVisible)
      return;

    p.beginShape();
    p.noFill();
    p.stroke(255, 0, 0);
    const steps = 100;
    for (let i = 0; i <= steps; i++) {
      this.cur.fn.call(this, i / steps, p);
      p.vertex(this.translation[0], this.translation[1], this.translation[2] + this.platform.T0[2]);
    }
    p.endShape();
  },
  start: function (t) {

    if (this.map[t]) {
      t = this.map[t];
    }

    if (!this.fn[t]) {
      console.log("Failed ", t);
      return;
    } else {
      this._start(this.fn[t], this.fn[t].next);
    }
  },

  _start: function (play, next) {
    if (play.start) {
      play.start.call(this);
    }
    this.cur = play;
    this.next = next; // Loop
    this.startTime = Date.now();
  },

  moveTo: function (nt, no, time, next) {

    const ot = this.translation.slice();
    const oo = this.orientation.clone();
    const tw = oo.slerp(no);

    this.cur = {
      duration: time,
      pathVisible: false,
      fn: function (pct) {
        this.orientation = tw(pct);
        this.translation = [
          ot[0] + pct * (nt[0] - ot[0]),
          ot[1] + pct * (nt[1] - ot[1]),
          ot[2] + pct * (nt[2] - ot[2])
        ];
      }
    };
    this.startTime = Date.now();
    this.next = next;
  },

  update: function (p) {

    const now = Date.now();

    let elapsed = (now - this.startTime) / this.cur.duration;

    if (elapsed > 1)
      elapsed = 1;

    // Update actual orientation + position
    this.cur.fn.call(this, elapsed, p);

    if (elapsed === 1 && this.cur.duration !== 0 && this.next !== null) {
      this.start(this.next);
    }

    this.platform.update(this.translation, this.orientation);
  },
  fn: {
    rotate: {
      duration: 4000,
      pathVisible: false,
      next: 'rotate',
      fn: function (pct) {
        const b = Math.pow(Math.sin(pct * Math.PI * 2 - Math.PI * 8), 5) / 2;

        this.translation[0] = 0;
        this.translation[1] = 0;
        this.translation[2] = 0;
        this.orientation = Quaternion.fromAxisAngle([0, 0, 1], b);
      }
    },
    tilt: {
      duration: 7000,
      pathVisible: false,
      next: 'tilt',
      fn: function (pct) {

        let a = 0;
        let z = 0;

        if (pct < 1 / 4) {
          pct = pct * 4;
          a = 0;
        } else if (pct < 1 / 2) {
          pct = (pct - 1 / 4) * 4;
          a = 1 * Math.PI / 3;
        } else if (pct < 3 / 4) {
          pct = (pct - 1 / 2) * 4;
          a = TAU / 3;
        } else {
          pct = (pct - 3 / 4) * 4;
          z = 1;
        }

        let x = 0;
        let y = 0;

        if (z === 0) {
          x = Math.sin(a);
          y = -Math.cos(a);
        }

        const b = Math.pow(Math.sin(pct * Math.PI * 2 - Math.PI * 8), 5) / 3;

        this.translation[0] = 0;
        this.translation[1] = 0;
        this.translation[2] = 0;
        this.orientation = Quaternion.fromAxisAngle([x, y, z], b);
      }
    },
    square: (function () {
      const tmp = StewartAnimation.Interpolate([
        { x: -30, y: -30, z: 0 + 10, t: 0 },
        { x: -30, y: 30, z: 0, t: 1000 },
        { x: 30, y: 30, z: +10, t: 1000 },
        { x: 30, y: -30, z: 0, t: 1000 },
        { x: -30, y: -30, z: 0 + 10, t: 1000 },
      ]);
      tmp.next = "square";
      return tmp;
    })(),
    wobble: {
      duration: 3000,
      pathVisible: false,
      next: 'wobble',
      fn: function (pct) {

        const b = pct * TAU;

        this.translation[0] = Math.cos(-b) * 13;
        this.translation[1] = Math.sin(-b) * 13;
        this.translation[2] = 0;
        this.orientation = new Quaternion(-13, -Math.cos(b), Math.sin(b), 0).normalize();
      }
    },
    breathe: {
      duration: 5000,
      pathVisible: false,
      next: 'breathe',
      fn: function (pct) {

        const y = (Math.exp(Math.sin(TAU * pct) - 1)) / (Math.E * Math.E - 1);

        this.translation = [0, 0, y * 50];
        this.orientation = Quaternion.ONE;
      }
    },
    eight: {
      duration: 3500,
      pathVisible: true,
      next: 'eight',
      fn: function (pct) {
        const t = (-0.5 + 2.0 * pct) * Math.PI;
        this.translation = [Math.cos(t) * 30, Math.sin(t) * Math.cos(t) * 30, 0];
        this.orientation = Quaternion.ONE;
      }
    },
    lissajous: {
      duration: 10000,
      pathVisible: true,
      next: 'lissajous',
      fn: function (pct) {
        this.translation = [(Math.sin(3 * pct * TAU) * 30), (Math.sin(pct * 2 * TAU) * 30), 0];
        this.orientation = Quaternion.ONE;
      }
    },
    helical: {
      duration: 5000,
      pathVisible: true,
      next: null,
      fn: function (pct) {
        pct = 1 - pct;
        this.translation = [(Math.cos(pct * Math.PI * 8) * 20), (Math.sin(pct * Math.PI * 8) * 20), pct * 20];
        this.orientation = Quaternion.ONE;
      }
    },
    mouse: {
      duration: 0,
      pathVisible: false,
      next: null,
      fn: function (pct, p) {
        this.translation = [(p.mouseX - 512) / 10, (p.mouseY - 382) / 10, 0];
        this.orientation = Quaternion.ONE;
      }
    }, /*
       perlin: (function() {
       
       const xoff = 0;
       const yoff = 0;
       
       
       return {
       duration: 0,
       fn: function(none, p) {
       
       const b = p.noise(xoff, xoff) * TAU;
       
       this.translation[0] = Math.cos(-b) * 13;
       this.translation[1] = Math.sin(-b) * 13;
       this.translation[2] = 0;
       this.orientation = new Quaternion(-13, -Math.cos(b), Math.sin(b), 0).normalize();
       
       
       xoff += 0.0001;
       yoff += 0.0001;
       }
       }
       })(),*/
    gamepad: (function () {

      let gamepadActive = false;
      if (window.addEventListener) {
        window.addEventListener("gamepadconnected", function (e) {
          gamepadActive = true;
        });

        window.addEventListener("gamepaddisconnected", function (e) {
          gamepadActive = false;
        });
      }

      return {
        duration: 0,
        pathVisible: false,
        next: null,
        start: function () {
          this.orientation = Quaternion.ONE;
          this.translation = [0, 0, 0];
          if (gamepadActive) {
            alert("Use the joysticks and L1 button");
          } else {
            alert("Plug in a Playstation or Xbox controller and use the joysticks");
          }
        },
        fn: function () {

          if (!gamepadActive) {
            return;
          }

          const gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
          const buttons = gamepads[0].buttons;
          const axes = gamepads[0].axes;

          if (buttons[6].value) { // Is L1 pressed?
            // Rotate around Z axis with joystick 2 left-right
            this.orientation = Quaternion.fromAxisAngle([0, 0, 1], -axes[3] * Math.PI / 6);
            this.translation = [0, 0, 0];
          } else {
            // Control with both joysticks
            const b = Math.atan2(-axes[3], -axes[2]);
            this.translation = [axes[1] * 30, axes[0] * 30, 0];
            this.orientation = new Quaternion(-13, -Math.cos(b), Math.sin(b), 0).normalize();
          }
        }
      };
    })()
  },
  map: {
    q: "square",
    w: "wobble",
    e: "eight",
    r: "rotate",
    t: "tilt",
    y: "lissajous",

    m: "mouse",
    g: "gamepad",
    b: "breathe",
    h: "helical",
    p: "perlin"
  }
};

function Stewart() { }

Stewart.prototype = {
  translation: null,
  orientation: null,

  drawBasePlate: null,
  drawPlatformPlate: null,

  rodLength: 0,
  hornLength: 0,
  hornDirection: 0,
  servoRange: null,
  servoRangeVisible: false,

  sinBeta: [], // Sin of Pan angle of motors in base plate
  cosBeta: [], // Cos of Pan angle of motors in base plate
  B: [], // base joints in base frame
  P: [], // platform joints in platform frame

  q: [], // vector from base origin to Pk
  l: [], // vector from B to P
  H: [], // servo horn end to mount the rod

  e: [], // helper parameter e_k = 2 h l_kz
  f: [], // helper parameter f_k = 2 h (cosβ l_kx + sinβ l_ky)
  g: [], // helper parameter g_k = ||l_k||² - (d² - h²)

  T0: [], // Initial offset

  init: function (opts) {

    this.rodLength = opts.rodLength;
    this.hornLength = opts.hornLength;
    this.hornDirection = opts.hornDirection;
    this.drawBasePlate = opts.drawBasePlate;
    this.drawPlatformPlate = opts.drawPlatformPlate;
    this.servoRange = opts.servoRange;
    this.servoRangeVisible = opts.servoRangeVisible;

    this.B = [];
    this.P = [];
    this.q = [];
    this.l = [];
    this.H = [];
    this.e = [];
    this.f = [];
    this.g = [];
    this.sinBeta = [];
    this.cosBeta = [];

    const legs = opts.getLegs.call(this);

    for (let i = 0; i < legs.length; i++) {
      this.B.push(legs[i].baseJoint);
      this.P.push(legs[i].platformJoint);
      this.sinBeta.push(Math.sin(legs[i].motorRotation));
      this.cosBeta.push(Math.cos(legs[i].motorRotation));
      this.q.push([0, 0, 0]);
      this.l.push([0, 0, 0]);
      this.H.push([0, 0, 0]);
      this.e.push(0);
      this.f.push(0);
      this.g.push(0);
    }

    if (opts.absoluteHeight) {
      this.T0 = [0, 0, 0];
    } else {
      this.T0 = [0, 0, Math.sqrt(this.rodLength * this.rodLength + this.hornLength * this.hornLength
        - Math.pow(this.P[0][0] - this.B[0][0], 2)
        - Math.pow(this.P[0][1] - this.B[0][1], 2))];
    }

  },

  initCustom: function (layout) {

    if (!validateLayout(layout)) {
      throw new Error('Invalid layout: ' + ajv.errorsText(validateLayout.errors));
    }

    this.rodLength = layout.rod_length;
    this.hornLength = layout.horn_length;

    this.servoRange = (layout.servo_range || [-90, 90]).map(function (d) {
      return d * Math.PI / 180;
    });

    const len = layout.base_anchors.length;

    this.B = layout.base_anchors.map(a => a.slice());
    this.P = layout.platform_anchors.map(a => a.slice());
    // Use general β-angle expressions
    this.sinBeta = layout.beta_angles.map(b => Math.sin(b));
    this.cosBeta = layout.beta_angles.map(b => Math.cos(b));

    this.q = Array.from({ length: len }, () => [0, 0, 0]);
    this.l = Array.from({ length: len }, () => [0, 0, 0]);
    this.H = Array.from({ length: len }, () => [0, 0, 0]);
    this.e = new Array(len).fill(0);
    this.f = new Array(len).fill(0);
    this.g = new Array(len).fill(0);

    this.T0 = [0, 0, Math.sqrt(this.rodLength * this.rodLength + this.hornLength * this.hornLength
      - Math.pow(this.P[0][0] - this.B[0][0], 2)
      - Math.pow(this.P[0][1] - this.B[0][1], 2))];

    this.drawBasePlate = function () { };
    this.drawPlatformPlate = function () { };
    this.servoRangeVisible = false;

  },
  initCircular: function (opts) {

    if (!opts)
      opts = {};

    const baseRadius = opts.baseRadius || 80; // 8cm
    const platformRadius = opts.platformRadius || 50; // 5cm

    // Circle segment s = alpha_deg / 180 * pi * R <=> alpha_deg = s / R / pi * 180 <=> alpha_rad = s / R
    const shaftDistance = (opts.shaftDistance || 20) / baseRadius;
    const anchorDistance = (opts.anchorDistance || 20) / baseRadius;

    const rodLength = opts.rodLength || 130;

    const hornLength = opts.hornLength || 50;
    const hornDirection = opts.hornDirection || 0;

    const servoRange = opts.servoRange || [-Math.PI / 2, Math.PI / 2];
    const servoRangeVisible = opts.servoRangeVisible === undefined ? false : opts.servoRangeVisible;

    const baseAnchors = [];
    const platformAnchors = [];
    const betaAngles = [];

    for (let i = 0; i < 6; i++) {

      const pm = ((i & 1) ? -1 : 1);
      const phiCut = (1 + i - i % 2) * Math.PI / 3;

      const phiB = (i + i % 2) * Math.PI / 3 + pm * shaftDistance / 2;
      const phiP = phiCut - pm * anchorDistance / 2;

      baseAnchors.push([Math.cos(phiB) * baseRadius, Math.sin(phiB) * baseRadius, 0]);
      platformAnchors.push([Math.cos(phiP) * platformRadius, Math.sin(phiP) * platformRadius, 0]);
      betaAngles.push(phiB + ((i + hornDirection) % 2) * Math.PI + Math.PI / 2);
    }

    this.initCustom({
      base_anchors: baseAnchors,
      platform_anchors: platformAnchors,
      beta_angles: betaAngles,
      horn_length: hornLength,
      rod_length: rodLength,
      servo_range: servoRange.map(function (r) { return r * 180 / Math.PI; })
    });

    this.servoRangeVisible = servoRangeVisible;

    this.drawBasePlate = function (p) {
      p.stroke(0);
      p.fill(0xFE, 0xF1, 0x35);
      p.ellipse(0, 0, 2 * baseRadius, 2 * baseRadius);
    };
    this.drawPlatformPlate = function (p) {
      p.stroke(0);
      p.fill(0x2A, 0xEC, 0xFD);
      p.ellipse(0, 0, 2 * platformRadius, 2 * platformRadius);
    };
  },

  initHexagonal: function (opts) {

    if (!opts)
      opts = {};

    const baseRadius = opts.baseRadius || 80; // 8cm
    const baseRadiusOuter = opts.baseRadiusOuter || 110; // 11cm

    const platformRadius = opts.platformRadius || 50; // 5cm
    const platformRadiusOuter = opts.platformRadiusOuter || 80; // 8cm
    const platformTurn = opts.platformTurn === undefined ? true : opts.platformTurn;

    const rodLength = opts.rodLength || 130;

    const hornLength = opts.hornLength || 50;
    const hornDirection = opts.hornDirection || 0;

    const shaftDistance = opts.shaftDistance || 20;
    const anchorDistance = opts.anchorDistance || 20;

    const baseInts = getHexPlate(baseRadius, baseRadiusOuter, 0);
    const platformInts = getHexPlate(platformRadius, platformRadiusOuter, platformTurn ? Math.PI : 0);

    const servoRange = opts.servoRange || [-Math.PI / 2, Math.PI / 2];
    const servoRangeVisible = opts.servoRangeVisible === undefined ? false : opts.servoRangeVisible;

    const basePoints = [];
    const platPoints = [];
    const motorAngle = [];

    for (let i = 0; i < 6; i++) {

      const midK = i | 1;
      const baseCx = baseInts[midK].x;
      const baseCy = baseInts[midK].y;
      const baseNx = baseInts[(midK + 1) % 6].x;
      const baseNY = baseInts[(midK + 1) % 6].y;

      const platCx = platformInts[midK].x;
      const platCy = platformInts[midK].y;
      const platNx = platformInts[(midK + 1) % 6].x;
      const platNY = platformInts[(midK + 1) % 6].y;

      let baseDX = baseNx - baseCx;
      let baseDY = baseNY - baseCy;
      const lenBaseSide = Math.hypot(baseDX, baseDY);

      const pm = ((i & 1) ? -1 : 1);

      const baseMidX = (baseCx + baseNx) / 2;
      const baseMidY = (baseCy + baseNY) / 2;

      const platMidX = (platCx + platNx) / 2;
      const platMidY = (platCy + platNY) / 2;

      baseDX /= lenBaseSide;
      baseDY /= lenBaseSide;

      basePoints.push([baseMidX + baseDX * shaftDistance * pm, baseMidY + baseDY * shaftDistance * pm, 0]);
      platPoints.push([platMidX + baseDX * anchorDistance * pm, platMidY + baseDY * anchorDistance * pm, 0]);
      motorAngle.push(Math.atan2(baseDY, baseDX) + ((i + hornDirection) % 2) * Math.PI);
    }

    let platformIndex = [0, 1, 2, 3, 4, 5];

    if (platformTurn) {
      platformIndex = [4, 3, 0, 5, 2, 1];
    }

    const baseAnchors = basePoints;
    const platformAnchors = [];
    for (let i = 0; i < platformIndex.length; i++) {
      platformAnchors.push(platPoints[platformIndex[i]]);
    }

    this.initCustom({
      base_anchors: baseAnchors,
      platform_anchors: platformAnchors,
      beta_angles: motorAngle,
      horn_length: hornLength,
      rod_length: rodLength,
      servo_range: servoRange.map(function (r) { return r * 180 / Math.PI; })
    });

    this.servoRangeVisible = servoRangeVisible;

    this.drawBasePlate = function (p) { // Called periodically
      p.stroke(0);
      p.fill(0xFE, 0xF1, 0x35);

      p.beginShape();
      for (let i = 0; i < baseInts.length; i++) {
        p.vertex(baseInts[i].x, baseInts[i].y);
      }
      p.endShape(p.CLOSE);
    };
    this.drawPlatformPlate = function (p) { // Called periodically
      p.stroke(0);
      p.fill(0x2A, 0xEC, 0xFD);

      p.beginShape();
      for (let i = 0; i < platformInts.length; i++) {
        p.vertex(platformInts[i].x, platformInts[i].y);
      }
      p.endShape(p.CLOSE);
    };
  },

  draw: (function () {

    function drawCone(p, radius, h) {

      const sides = 12;
      let angle = 0;
      const angleIncrement = TAU / sides;
      p.beginShape(p.TRIANGLE_STRIP);
      for (let i = 0; i <= sides; i++) {
        p.vertex(0, 0, 0);
        p.vertex(radius * Math.cos(angle), h, radius * Math.sin(angle));
        angle += angleIncrement;
      }
      p.endShape();

      angle = 0;
      p.beginShape(p.TRIANGLE_FAN);

      p.vertex(0, h, 0);
      for (let i = 0; i < sides + 1; i++) {
        p.vertex(radius * Math.cos(angle), h, radius * Math.sin(angle));
        angle += angleIncrement;
      }
      p.endShape();
    }

    function drawFrame(p) {

      const w = 40;
      const ch = 10; // cone head

      // Draw 3 lines
      p.push();
      p.strokeWeight(2);
      p.stroke(255, 0, 0); // rot=x
      p.line(0, 0, 0, w, 0, 0);
      p.stroke(0, 255, 0); // grün=y
      p.line(0, 0, 0, 0, w, 0);
      p.stroke(0, 0, 255); // blau=z
      p.line(0, 0, 0, 0, 0, w);
      p.pop();

      // Red Cone
      p.push();
      p.noStroke();
      p.fill(255, 0, 0);
      p.rotateZ(Math.PI / 2);
      p.translate(0, -w - ch, 0);
      drawCone(p, 3, ch);
      p.pop();

      // Green Cone
      p.push();
      p.noStroke();
      p.fill(0, 255, 0);
      p.rotateX(-Math.PI);
      p.translate(0, -w - ch, 0);
      drawCone(p, 3, ch);
      p.pop();

      // Blue Cone
      p.push();
      p.noStroke();
      p.fill(0, 0, 255);
      p.rotateX(-Math.PI / 2);
      p.translate(0, -w - ch, 0);
      drawCone(p, 3, ch);
      p.pop();
    }

    return function (p) {

      // Base Frame
      drawFrame(p);

      // Base plate
      this.drawBasePlate.call(this, p);

      p.push();
      p.translate(this.translation[0], this.translation[1], this.translation[2] + this.T0[2]);
      p.applyMatrix.apply(p, this.orientation.conjugate().toMatrix4());
      this.drawPlatformPlate.call(this, p);
      drawFrame(p);
      p.pop();

      // Base joints
      p.noStroke();
      p.fill(0);
      for (let i = 0; i < this.B.length; i++) {
        p.push();
        p.translate(this.B[i][0], this.B[i][1], this.B[i][2]);
        p.sphere(3);
        p.pop();
      }

      // Platform joints
      p.noStroke();
      p.fill(255, 0, 0);
      for (let i = 0; i < this.q.length; i++) {
        p.push();
        p.translate(this.q[i][0], this.q[i][1], this.q[i][2]);
        p.sphere(3);
        p.pop();
      }

      // Rods with highlighting for out-of-range servos
      p.push();
      p.strokeWeight(1);
      for (let i = 0; i < this.B.length; i++) {
        const bad = this.outOfRangeLegs && this.outOfRangeLegs[i];
        if (bad) {
          p.stroke(255, 0, 0);
        } else {
          p.stroke(100); // neutral color
        }
        p.line(this.H[i][0], this.H[i][1], this.H[i][2], this.q[i][0], this.q[i][1], this.q[i][2]);
        p.line(this.B[i][0], this.B[i][1], this.B[i][2], this.H[i][0], this.H[i][1], this.H[i][2]);
      }
      p.pop();

      // --- Servo range arcs (need per-leg transform; batch style only) ---
      if (this.servoRangeVisible) {
        p.fill('rgba(255,0,0,0.1)');
        p.noStroke();
        for (let i = 0; i < this.B.length; i++) {
          p.push();
          p.translate(this.B[i][0], this.B[i][1], this.B[i][2]);
          p.rotateX(Math.PI / 2);
          p.rotateY(Math.atan2(this.H[i][1] - this.B[i][1], this.H[i][0] - this.B[i][0]));
          p.arc(0, 0, 2 * this.hornLength, 2 * this.hornLength,
            this.servoRange[0], this.servoRange[1], p.PIE);
          p.pop();
        }
      }

      // On-canvas warning for out-of-range servos
      if (this.outOfRangeLegs && this.outOfRangeLegs.some(Boolean)) {
        const badIdx = [];
        for (let i = 0; i < this.outOfRangeLegs.length; i++) {
          if (this.outOfRangeLegs[i]) badIdx.push(i + 1);
        }
        // Optional console warning to aid debugging or provide an external UI hook
        console.warn(`Out-of-range servos: ${badIdx.join(', ')}`);
        p.push();
        p.resetMatrix();
        p.translate(-p.width / 2, -p.height / 2);
        p.fill(255, 0, 0);
        p.noStroke();
        p.text(`Out-of-range servos: ${badIdx.join(', ')}`, 10, 20);
        p.pop();
      }
    };

  })(),

  update: function (translation, orientation) {

    const hornLength = this.hornLength;
    const rodLength = this.rodLength;

    const q = this.q;
    const l = this.l;
    const B = this.B;
    const P = this.P;
    const H = this.H;
    const z = this.T0[2];

    this.translation = translation;
    this.orientation = orientation;

    // Calculate H, q and l
    for (let i = 0; i < B.length; i++) {

      const o = orientation.rotateVector(P[i]);

      const li = l[i];
      const qi = q[i];
      const Hi = H[i];
      const Bi = B[i];

      qi[0] = translation[0] + o[0];
      qi[1] = translation[1] + o[1];
      qi[2] = translation[2] + o[2] + z;

      li[0] = qi[0] - Bi[0];
      li[1] = qi[1] - Bi[1];
      li[2] = qi[2] - Bi[2];

      const gk = li[0] * li[0] + li[1] * li[1] + li[2] * li[2] - rodLength * rodLength + hornLength * hornLength;
      const ek = 2 * hornLength * li[2];
      const fk = 2 * hornLength * (this.cosBeta[i] * li[0] + this.sinBeta[i] * li[1]);

      this.g[i] = gk;
      this.e[i] = ek;
      this.f[i] = fk;

      const sqSum = ek * ek + fk * fk;
      const sqrt1 = Math.sqrt(Math.max(0, 1 - gk * gk / sqSum));
      const sqrt2 = Math.sqrt(sqSum);
      const sinAlpha = (gk * ek) / sqSum - (fk * sqrt1) / sqrt2;
      const cosAlpha = (gk * fk) / sqSum + (ek * sqrt1) / sqrt2;

      Hi[0] = Bi[0] + hornLength * cosAlpha * this.cosBeta[i];
      Hi[1] = Bi[1] + hornLength * cosAlpha * this.sinBeta[i];
      Hi[2] = Bi[2] + hornLength * sinAlpha;
    }

    // Compute servo angles and mark any legs outside the allowed range
    if (this.computeAngles) {
      this.computeAngles();
    }
  },

  computeAngles: function () {

    const ret = [];
    const out = [];
    for (let i = 0; i < this.B.length; i++) {
      const e = this.e[i];
      const f = this.f[i];
      const g = this.g[i];

      const denom = Math.sqrt(e * e + f * f);
      const ratio = g / denom;
      if (!isFinite(ratio) || Math.abs(ratio) > 1) {
        ret[i] = null;
        out[i] = true;
        continue;
      }

      const alpha = Math.asin(ratio) - Math.atan2(f, e);

      if (isNaN(alpha) || alpha < this.servoRange[0] || alpha > this.servoRange[1]) {
        ret[i] = null;
        out[i] = true;
      } else {
        ret[i] = alpha;
        out[i] = false;
      }
    }
    this.outOfRangeLegs = out;
    return ret;
  }

};

Stewart.Animation = StewartAnimation;

export default Stewart;
