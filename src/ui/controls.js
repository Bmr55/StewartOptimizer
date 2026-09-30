// Keep defaults separate from input values so explicit user edits survive JSON changes.
export function createControls(document) {
    const ballJointLimitInput = document.getElementById('ballJointLimit');
    let lastWorkspaceDefaults = null;
    let lastJointDefault = null;
    let lastHeightDefaults = null;
    function populateWorkspaceInputs(workspace, preserveEdits = false) {
        const mappings = [
            ['X', workspace.x],
            ['Y', workspace.y],
            ['Z', workspace.z],
            ['Rx', workspace.rx],
            ['Ry', workspace.ry],
            ['Rz', workspace.rz],
        ];
        for (const [prefix, range] of mappings) {
            const previous = lastWorkspaceDefaults?.[prefix.toLowerCase()];
            for (const [suffix, key] of [['Min', 'min'], ['Max', 'max'], ['Step', 'step']]) {
                const input = document.getElementById(`opt${prefix}${suffix}`);
                if (!preserveEdits || !previous || input.value === '' || Number(input.value) === previous[key]) {
                    input.value = range[key];
                }
            }
        }
        lastWorkspaceDefaults = workspace;
    }

    function populateRequirementsDefaults(parsed, preserveEdits = false) {
        populateWorkspaceInputs(parsed.workspace, preserveEdits);
        const jointDefault = parsed.normalized.ball_joint_max_deg;
        if (!preserveEdits || lastJointDefault === null || ballJointLimitInput.value === ''
            || Number(ballJointLimitInput.value) === lastJointDefault) {
            ballJointLimitInput.value = jointDefault;
        }
        lastJointDefault = jointDefault;
        const heights = parsed.normalized.home_height_bounds_mm ?? [50, 450];
        for (const [id, value, previous] of [
            ['homeHeightMin', heights[0], lastHeightDefaults?.[0]],
            ['homeHeightMax', heights[1], lastHeightDefaults?.[1]],
        ]) {
            const input = document.getElementById(id);
            if (!preserveEdits || previous === undefined || input.value === '' || Number(input.value) === previous) {
                input.value = value;
            }
        }
        lastHeightDefaults = heights.slice();
    }

    function readHomeHeightBounds() {
        const bounds = ['homeHeightMin', 'homeHeightMax'].map(id => Number(document.getElementById(id).value));
        if (!bounds.every(Number.isFinite) || bounds[0] <= 0 || bounds[1] < bounds[0]) {
            throw new Error('home_height_bounds_mm must be positive finite bounds with max >= min.');
        }
        return bounds;
    }

    function readWorkspaceRanges() {
        const fallback = lastWorkspaceDefaults || {
            x: { min: 0, max: 0, step: 1 },
            y: { min: 0, max: 0, step: 1 },
            z: { min: 0, max: 0, step: 1 },
            rx: { min: 0, max: 0, step: 1 },
            ry: { min: 0, max: 0, step: 1 },
            rz: { min: 0, max: 0, step: 1 },
        };
        const parseValue = (id, fallbackValue) => {
            const text = document.getElementById(id).value.trim();
            if (!text) return fallbackValue;
            const value = Number(text);
            if (!Number.isFinite(value)) throw new Error(`${id} must be a finite number.`);
            return value;
        };
        const read = (prefix, defaults) => {
            const min = parseValue(`opt${prefix}Min`, defaults.min);
            const max = parseValue(`opt${prefix}Max`, defaults.max);
            const step = parseValue(`opt${prefix}Step`, defaults.step);
            if (max < min || step <= 0) throw new Error(`${prefix} range requires max >= min and step > 0.`);
            return { min, max, step };
        };
        return {
            x: read('X', fallback.x),
            y: read('Y', fallback.y),
            z: read('Z', fallback.z),
            rx: read('Rx', fallback.rx),
            ry: read('Ry', fallback.ry),
            rz: read('Rz', fallback.rz),
        };
    }

        return { populateRequirementsDefaults, readWorkspaceRanges, readHomeHeightBounds };
}
