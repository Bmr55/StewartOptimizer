export function createServoRatingControls(document) {
  let previousDefaults = new Map();
  const field = id => document.getElementById(id);
  const entries = normalized => {
    const values = [
      ['servoTorqueRating', normalized.servo_torque_rating_nm ?? ''],
      ['servoSpeedRating', normalized.servo_speed_rating_deg_s ?? ''],
      ['servoContinuousTorqueRating', normalized.servo_continuous_torque_rating_nm ?? ''],
      ['servoRatingPolicy', normalized.servo_rating_policy ?? 'enforced'],
    ];
    for (let index = 0; index < 6; index++) {
      values.push([`servoTorque${index + 1}`, normalized.per_servo_ratings?.[index]?.torque_nm ?? '']);
      values.push([`servoSpeed${index + 1}`, normalized.per_servo_ratings?.[index]?.speed_deg_s ?? '']);
    }
    return values;
  };
  return {
    populate(normalized, preserveEdits = false) {
      const next = new Map(entries(normalized).map(([id, value]) => [id, String(value)]));
      for (const [id, value] of next) {
        const input = field(id);
        if (!preserveEdits || !previousDefaults.has(id) || input.value === previousDefaults.get(id)) {
          input.value = value;
        }
      }
      previousDefaults = next;
    },
    read() {
      const readPositive = (id, name) => {
        const text = field(id).value.trim();
        if (!text) return null;
        const value = Number(text);
        if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${name} must be a finite positive number.`);
        return value;
      };
      const per_servo_ratings = Array.from({ length: 6 }, (_, index) => ({
        torque_nm: readPositive(`servoTorque${index + 1}`, `Servo ${index + 1} torque rating`),
        speed_deg_s: readPositive(`servoSpeed${index + 1}`, `Servo ${index + 1} speed rating`),
      }));
      return {
        servo_torque_rating_nm: readPositive('servoTorqueRating', 'Shared servo torque rating'),
        servo_speed_rating_deg_s: readPositive('servoSpeedRating', 'Shared servo speed rating'),
        servo_continuous_torque_rating_nm: readPositive('servoContinuousTorqueRating', 'Shared continuous torque rating'),
        per_servo_ratings,
        servo_rating_policy: field('servoRatingPolicy').value,
      };
    },
    setDisabled(disabled) {
      for (const [id] of entries({})) field(id).disabled = disabled;
    },
  };
}
