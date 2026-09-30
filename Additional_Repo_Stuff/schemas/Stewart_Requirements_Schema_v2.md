# Requirements v2 shape sketch

The adjacent `Stewart_Requirements_Schema_v2.json` is a human-readable sketch using values such as "number" and "min". It is **not a formal JSON Schema**, should not be submitted as requirements, and is not used by a schema validator. `requirements.js` performs runtime normalization and validation for both nested and flat formats.

Use [Sample_Requirements.json](../examples/Sample_Requirements.json) for runnable input and [the current input reference](../../docs/REQUIREMENTS.md) for fields, defaults, allowed ranges and precedence. The JSON sketch illustrates nested sections; its numeric constraint entries show parser defaults. Optional `servo_max_deg` supplies symmetric travel only when explicit travel bounds are absent.
