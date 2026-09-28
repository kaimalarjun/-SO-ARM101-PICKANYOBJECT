import { Type } from "typebox";
import { defineToolPlugin } from "openclaw/plugin-sdk/tool-plugin";

const endpoint = "http://127.0.0.1:8765";
async function call(method: string, params: object, signal?: AbortSignal) {
  const motion = !["status", "look", "stop"].includes(method);
  const beat = () => { void fetch(`${endpoint}/robot/heartbeat`, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
  }).catch(() => undefined); };
  if (motion) beat();
  const timer = motion ? setInterval(beat, 500) : undefined;
  try {
    const res = await fetch(`${endpoint}/robot/${method}`, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(params), signal,
    });
    const result = await res.json();
    if (!res.ok) throw new Error(`Robot rejected ${method}: ${String(result.error)}`);
    return result;
  } finally { if (timer) clearInterval(timer); }
}

const xyz = { x: Type.Number(), y: Type.Number(), z: Type.Number() };

export default defineToolPlugin({
  id: "kendra-so-arm101-tools",
  name: "SO-ARM101 Robot Tools",
  description: "Safety-gated loopback robot tools.",
  tools: (tool) => [
    tool({ name: "robot_status", description: "Check robot readiness and motion mode.",
      parameters: Type.Object({}), optional: true,
      execute: (_p, _c, ctx) => call("status", {}, ctx.signal) }),
    tool({ name: "robot_look", description: "Capture a fresh overhead image and find tabletop objects. Supply a requested object name when choosing a pick target.",
      parameters: Type.Object({ query: Type.Optional(Type.String()) }), optional: true,
      execute: (p, _c, ctx) => call("look", p, ctx.signal) }),
    tool({ name: "robot_home", description: "Plan a bounded move to the measured home pose.",
      parameters: Type.Object({}), optional: true,
      execute: (_p, _c, ctx) => call("home", {}, ctx.signal) }),
    tool({ name: "robot_stop", description: "Cancel motion immediately; stop remains latched.",
      parameters: Type.Object({}), optional: true,
      execute: (_p, _c, ctx) => call("stop", {}, ctx.signal) }),
    tool({ name: "robot_open_gripper", description: "Open the gripper through the controller.",
      parameters: Type.Object({}), optional: true,
      execute: (_p, _c, ctx) => call("open_gripper", {}, ctx.signal) }),
    tool({ name: "robot_close_gripper", description: "Close the gripper through the controller.",
      parameters: Type.Object({}), optional: true,
      execute: (_p, _c, ctx) => call("close_gripper", {}, ctx.signal) }),
    tool({ name: "robot_move_to", description: "Move to measured metres in base_link, within workspace limits.",
      parameters: Type.Object(xyz), optional: true,
      execute: (p, _c, ctx) => call("move_to", p, ctx.signal) }),
    tool({ name: "robot_pick", description: "Pick one observed object. Requires the fresh ID from robot_look and its measured coordinates.",
      parameters: Type.Object({ ...xyz, observation_id: Type.String() }), optional: true,
      execute: (p, _c, ctx) => call("pick", p, ctx.signal) }),
    tool({ name: "robot_place", description: "Place at a camera-checked empty location in metres in base_link.",
      parameters: Type.Object(xyz), optional: true,
      execute: (p, _c, ctx) => call("place", p, ctx.signal) }),
  ],
});
