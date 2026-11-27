import test from 'node:test';
import assert from 'node:assert/strict';
import { createExecutor, PROGRAM_STATES } from '../js/executor.js';

function fixture() {
  const calls = [];
  const controller = {
    joints: [0],
    getJointAnglesRad() { return [...this.joints]; },
    setAllJoints(joints) { this.joints = [...joints]; },
  };
  const motionApi = {
    async planMOVJ(start, end, options) {
      calls.push({ type: 'MOVJ', start, end, options });
      return { ok: true, trajectory: [{ t: 0, joints: start }, { t: 0.001, joints: end }] };
    },
    async planMOVL(start, pose, options) {
      calls.push({ type: 'MOVL', start, pose, options });
      return { ok: true, trajectory: [{ t: 0, joints: start }, { t: 0.001, joints: [2] }] };
    },
    emergencyStop() { calls.push({ type: 'E-STOP' }); },
  };
  const gripper = {
    async close() { calls.push({ type: 'CLOSE' }); return { ok: true }; },
    async open() { calls.push({ type: 'OPEN' }); return { ok: true }; },
  };
  const executor = createExecutor({
    motionApi,
    controller,
    gripper,
    points: {
      HOME: { joints: [1], pose: { position: [0, 0, 0], quaternion: [0, 0, 0, 1] } },
      P_PLACE: { joints: [2], pose: { position: [1, 0, 0], quaternion: [0, 0, 0, 1] } },
    },
  });
  return { executor, controller, calls };
}

test('executor 执行运动、夹爪、速度与循环', async () => {
  const { executor, controller, calls } = fixture();
  const result = await executor.run([
    { type: 'SET_SPEED', speed: 50, line: 1 },
    { type: 'MOVJ', point: 'HOME', speed: null, line: 2 },
    { type: 'GRIP', action: 'CLOSE', line: 3 },
    { type: 'LOOP', count: 2, line: 4, body: [
      { type: 'MOVL', point: 'P_PLACE', speed: 20, line: 5 },
    ] },
    { type: 'GRIP', action: 'OPEN', line: 7 },
  ]);

  assert.equal(result.ok, true);
  assert.equal(executor.state, PROGRAM_STATES.DONE);
  assert.deepEqual(controller.joints, [2]);
  assert.equal(calls.filter((call) => call.type === 'MOVL').length, 2);
  assert.equal(calls.find((call) => call.type === 'MOVJ').options.speedRatio, 0.5);
  assert.equal(calls.find((call) => call.type === 'MOVL').options.speedRatio, 0.2);
  assert.ok(calls.some((call) => call.type === 'CLOSE'));
  assert.ok(calls.some((call) => call.type === 'OPEN'));
});

test('executor 急停会立即清空队列，复位后才可再运行', async () => {
  const { executor, calls } = fixture();
  const running = executor.run([
    { type: 'WAIT', seconds: 1, line: 1 },
    { type: 'MOVJ', point: 'HOME', speed: null, line: 2 },
  ]);

  await new Promise((resolve) => setTimeout(resolve, 20));
  executor.stop();
  const result = await running;

  assert.equal(result.ok, false);
  assert.equal(executor.state, PROGRAM_STATES.E_STOP);
  assert.equal(executor.queueLength, 0);
  assert.ok(calls.some((call) => call.type === 'E-STOP'));
  assert.equal((await executor.run([])).ok, false);
  assert.equal(executor.reset().ok, true);
  assert.equal(executor.state, PROGRAM_STATES.IDLE);
});
