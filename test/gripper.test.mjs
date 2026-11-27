import test from 'node:test';
import assert from 'node:assert/strict';
import { GripperController, GRIPPER_STATES } from '../js/gripper.js';

class FakeVector3 {
  constructor(x, y, z) { this.x = x; this.y = y; this.z = z; }
  clone() { return new FakeVector3(this.x, this.y, this.z); }
  distanceTo(other) {
    return Math.hypot(this.x - other.x, this.y - other.y, this.z - other.z);
  }
}

function objectAt(x, y, z) {
  return {
    position: new FakeVector3(x, y, z),
    userData: { pickable: true },
    getWorldPosition(output) {
      Object.assign(output, this.position);
      return output;
    },
  };
}

test('夹爪合拢时附着最近工件，张开时保留世界位姿并释放', async () => {
  const tcp = objectAt(0, 0, 0);
  tcp.attach = (object) => { object.parent = tcp; };
  const scene = { attach: (object) => { object.parent = scene; } };
  const near = objectAt(0.2, 0, 0);
  const far = objectAt(2, 0, 0);
  const jawOpenings = [];
  const gripper = new GripperController({
    tcp,
    scene,
    candidates: [far, near],
    graspDistance: 1,
    setJawOpening: (opening) => jawOpenings.push(opening),
  });

  const closed = await gripper.close();
  assert.equal(closed.ok, true);
  assert.equal(gripper.state, GRIPPER_STATES.CLOSED);
  assert.equal(gripper.attachedObject, near);
  assert.equal(near.parent, tcp);
  assert.equal(near.userData.gripped, true);

  const opened = await gripper.open();
  assert.equal(opened.ok, true);
  assert.equal(gripper.state, GRIPPER_STATES.OPEN);
  assert.equal(gripper.attachedObject, null);
  assert.equal(near.parent, scene);
  assert.equal(near.userData.gripped, false);
  assert.deepEqual(jawOpenings, [0, 1]);
});
