export const PROGRAM_STATES = Object.freeze({
  IDLE: 'IDLE',
  RUNNING: 'RUNNING',
  PAUSED: 'PAUSED',
  DONE: 'DONE',
  E_STOP: 'E-STOP',
});

const now = () => globalThis.performance?.now?.() ?? Date.now();
const requestFrame = (callback) => (
  typeof globalThis.requestAnimationFrame === 'function'
    ? globalThis.requestAnimationFrame(callback)
    : globalThis.setTimeout(() => callback(now()), 16)
);
const cancelFrame = (id) => (
  typeof globalThis.cancelAnimationFrame === 'function'
    ? globalThis.cancelAnimationFrame(id)
    : globalThis.clearTimeout(id)
);

function expandInstructions(instructions, context = {}) {
  const expanded = [];
  for (const instruction of instructions ?? []) {
    if (instruction?.type !== 'LOOP') {
      expanded.push({ ...instruction, ...context });
      continue;
    }
    for (let iteration = 1; iteration <= instruction.count; iteration += 1) {
      expanded.push(...expandInstructions(instruction.body, {
        loopLine: instruction.line,
        loopIteration: iteration,
        loopCount: instruction.count,
      }));
    }
  }
  return expanded;
}

function normalizeTrajectoryResult(result) {
  if (Array.isArray(result)) return { ok: true, trajectory: result };
  if (!result || result.ok === false) {
    return { ok: false, message: result?.message ?? '轨迹规划失败' };
  }
  if (!Array.isArray(result.trajectory)) {
    return { ok: false, message: '轨迹规划器未返回 trajectory 数组' };
  }
  return result;
}

function getPoint(points, name) {
  if (typeof points === 'function') return points(name);
  if (points instanceof Map) return points.get(name) ?? points.get(name.toLowerCase());
  return points?.[name] ?? points?.[name.toLowerCase()];
}

function poseFromPoint(point) {
  if (point?.pose) return point.pose;
  if (Array.isArray(point?.position) && Array.isArray(point?.quaternion)) {
    return { position: point.position, quaternion: point.quaternion };
  }
  return null;
}

/**
 * DSL 状态机执行器。轨迹规划由 motionApi 完成，本类只负责调度、
 * 动画采样、暂停/续跑和可立即中断的急停。
 */
export class ProgramExecutor {
  constructor(options = {}) {
    if (!options.motionApi) throw new TypeError('createExecutor 缺少 motionApi');

    this.motionApi = options.motionApi;
    this.controller = options.controller ?? null;
    this.gripper = options.gripper ?? null;
    this.points = options.points ?? {};
    this.baseJointSpeed = options.baseJointSpeed ?? Math.PI / 2;
    this.baseLinearSpeed = options.baseLinearSpeed ?? 4;
    this.defaultSpeed = options.defaultSpeed ?? 100;
    this.onStateChange = options.onStateChange ?? (() => {});
    this.onInstruction = options.onInstruction ?? (() => {});

    this._state = PROGRAM_STATES.IDLE;
    this._queue = [];
    this._runId = 0;
    this._activeFrame = null;
    this._currentJoints = Array.isArray(options.initialJoints)
      ? [...options.initialJoints]
      : null;
    this._speed = this.defaultSpeed;
    this._activeInstruction = null;
    this.lastError = null;
  }

  get state() { return this._state; }
  get queueLength() { return this._queue.length; }
  get activeInstruction() { return this._activeInstruction; }
  get speed() { return this._speed; }

  _setState(state) {
    if (this._state === state) return;
    this._state = state;
    try {
      this.onStateChange(state, this);
    } catch (error) {
      console.error('[executor] onStateChange 回调失败', error);
    }
  }

  _isCurrent(runId) {
    return runId === this._runId && this._state !== PROGRAM_STATES.E_STOP;
  }

  _nextFrame() {
    return new Promise((resolve) => {
      const id = requestFrame((time) => {
        if (this._activeFrame?.id === id) this._activeFrame = null;
        resolve(time);
      });
      this._activeFrame = { id, resolve };
    });
  }

  _cancelFrameWait() {
    if (!this._activeFrame) return;
    const { id, resolve } = this._activeFrame;
    this._activeFrame = null;
    cancelFrame(id);
    resolve(now());
  }

  async _waitWhilePaused(runId) {
    while (this._state === PROGRAM_STATES.PAUSED && this._isCurrent(runId)) {
      await this._nextFrame();
    }
    return this._isCurrent(runId);
  }

  async _delay(seconds, runId) {
    let elapsed = 0;
    let previous = now();
    const target = Math.max(0, seconds * 1000);

    while (elapsed < target) {
      await this._nextFrame();
      const current = now();
      if (!this._isCurrent(runId)) return false;
      if (this._state === PROGRAM_STATES.RUNNING) elapsed += current - previous;
      previous = current;
    }
    return this._isCurrent(runId);
  }

  _readCurrentJoints() {
    if (typeof this.controller?.getJointAnglesRad === 'function') {
      return [...this.controller.getJointAnglesRad()];
    }
    if (this._currentJoints) return [...this._currentJoints];
    throw new Error('执行 MOVJ/MOVL 前无法获取当前关节角');
  }

  _applyJoints(joints) {
    if (!Array.isArray(joints) || !joints.every(Number.isFinite)) {
      throw new Error('轨迹采样点缺少有效的 joints');
    }
    this._currentJoints = [...joints];
    if (typeof this.controller?.setAllJoints === 'function') {
      this.controller.setAllJoints(joints);
    }
  }

  async _playTrajectory(trajectory, runId) {
    if (trajectory.length === 0) throw new Error('轨迹为空');
    let previousTime = 0;

    for (const sample of trajectory) {
      const joints = Array.isArray(sample) ? sample : sample?.joints;
      const sampleTime = Number.isFinite(sample?.t) ? sample.t : previousTime;
      const wait = Math.max(0, sampleTime - previousTime);
      if (wait > 0 && !await this._delay(wait, runId)) return false;
      if (!await this._waitWhilePaused(runId)) return false;
      this._applyJoints(joints);
      previousTime = sampleTime;
    }
    return this._isCurrent(runId);
  }

  async _executeMotion(instruction, runId) {
    const point = getPoint(this.points, instruction.point);
    if (!point) throw new Error(`第 ${instruction.line} 行：运行时找不到点位 ${instruction.point}`);

    const startJoints = this._readCurrentJoints();
    const speed = instruction.speed ?? this._speed;
    const speedRatio = speed / 100;
    let planned;

    if (instruction.type === 'MOVJ') {
      if (!Array.isArray(point.joints)) {
        throw new Error(`第 ${instruction.line} 行：点位 ${instruction.point} 缺少 joints，无法 MOVJ`);
      }
      planned = await this.motionApi.planMOVJ(startJoints, point.joints, {
        speed,
        speedRatio,
        maxJointSpeed: this.baseJointSpeed * speedRatio,
      });
    } else {
      const pose = poseFromPoint(point);
      if (!pose) {
        throw new Error(`第 ${instruction.line} 行：点位 ${instruction.point} 缺少位姿，无法 MOVL`);
      }
      planned = await this.motionApi.planMOVL(startJoints, pose, {
        speed: this.baseLinearSpeed * speedRatio,
        speedPercent: speed,
        speedRatio,
      });
    }

    if (!this._isCurrent(runId)) return false;
    const result = normalizeTrajectoryResult(planned);
    if (!result.ok) throw new Error(`第 ${instruction.line} 行：${result.message}`);
    return this._playTrajectory(result.trajectory, runId);
  }

  async _execute(instruction, runId) {
    switch (instruction.type) {
      case 'SET_SPEED':
        this._speed = instruction.speed;
        return true;
      case 'WAIT':
        return this._delay(instruction.seconds, runId);
      case 'GRIP': {
        if (!this.gripper) throw new Error(`第 ${instruction.line} 行：夹爪模块未接入`);
        const result = instruction.action === 'CLOSE'
          ? await this.gripper.close()
          : await this.gripper.open();
        if (result?.ok === false) {
          throw new Error(`第 ${instruction.line} 行：${result.message ?? '夹爪动作失败'}`);
        }
        return this._isCurrent(runId);
      }
      case 'MOVJ':
      case 'MOVL':
        return this._executeMotion(instruction, runId);
      default:
        throw new Error(`第 ${instruction.line ?? '?'} 行：执行器不支持 ${instruction.type}`);
    }
  }

  async run(instructions) {
    if (this._state === PROGRAM_STATES.RUNNING || this._state === PROGRAM_STATES.PAUSED) {
      return { ok: false, state: this._state, message: '程序正在运行' };
    }
    if (this._state === PROGRAM_STATES.E_STOP) {
      return { ok: false, state: this._state, message: '急停已触发，请先复位' };
    }
    if (!Array.isArray(instructions)) {
      return { ok: false, state: this._state, message: 'instructions 必须是数组' };
    }

    const runId = ++this._runId;
    this._queue = expandInstructions(instructions);
    this._speed = this.defaultSpeed;
    this.lastError = null;
    this._setState(PROGRAM_STATES.RUNNING);

    try {
      while (this._queue.length && this._isCurrent(runId)) {
        if (!await this._waitWhilePaused(runId)) break;
        const instruction = this._queue.shift();
        this._activeInstruction = instruction;
        try {
          this.onInstruction(instruction, this);
        } catch (error) {
          console.error('[executor] onInstruction 回调失败', error);
        }
        if (!await this._execute(instruction, runId)) break;
      }

      this._activeInstruction = null;
      if (!this._isCurrent(runId)) {
        return { ok: false, state: this._state, message: '程序已急停' };
      }
      this._setState(PROGRAM_STATES.DONE);
      return { ok: true, state: this._state, message: '程序执行完成' };
    } catch (error) {
      this._queue.length = 0;
      this._activeInstruction = null;
      this.lastError = error;
      if (this._state !== PROGRAM_STATES.E_STOP) this._setState(PROGRAM_STATES.DONE);
      return { ok: false, state: this._state, message: error.message, error };
    }
  }

  pause() {
    if (this._state !== PROGRAM_STATES.RUNNING) return false;
    this._setState(PROGRAM_STATES.PAUSED);
    return true;
  }

  resume() {
    if (this._state !== PROGRAM_STATES.PAUSED) return false;
    this._setState(PROGRAM_STATES.RUNNING);
    return true;
  }

  /** 急停：当前轨迹不再应用新采样，并同步清空所有待执行指令。 */
  stop() {
    ++this._runId;
    this._queue.length = 0;
    this._activeInstruction = null;
    this._cancelFrameWait();
    try {
      this.motionApi.emergencyStop?.();
    } finally {
      this._setState(PROGRAM_STATES.E_STOP);
    }
    return { ok: true, state: this._state };
  }

  emergencyStop() { return this.stop(); }

  reset() {
    if (this._state === PROGRAM_STATES.RUNNING || this._state === PROGRAM_STATES.PAUSED) {
      return { ok: false, state: this._state, message: '运行中不能复位，请先急停' };
    }
    this._queue.length = 0;
    this.lastError = null;
    this.motionApi.resetEmergencyStop?.();
    this._setState(PROGRAM_STATES.IDLE);
    return { ok: true, state: this._state };
  }
}

export function createExecutor(options) {
  return new ProgramExecutor(options);
}

export default createExecutor;
