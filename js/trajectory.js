import { forwardKinematics } from './fk.js';

function normalizeQuat(q) {
  const length = Math.hypot(...q);
  if (!Number.isFinite(length) || length < 1e-12) {
    throw new TypeError('四元数必须是有效的非零数组');
  }
  return q.map((v) => v / length);
}

function slerpQuat(aInput, bInput, t) {
  const a = normalizeQuat(aInput);
  let b = normalizeQuat(bInput);
  let dot = a.reduce((sum, v, i) => sum + v * b[i], 0);

  // 选较短旋转方向
  if (dot < 0) {
    b = b.map((v) => -v);
    dot = -dot;
  }

  if (dot > 0.9995) {
    return normalizeQuat(a.map((v, i) => v + t * (b[i] - v)));
  }

  const theta = Math.acos(Math.min(1, dot));
  const sinTheta = Math.sin(theta);
  const wa = Math.sin((1 - t) * theta) / sinTheta;
  const wb = Math.sin(t * theta) / sinTheta;
  return a.map((v, i) => wa * v + wb * b[i]);
}

/**
 * MOVL: TCP 位置直线插补 + 姿态 SLERP + 每点 IK。
 * solveIK(pose, options) 返回 {ok, joints, ...}。
 */
export async function planMOVL(model, startJoints, targetPose, solveIK, options = {}) {
  const n = model.joints.length;
  if (!Array.isArray(startJoints) || startJoints.length !== n ||
      !startJoints.every(Number.isFinite)) {
    return { ok: false, message: `起点关节角必须是 ${n} 个有限弧度值` };
  }
  if (typeof solveIK !== 'function') {
    return { ok: false, message: '缺少 solveIK 求解函数' };
  }

  const startTcp = forwardKinematics(model, startJoints).tcp;
  const endPosition = targetPose?.position;
  const endQuaternion = targetPose?.quaternion;

  if (!Array.isArray(endPosition) || endPosition.length !== 3 ||
      !endPosition.every(Number.isFinite) ||
      !Array.isArray(endQuaternion) || endQuaternion.length !== 4 ||
      !endQuaternion.every(Number.isFinite)) {
    return { ok: false, message: '目标位姿格式无效，需要 position[3] 和 quaternion[4]' };
  }

  const samplePeriod = options.samplePeriod ?? 0.02;
  const speed = options.speed ?? 1; // 模型单位/秒
  const distance = Math.hypot(...endPosition.map((v, i) => v - startTcp.p[i]));
  const duration = options.duration ?? Math.max(0.1, distance / speed);

  if (!(samplePeriod > 0) || !(duration > 0) ||
      !Number.isFinite(samplePeriod) || !Number.isFinite(duration)) {
    return { ok: false, message: 'duration 和 samplePeriod 必须是有限正数' };
  }

  const count = Math.ceil(duration / samplePeriod);
  const trajectory = [];
  let seed = [...startJoints];

  for (let k = 0; k <= count; k++) {
    const t = Math.min(k * samplePeriod, duration);
    const s = t / duration;
    const u = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5;

    const pose = {
      position: startTcp.p.map((v, i) => v + (endPosition[i] - v) * u),
      quaternion: slerpQuat(startTcp.quat, endQuaternion, u),
    };

    const result = await solveIK(pose, {
      ...options.ik,
      initialJoints: seed,
    });

    if (!result?.ok) {
    return {
        ok: false,
        message: `MOVL 第 ${k + 1}/${count + 1} 个采样点 IK 失败：${result?.message ?? '目标不可达'}`,
        failedIndex: k,
        failedPose: pose,
        ikError: result?.error,
        ikJoints: result?.joints,
        partialTrajectory: trajectory,
  };
}

    seed = result.joints;
    trajectory.push({ t, joints: [...seed], pose });
  }

  return { ok: true, trajectory, duration };
}

/**
 * MOVJ 五次多项式轨迹。
 * 起点和终点速度、加速度均为 0。
 * @returns {{ok:boolean, trajectory?:{t:number,joints:number[]}[], duration?:number, message?:string}}
 */
export function planMOVJ(model, startJoints, endJoints, options = {}) {
  const n = model.joints.length;

  const validJoints = (q) =>
    Array.isArray(q) &&
    q.length === n &&
    q.every(Number.isFinite);

  if (!validJoints(startJoints) || !validJoints(endJoints)) {
    return { ok: false, message: `起点和终点必须是 ${n} 个有限弧度值` };
  }

  // 超限就明确失败，不悄悄把目标改到限位上
  for (let i = 0; i < n; i++) {
    if (startJoints[i] < model.lower[i] || startJoints[i] > model.upper[i] ||
        endJoints[i] < model.lower[i] || endJoints[i] > model.upper[i]) {
      return { ok: false, message: `关节 ${model.names[i]} 超出限位` };
    }
  }

  const samplePeriod = options.samplePeriod ?? 0.02; // 50 Hz
  const duration = options.duration ?? Math.max(
    0.5,
    ...startJoints.map((q, i) =>
      Math.abs(endJoints[i] - q) / (options.maxJointSpeed ?? Math.PI / 2)
    )
  );

  if (!(samplePeriod > 0) || !(duration > 0) ||
      !Number.isFinite(samplePeriod) || !Number.isFinite(duration)) {
    return { ok: false, message: 'duration 和 samplePeriod 必须是有限正数' };
  }

  const count = Math.ceil(duration / samplePeriod);
  const trajectory = [];

  for (let k = 0; k <= count; k++) {
    const t = Math.min(k * samplePeriod, duration);
    const s = t / duration;
    const u = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5;

    const joints = startJoints.map((q, i) =>
      q + (endJoints[i] - q) * u
    );

    trajectory.push({ t, joints });
  }

  return { ok: true, trajectory, duration };
}