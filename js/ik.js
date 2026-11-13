import { forwardKinematics, clampJoint } from './fk.js';
import {planMOVJ} from './trajectory.js';
import { planMOVL } from './trajectory.js';

/**
 * 计算当前关节角对应的 TCP 位置误差。
 * qRad 和位置都使用模型单位；关节角使用弧度。
 */
export function positionError(model, targetPosition, qRad) {
  if (!Array.isArray(targetPosition) || targetPosition.length !== 3 ||
      !targetPosition.every(Number.isFinite)) {
    throw new TypeError('targetPosition 必须是包含 3 个有限数字的数组');
  }

  const currentPosition = forwardKinematics(model, qRad).tcp.p;

  const error = targetPosition.map((target, i) => target - currentPosition[i]);
  const errorNorm = Math.hypot(...error);

  return {
    currentPosition,
    error,
    errorNorm,
  };
}

/**
 * 用有限差分估算位置雅可比矩阵。
 * 返回 3×N 矩阵：行对应 X/Y/Z，列对应各关节。
 */
export function positionJacobian(model, qRad, delta = 1e-4) {
  const n = model.joints.length;
  const base = forwardKinematics(model, qRad).tcp.p;
  const J = [[], [], []];

  for (let j = 0; j < n; j++) {
    const qNext = [...qRad];
    qNext[j] += delta;

    const moved = forwardKinematics(model, qNext).tcp.p;
    for (let axis = 0; axis < 3; axis++) {
      J[axis][j] = (moved[axis] - base[axis]) / delta;
    }
  }

  return J;
}

function normalizeQuat(q) {
  const length = Math.hypot(...q);
  if (!Number.isFinite(length) || length < 1e-12) {
    throw new TypeError('四元数必须是有效的非零数组');
  }
  return q.map((v) => v / length);
}

function quatMultiply(a, b) {
  const [x1, y1, z1, w1] = a;
  const [x2, y2, z2, w2] = b;
  return [
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
  ];
}

function quatConjugate(q) {
  return [-q[0], -q[1], -q[2], q[3]];
}

/** 返回从 current 转到 target 的轴角误差向量，单位为弧度。 */
function orientationError(targetQuat, currentQuat) {
  const target = normalizeQuat(targetQuat);
  const current = normalizeQuat(currentQuat);
  let relative = normalizeQuat(
    quatMultiply(target, quatConjugate(current))
  );

  // q 和 -q 表示同一姿态，选较短的旋转方向
  if (relative[3] < 0) relative = relative.map((v) => -v);

  const vectorLength = Math.hypot(relative[0], relative[1], relative[2]);
  if (vectorLength < 1e-12) return [0, 0, 0];

  const angle = 2 * Math.atan2(vectorLength, relative[3]);
  return relative.slice(0, 3).map((v) => (v / vectorLength) * angle);
}

function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(M[row][col]) > Math.abs(M[pivot][col])) pivot = row;
    }
    if (Math.abs(M[pivot][col]) < 1e-12) {
      throw new Error('阻尼线性方程组仍然奇异');
    }
    [M[col], M[pivot]] = [M[pivot], M[col]];

    const divisor = M[col][col];
    for (let j = col; j <= n; j++) M[col][j] /= divisor;

    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const factor = M[row][col];
      for (let j = col; j <= n; j++) {
        M[row][j] -= factor * M[col][j];
      }
    }
  }

  return M.map((row) => row[n]);
}

/**
 * 位置 IK：输入目标位置和初始关节角，返回求得的关节角。
 * 暂不考虑目标姿态；关节角单位为弧度。
 */
export function solvePositionIK(model, target, initialJoints, options = {}) {
    const n = model.joints.length;

    if (!Array.isArray(initialJoints) || initialJoints.length !== n ||
        !initialJoints.every(Number.isFinite)) {
    throw new TypeError(`initialJoints 必须是 ${n} 个有限弧度值`);
    }

    if (!Array.isArray(target) || target.length !== 3 ||
        !target.every(Number.isFinite)) {
    throw new TypeError('target 必须是包含 3 个有限数字的位置数组');
    }

    let q = initialJoints.map((angle, i) => clampJoint(model, i, angle));

    const maxIterations = options.maxIterations ?? 100;
    const tolerance = options.tolerance ?? 1e-3;
    const delta = options.delta ?? 1e-4;
    const damping = options.damping ?? 1e-2;
    const stepScale = options.stepScale ?? 0.5;

    for (let i = 0; i < maxIterations; i++) {
        const { error, errorNorm } = positionError(model, target, q);

        if (errorNorm < tolerance) {
        return { ok: true, joints: q, iterations: i, error: errorNorm };
        }

        const J = positionJacobian(model, q, delta);

        // A = J*J^T + damping^2 * I
        const A = Array.from({ length: 3 }, (_, r) =>
        Array.from({ length: 3 }, (_, c) => {
            let sum = 0;
            for (let j = 0; j < q.length; j++) sum += J[r][j] * J[c][j];
            return sum + (r === c ? damping * damping : 0);
        })
        );

        const y = solveLinear(A, error);

        // Δq = J^T*y
        const dq = q.map((_, j) =>
        J.reduce((sum, row, axis) => sum + row[j] * y[axis], 0)
        );

        q = q.map((angle, j) =>
        clampJoint(model, j, angle + stepScale * dq[j])
);
    }

    const final = positionError(model, target, q);
    return {
        ok: final.errorNorm < tolerance,
        joints: q,
        iterations: maxIterations,
        error: final.errorNorm,
        message: final.errorNorm < tolerance ? undefined : '未收敛：目标可能不可达或迭代次数不足',
    };
}

export function solveIK(model, targetPose, options = {}) {
  const n = model.joints.length;
  const targetPosition = targetPose?.position;
  const targetQuaternion = targetPose?.quaternion;

  if (!Array.isArray(targetPosition) || targetPosition.length !== 3 ||
      !targetPosition.every(Number.isFinite)) {
    throw new TypeError('targetPose.position 必须是 3 个有限数字');
  }
  if (!Array.isArray(targetQuaternion) || targetQuaternion.length !== 4 ||
      !targetQuaternion.every(Number.isFinite)) {
    throw new TypeError('targetPose.quaternion 必须是 [x,y,z,w] 四元数');
  }

  const targetQ = normalizeQuat(targetQuaternion);
  let q = (options.initialJoints ?? model.joints.map(() => 0))
    .map((angle, i) => clampJoint(model, i, angle));

  if (q.length !== n || !q.every(Number.isFinite)) {
    throw new TypeError(`initialJoints 必须是 ${n} 个有限弧度值`);
  }

  const maxIterations = options.maxIterations ?? 120;
  const positionTolerance = options.positionTolerance ?? 1e-3;
  const orientationTolerance = options.orientationTolerance ?? 1e-3;
  const delta = options.delta ?? 1e-4;
  const damping = options.damping ?? 1e-2;
  const stepScale = options.stepScale ?? 0.4;
  const orientationWeight = options.orientationWeight ?? 1;
  const minStepScale = options.minStepScale ?? 1 / 64;

  function poseError(joints) {
    const tcp = forwardKinematics(model, joints).tcp;
    const position = targetPosition.map((v, i) => v - tcp.p[i]);
    const orientation = orientationError(targetQ, tcp.quat);
    return { position, orientation };
  }

  let iterations = 0;
  for (let iteration = 0; iteration < maxIterations; iteration++) {
    iterations = iteration + 1;
    const e = poseError(q);
    const positionNorm = Math.hypot(...e.position);
    const orientationNorm = Math.hypot(...e.orientation);

    if (positionNorm <= positionTolerance &&
        orientationNorm <= orientationTolerance) {
      return {
        ok: true,
        joints: q,
        iterations: iteration,
        error: { position: positionNorm, orientation: orientationNorm },
        singularity: false,
      };
    }

    // J 是 6×N：前三行位置变化，后三行姿态变化
    const J = Array.from({ length: 6 }, () => Array(n).fill(0));
    for (let j = 0; j < n; j++) {
      const qNext = [...q];
      qNext[j] += delta;
      const next = forwardKinematics(model, qNext).tcp;
      const base = forwardKinematics(model, q).tcp;
      const dp = next.p.map((v, axis) => (v - base.p[axis]) / delta);
      const dr = orientationError(next.quat, base.quat)
        .map((v) => v / delta);

      for (let axis = 0; axis < 3; axis++) {
        J[axis][j] = dp[axis];
        J[axis + 3][j] = dr[axis] * orientationWeight;
      }
    }

    const error = [...e.position, ...e.orientation.map((v) => v * orientationWeight)];
    const A = Array.from({ length: 6 }, (_, r) =>
      Array.from({ length: 6 }, (_, c) => {
        let sum = 0;
        for (let j = 0; j < n; j++) sum += J[r][j] * J[c][j];
        return sum + (r === c ? damping * damping : 0);
      })
    );

    const y = solveLinear(A, error);
    const dq = Array.from({ length: n }, (_, j) =>
      J.reduce((sum, row, axis) => sum + row[j] * y[axis], 0)
    );

    // Backtracking prevents a full DLS step from increasing pose error near
    // joint limits or in poorly conditioned configurations.
    const currentCost = positionNorm ** 2 +
      (orientationWeight * orientationNorm) ** 2;
    let accepted = false;
    for (let scale = stepScale; scale >= minStepScale; scale *= 0.5) {
      const candidate = q.map((angle, j) =>
        clampJoint(model, j, angle + scale * dq[j])
      );
      const candidateError = poseError(candidate);
      const candidatePositionNorm = Math.hypot(...candidateError.position);
      const candidateOrientationNorm = Math.hypot(...candidateError.orientation);
      const candidateCost = candidatePositionNorm ** 2 +
        (orientationWeight * candidateOrientationNorm) ** 2;

      if (candidateCost < currentCost) {
        q = candidate;
        accepted = true;
        break;
      }
    }

    // No useful descent direction remains; return the best solution found
    // instead of repeating numerically identical iterations.
    if (!accepted) break;
  }

  const finalError = poseError(q);
  const positionNorm = Math.hypot(...finalError.position);
  const orientationNorm = Math.hypot(...finalError.orientation);

  return {
    ok: positionNorm <= positionTolerance &&
        orientationNorm <= orientationTolerance,
    joints: q,
    iterations,
    error: { position: positionNorm, orientation: orientationNorm },
    singularity: false,
    message: '未收敛：目标可能不可达、接近奇异位形或迭代次数不足',
  };
}

export function createMotionApi(model) {
  return {
    solveIK: async (targetPose, options = {}) =>
      solveIK(model, targetPose, options),
    planMOVJ: async (startJoints, endJoints, options = {}) =>
        planMOVJ(model, startJoints, endJoints, options),
    planMOVL: async (startJoints, targetPose, options = {}) =>
        planMOVL(model, startJoints, targetPose, (pose, ikOptions) =>
    solveIK(model, pose, ikOptions),
    options
  ),
    emergencyStop: () => {},
  };
}
