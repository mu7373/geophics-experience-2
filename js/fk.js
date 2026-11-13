/**(from shiyan yi)
 * kinematics.js —— 纯数学运动学内核，不依赖 three.js、不依赖 DOM。
 *
 * 为什么单独一份：要求 c（轨迹分析）和要求 d（可达区域）都只需要"给一组关节角，
 * 算出末端 TCP 位姿"，跟示教器界面无关。把它做成不依赖渲染引擎的纯函数，
 * 浏览器里能用、node 里也能跑，才能做「三档验证」——自写 FK 与场景树 matrixWorld 对拍。
 *
 * 事实源：assets/robot-model.json（由 GLB 实测导出）。
 * 关节轴由 assets/robot-model.json 的 joints[].axis 提供（实测：全部绕局部 +Y）。
 * 判定方法与走过的弯路见 contract.js 顶部注释。
 */

export const JOINT_NAMES = ['J0', 'J1', 'J2', 'J3', 'J4', 'J5'];

/* ───────────────── 四元数 / 矩阵基本运算 ───────────────── */

const RAD = Math.PI / 180;
export const degToRad = (d) => d * RAD;
export const radToDeg = (r) => r / RAD;

/** 单位轴 + 角度 → 四元数 [x,y,z,w]（右手定则） */
export function quatFromAxisAngle(axis, angleRad) {
  const n = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const s = Math.sin(angleRad / 2);
  return [(axis[0] / n) * s, (axis[1] / n) * s, (axis[2] / n) * s, Math.cos(angleRad / 2)];
}

/** a ⊗ b（先 b 后 a） */
export function quatMul(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz
  ];
}

/** 四元数归一化，防止长链连乘后漂移 */
export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** 四元数 → 行主序 4x4 矩阵（含平移 t） */
export function quatToMat4(q, t = [0, 0, 0]) {
  const [x, y, z, w] = quatNormalize(q);
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return [
    1 - (yy + zz), xy - wz, xz + wy, t[0],
    xy + wz, 1 - (xx + zz), yz - wx, t[1],
    xz - wy, yz + wx, 1 - (xx + yy), t[2],
    0, 0, 0, 1
  ];
}

/** 行主序 4x4 相乘 a·b */
export function mat4Mul(a, b) {
  const o = new Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      o[r * 4 + c] =
        a[r * 4 + 0] * b[0 * 4 + c] +
        a[r * 4 + 1] * b[1 * 4 + c] +
        a[r * 4 + 2] * b[2 * 4 + c] +
        a[r * 4 + 3] * b[3 * 4 + c];
    }
  }
  return o;
}

/** 取矩阵的平移分量 */
export function mat4Position(m) {
  return [m[3], m[7], m[11]];
}

/** 取矩阵的旋转部分（行主序 3x3），用于算 RPY */
export function mat4Rotation(m) {
  return [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
}

/** 旋转矩阵（行主序 3x3）→ ZYX 内旋欧拉角 [rx, ry, rz]（弧度），即工业界常用 RPY */
export function rotToRPY(r) {
  const sy = -r[6];                       // -R20
  const cy = Math.hypot(r[0], r[3]);
  const ry = Math.atan2(sy, cy);
  let rx, rz;
  if (cy > 1e-9) {
    rx = Math.atan2(r[7], r[8]);
    rz = Math.atan2(r[3], r[0]);
  } else {                                 // 万向锁：roll/pitch 不可分，把自由度全给 roll
    rx = Math.atan2(-r[5], r[4]);
    rz = 0;
  }
  return [rx, ry, rz];
}

/* ───────────────── 机器人模型 ───────────────── */

/**
 * 由 robot-model.json 建一个可计算的最小模型（纯数据，不含渲染对象）。
 * @param {object} modelJson assets/robot-model.json 的内容
 */
export function createModel(modelJson) {
  const nodes = modelJson.nodes.map((n) => ({
    name: n.name,
    parent: n.parent,
    t: n.translation ?? [0, 0, 0],
    q: n.rotation ?? [0, 0, 0, 1]
  }));
  const byName = new Map(nodes.map((n) => [n.name, n]));

  // 拓扑序：父节点必须排在子节点之前
  const order = [];
  const visit = (n) => {
    order.push(n);
    for (const m of nodes) if (m.parent === n.name) visit(m);
  };
  const roots = nodes.filter((n) => !n.parent || !byName.has(n.parent));
  roots.forEach(visit);

  const joints = modelJson.joints.map((j) => ({
    name: j.name,
    node: j.node ?? j.name,
    axis: j.axis,
    lower: degToRad(j.limitDeg[0]),
    upper: degToRad(j.limitDeg[1]),
    qZero: j.qZero
  }));
  const jointByNode = new Map(joints.map((j) => [j.node, j]));

  return {
    nodes,
    order,
    joints,
    jointByNode,
    tcpNode: modelJson.tcpNode ?? 'tcp',
    names: joints.map((j) => j.name),
    lower: joints.map((j) => j.lower),
    upper: joints.map((j) => j.upper)
  };
}

/** 把角度钳制进限位；返回钳制后的值 */
export function clampJoint(model, i, angleRad) {
  return Math.min(model.upper[i], Math.max(model.lower[i], angleRad));
}

/**
 * 正运动学。这就是「自写的 FK 函数」——与场景树 matrixWorld 对拍的对象。
 *
 * @param {object} model   createModel() 的结果
 * @param {number[]} qRad  6 个关节角（弧度），长度不足按 0 补
 * @returns {{world: Map<string, number[]>, tcp: {p:number[], rpy:number[], quat:number[], matrix:number[]}}}
 */
export function forwardKinematics(model, qRad) {
  const world = new Map();

  for (const node of model.order) {
    const parentWorld = node.parent && world.has(node.parent) ? world.get(node.parent) : null;

    // 关节节点：local = qZero ⊗ R_axis(θ)，平移取节点自身平移
    const joint = model.jointByNode.get(node.name);
    let localQ = node.q;
    if (joint) {
      const i = model.joints.indexOf(joint);
      const theta = clampJoint(model, i, Number(qRad?.[i]) || 0);
      localQ = quatMul(joint.qZero, quatFromAxisAngle(joint.axis, theta));
    }
    const local = quatToMat4(localQ, node.t);
    world.set(node.name, parentWorld ? mat4Mul(parentWorld, local) : local);
  }

  const tcpM = world.get(model.tcpNode) ?? quatToMat4([0, 0, 0, 1]);
  return {
    world,
    tcp: {
      matrix: tcpM,
      p: mat4Position(tcpM),
      rpy: rotToRPY(mat4Rotation(tcpM)),
      quat: (() => {
        // 从旋转矩阵反解四元数（Shepperd 方法，数值稳定）
        const m = mat4Rotation(tcpM);
        const tr = m[0] + m[4] + m[8];
        let q;
        if (tr > 0) {
          const s = Math.sqrt(tr + 1) * 2;
          q = [(m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s, 0.25 * s];
        } else if (m[0] > m[4] && m[0] > m[8]) {
          const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
          q = [0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s, (m[7] - m[5]) / s];
        } else if (m[4] > m[8]) {
          const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
          q = [(m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s, (m[2] - m[6]) / s];
        } else {
          const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
          q = [(m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s, (m[3] - m[1]) / s];
        }
        return quatNormalize(q);
      })()
    }
  };
}


