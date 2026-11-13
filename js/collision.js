import { forwardKinematics, mat4Position } from './fk.js';

/** 两个 AABB 是否相交 */
function intersectsAABB(a, b) {
  return (
    a.min[0] <= b.max[0] && a.max[0] >= b.min[0] &&
    a.min[1] <= b.max[1] && a.max[1] >= b.min[1] &&
    a.min[2] <= b.max[2] && a.max[2] >= b.min[2]
  );
}

/** 用线段两端构造带厚度的 AABB */
function segmentAABB(start, end, radius) {
  return {
    min: start.map((v, i) => Math.min(v, end[i]) - radius),
    max: start.map((v, i) => Math.max(v, end[i]) + radius),
  };
}

/**
 * 检查一个关节姿态是否与障碍物 AABB 碰撞。
 * obstacles: [{ id, min:[x,y,z], max:[x,y,z] }]
 */
export function checkCollision(model, joints, obstacles, options = {}) {
  const radius = options.linkRadius ?? 0.25;

  if (!Array.isArray(obstacles)) {
    return { ok: false, collision: false, message: 'obstacles 必须是数组' };
  }

  for (const obstacle of obstacles) {
    if (!obstacle?.min || !obstacle?.max ||
        obstacle.min.length !== 3 || obstacle.max.length !== 3 ||
        ![...obstacle.min, ...obstacle.max].every(Number.isFinite) ||
        obstacle.min.some((v, i) => v > obstacle.max[i])) {
      return { ok: false, collision: false, message: '障碍物 AABB 格式无效' };
    }
  }

  const fk = forwardKinematics(model, joints);
  const jointNames = model.joints.map((joint) => joint.node);

  // 获取关节中心的世界坐标
  const points = jointNames
    .map((name) => fk.world.get(name))
    .filter(Boolean)
    .map(mat4Position);

  // 连杆段 + 一个粗略半径；第一个段从基座原点开始
  const segments = [];
  const base = points[0];
  if (base) segments.push({ name: 'base-link', box: segmentAABB([0, 0, 0], base, radius) });

  for (let i = 0; i < points.length - 1; i++) {
    segments.push({
      name: `link-${i}`,
      box: segmentAABB(points[i], points[i + 1], radius),
    });
  }

  for (const segment of segments) {
    for (const obstacle of obstacles) {
      if (intersectsAABB(segment.box, obstacle)) {
        return {
          ok: true,
          collision: true,
          link: segment.name,
          obstacle: obstacle.id ?? 'unnamed',
          message: `连杆 ${segment.name} 与障碍物 ${obstacle.id ?? 'unnamed'} 相交`,
        };
      }
    }
  }

  return { ok: true, collision: false, message: '未检测到碰撞' };
}

/** 检查轨迹中每个关节采样点；首次碰撞时返回采样索引 */
export function checkTrajectoryCollision(model, trajectory, obstacles, options = {}) {
  for (let i = 0; i < trajectory.length; i++) {
    const result = checkCollision(model, trajectory[i].joints, obstacles, options);
    if (!result.ok || result.collision) {
      return { ...result, sampleIndex: i, t: trajectory[i].t };
    }
  }
  return { ok: true, collision: false, message: '整条轨迹未检测到碰撞' };
}