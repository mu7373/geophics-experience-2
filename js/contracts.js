/**
 * 实验二共享接口。
 * 三人先按本文件对接，具体实现可分别放在 ik.js、calibration.js、dsl.js。
 */

/** @typedef {{position:number[], quaternion:number[]}} Pose */

/**
 * 运动服务：成员 1 实现。
 * solveIK(targetPose, options) 返回 { ok, joints, iterations, error, singularity }。
 * planMOVJ / planMOVL 返回按时间采样的关节角序列。
 */
export const motionApi = {
  solveIK: async () => ({ ok: false, message: 'IK 模块尚未接入' }),
  planMOVJ: async () => [],
  planMOVL: async () => [],
  emergencyStop: () => {},
};

/**
 * 标定服务：成员 2 实现。
 * toolPoints 是四组法兰位姿，userPoints 依次为 O、X+、Y 侧点。
 */
export const calibrationApi = {
  solveToolFrame: (toolPoints) => ({ ok: false, toolFrame: null, residual: null, toolPoints }),
  solveUserFrame: (userPoints) => ({ ok: false, userFrame: null, userPoints }),
};

/**
 * 指令服务：成员 3 实现。
 * parseProgram(text) 返回 { ok, instructions, errors }；执行器通过 motionApi 调度运动。
 */
export const programApi = {
  parseProgram: () => ({ ok: false, instructions: [], errors: ['DSL 模块尚未接入'] }),
  createExecutor: () => ({ state: 'IDLE', run: async () => {}, stop: () => {} }),
};

