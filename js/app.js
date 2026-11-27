import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RobotController } from './robotController.js';
import {
  calibrationApi,
  motionApi as defaultMotionApi,
  programApi,
} from './contracts.js';
import { createMotionApi } from './ik.js';
import { createModel, forwardKinematics } from './fk.js';
import { PROGRAM_STATES } from './executor.js';
import { createGripperVisual, GripperController } from './gripper.js';

const stateEl = document.querySelector('#run-state');
const calibrationMessage = document.querySelector('#calibration-message');
const programInput = document.querySelector('#program-input');
const programMessage = document.querySelector('#program-message');
const checkButton = document.querySelector('#check-program');
const runButton = document.querySelector('#run-program');
const pauseButton = document.querySelector('#pause-program');
const resumeButton = document.querySelector('#resume-program');
const stopButton = document.querySelector('#emergency-stop');
const resetButton = document.querySelector('#reset-program');

const STATE_LABELS = {
  [PROGRAM_STATES.IDLE]: '就绪',
  [PROGRAM_STATES.RUNNING]: '运行中',
  [PROGRAM_STATES.PAUSED]: '已暂停',
  [PROGRAM_STATES.DONE]: '已完成',
  [PROGRAM_STATES.E_STOP]: '急停',
};

let parsedProgram = null;
let executor = null;
let gripper = null;
let workpiece = null;
let demoPoints = null;
let sceneRef = null;
const toolPoses = [];
const userPoints = [];

function createDemoPoints(model) {
  const jointPoints = {
    HOME: [0, 0, 0, 0, 0, 0],
    P_PRE: [0.32, -0.24, 0.28, -0.12, 0.08, 0],
    P_GRASP: [0.32, -0.34, 0.43, -0.18, 0.08, 0],
    P_PLACE_PRE: [-0.42, -0.22, 0.3, -0.1, -0.08, 0],
    P_PLACE: [-0.42, -0.34, 0.45, -0.17, -0.08, 0],
  };

  return Object.fromEntries(Object.entries(jointPoints).map(([name, joints]) => {
    const tcp = forwardKinematics(model, joints).tcp;
    return [name, {
      joints: [...joints],
      pose: { position: [...tcp.p], quaternion: [...tcp.quat] },
    }];
  }));
}

function updateControls(state = executor?.state ?? PROGRAM_STATES.IDLE) {
  const ready = Boolean(executor && parsedProgram);
  const busy = state === PROGRAM_STATES.RUNNING || state === PROGRAM_STATES.PAUSED;
  const stopped = state === PROGRAM_STATES.E_STOP;
  runButton.disabled = !ready || busy || stopped;
  pauseButton.disabled = state !== PROGRAM_STATES.RUNNING;
  resumeButton.disabled = state !== PROGRAM_STATES.PAUSED;
  resetButton.disabled = !stopped;
  stateEl.textContent = STATE_LABELS[state] ?? state;
}

function createDemoObjects(scene, controller, points) {
  const visual = createGripperVisual(THREE, controller.tcp, {
    openWidth: 0.85,
    closedWidth: 0.35,
  });

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(0.65, 0.65, 0.65),
    new THREE.MeshStandardMaterial({ color: 0xffa53d, roughness: 0.45 }),
  );
  box.name = 'demo-workpiece';
  box.position.fromArray(points.P_GRASP.pose.position);
  box.userData.pickable = true;
  scene.add(box);

  const marker = new THREE.Mesh(
    new THREE.TorusGeometry(0.65, 0.07, 12, 48),
    new THREE.MeshStandardMaterial({ color: 0x65e397, emissive: 0x153d2a }),
  );
  marker.name = 'place-target';
  marker.position.fromArray(points.P_PLACE.pose.position);
  marker.rotation.x = Math.PI / 2;
  scene.add(marker);

  const controllerApi = new GripperController({
    tcp: controller.tcp,
    scene,
    candidates: [box],
    graspDistance: 1.1,
    setJawOpening: visual.setOpening,
    onStateChange: (state, instance) => {
      const detail = instance.attachedObject ? `：${instance.attachedObject.name}` : '';
      programMessage.textContent = `夹爪 ${state}${detail}`;
    },
  });

  return { gripper: controllerApi, workpiece: box };
}

function resetWorkpiece() {
  if (!workpiece || !demoPoints || !sceneRef) return;
  if (gripper?.attachedObject === workpiece) {
    sceneRef.attach(workpiece);
    gripper.attachedObject = null;
  }
  workpiece.userData.gripped = false;
  workpiece.position.fromArray(demoPoints.P_GRASP.pose.position);
  workpiece.quaternion.identity();
  workpiece.scale.set(1, 1, 1);
}

async function loadScene() {
  const mount = document.querySelector('#scene-root');
  const scene = new THREE.Scene();
  sceneRef = scene;
  scene.background = new THREE.Color(0x101722);
  const camera = new THREE.PerspectiveCamera(45, mount.clientWidth / mount.clientHeight, 0.1, 1000);
  camera.position.set(16, 12, 16);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(mount.clientWidth, mount.clientHeight);
  mount.replaceChildren(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xdbeeff, 0x263142, 2));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.5);
  keyLight.position.set(8, 12, 6);
  scene.add(keyLight, new THREE.GridHelper(32, 32, 0x385271, 0x25364c));

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 4, 0);
  controls.update();

  const controller = new RobotController();
  const [spec, modelJson] = await Promise.all([
    fetch('./images/models/joint_map.json').then((response) => response.json()),
    fetch('./images/models/robot-model.json').then((response) => response.json()),
  ]);
  const { robot } = await controller.loadRobot('./images/models/robot_articulated.glb', spec);
  scene.add(robot);

  const model = createModel(modelJson);
  Object.assign(defaultMotionApi, createMotionApi(model));
  demoPoints = createDemoPoints(model);
  ({ gripper, workpiece } = createDemoObjects(scene, controller, demoPoints));

  executor = programApi.createExecutor({
    motionApi: defaultMotionApi,
    controller,
    gripper,
    points: demoPoints,
    baseJointSpeed: Math.PI / 2,
    baseLinearSpeed: 4,
    onStateChange: (state) => updateControls(state),
    onInstruction: (instruction) => {
      const loop = instruction.loopIteration
        ? `（循环 ${instruction.loopIteration}/${instruction.loopCount}）`
        : '';
      programMessage.textContent = `第 ${instruction.line} 行${loop}：${instruction.source}`;
    },
  });

  updateControls(PROGRAM_STATES.IDLE);
  calibrationMessage.textContent = '可记录工具系四点与用户系三点。';

  const resize = () => {
    const { clientWidth: width, clientHeight: height } = mount;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  };
  window.addEventListener('resize', resize);

  const render = () => {
    requestAnimationFrame(render);
    controls.update();
    renderer.render(scene, camera);
  };
  render();

  document.querySelector('#record-tool-point').addEventListener('click', () => {
    toolPoses.push(controller.getTCPMatrix4());
    calibrationMessage.textContent = `已记录工具点 ${toolPoses.length}/4。`;
    document.querySelector('#solve-tool-frame').disabled = toolPoses.length < 4;
  });
  document.querySelector('#record-user-point').addEventListener('click', () => {
    const pose = controller.getTCPPose();
    userPoints.push([pose.p.x, pose.p.y, pose.p.z]);
    calibrationMessage.textContent = `已记录用户点 ${userPoints.length}/3。`;
    document.querySelector('#solve-user-frame').disabled = userPoints.length < 3;
  });
}

checkButton.addEventListener('click', () => {
  const result = programApi.parseProgram(programInput.value, {
    knownPoints: demoPoints ?? undefined,
  });
  parsedProgram = result.ok ? result : null;
  updateControls();
  programMessage.textContent = result.ok
    ? `脚本通过：展开循环后共 ${result.instructionCount} 条指令。`
    : `脚本有误：\n${result.errors.join('\n')}`;
});

runButton.addEventListener('click', async () => {
  if (!parsedProgram || !executor) return;
  resetWorkpiece();
  await gripper.open();
  const result = await executor.run(parsedProgram.instructions);
  programMessage.textContent = result.ok
    ? '抓取—搬运—放置程序执行完成。'
    : result.message;
  updateControls(result.state);
});

pauseButton.addEventListener('click', () => executor?.pause());
resumeButton.addEventListener('click', () => executor?.resume());

stopButton.addEventListener('click', () => {
  if (executor) executor.stop();
  else defaultMotionApi.emergencyStop();
  programMessage.textContent = '已立即停止当前任务并清空指令队列；复位后才可重新运行。';
});

resetButton.addEventListener('click', async () => {
  if (!executor) return;
  executor.reset();
  await gripper.open();
  resetWorkpiece();
  programMessage.textContent = '急停已复位，可重新运行程序。';
  updateControls();
});

document.querySelector('#solve-tool-frame').addEventListener('click', () => {
  calibrationMessage.textContent = '等待成员 2 接入四点法计算模块。';
  calibrationApi.solveToolFrame(toolPoses);
});

document.querySelector('#solve-user-frame').addEventListener('click', () => {
  calibrationMessage.textContent = '等待成员 2 接入三点法计算模块。';
  calibrationApi.solveUserFrame(userPoints);
});

loadScene().catch((error) => {
  stateEl.textContent = '加载失败';
  document.querySelector('#scene-root').innerHTML = `<p>模型加载失败：${error.message}</p>`;
  console.error(error);
});
