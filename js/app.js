import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RobotController } from './robotController.js';
import { calibrationApi, motionApi, programApi } from './contracts.js';

const stateEl = document.querySelector('#run-state');
const calibrationMessage = document.querySelector('#calibration-message');
const programInput = document.querySelector('#program-input');
const programMessage = document.querySelector('#program-message');
const checkButton = document.querySelector('#check-program');
const runButton = document.querySelector('#run-program');
const stopButton = document.querySelector('#emergency-stop');

let parsedProgram = null;
const toolPoses = [];
const userPoints = [];

async function loadScene() {
  const mount = document.querySelector('#scene-root');
  const scene = new THREE.Scene();
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
  const spec = await fetch('./images/models/joint_map.json').then((response) => response.json());
  const { robot } = await controller.loadRobot('./images/models/robot_articulated.glb', spec);
  scene.add(robot);
  stateEl.textContent = '模型已加载';
  calibrationMessage.textContent = '可记录工具系四点与用户系三点；计算模块待接入。';

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
  const result = programApi.parseProgram(programInput.value);
  parsedProgram = result.ok ? result : null;
  runButton.disabled = !result.ok;
  programMessage.textContent = result.ok
    ? `脚本通过：共 ${result.instructions.length} 条指令。`
    : `脚本有误：\n${result.errors.join('\n')}`;
});

runButton.addEventListener('click', async () => {
  if (!parsedProgram) return;
  stateEl.textContent = '运行中';
  const executor = programApi.createExecutor({ motionApi });
  await executor.run(parsedProgram.instructions);
  stateEl.textContent = '完成';
});

stopButton.addEventListener('click', () => {
  motionApi.emergencyStop();
  stateEl.textContent = '急停';
  programMessage.textContent = '已停止当前任务；复位后才可继续执行。';
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
});

