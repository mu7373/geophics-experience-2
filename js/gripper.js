export const GRIPPER_STATES = Object.freeze({
  OPEN: 'OPEN',
  CLOSED: 'CLOSED',
});

/**
 * 夹爪控制器。合拢时把夹取范围内最近的物体 attach 到 TCP，张开时再
 * attach 回场景根节点。Three.js Object3D.attach 会保留物体的世界位姿。
 */
export class GripperController {
  constructor(options = {}) {
    if (!options.tcp || !options.scene) {
      throw new TypeError('GripperController 需要 tcp 和 scene');
    }
    this.tcp = options.tcp;
    this.scene = options.scene;
    this.candidates = [...(options.candidates ?? [])];
    this.graspDistance = options.graspDistance ?? 1;
    this.setJawOpening = options.setJawOpening ?? (() => {});
    this.onStateChange = options.onStateChange ?? (() => {});
    this.state = GRIPPER_STATES.OPEN;
    this.attachedObject = null;
  }

  addCandidate(object) {
    if (object && !this.candidates.includes(object)) this.candidates.push(object);
    return object;
  }

  removeCandidate(object) {
    this.candidates = this.candidates.filter((candidate) => candidate !== object);
  }

  _worldPosition(object) {
    if (!object?.position?.clone || typeof object.getWorldPosition !== 'function') return null;
    const position = object.position.clone();
    object.getWorldPosition(position);
    return position;
  }

  _nearestCandidate() {
    const tcpPosition = this._worldPosition(this.tcp);
    if (!tcpPosition) return null;
    let nearest = null;
    let nearestDistance = Infinity;

    for (const object of this.candidates) {
      if (!object || object === this.attachedObject || object.visible === false ||
          object.userData?.pickable === false) continue;
      const position = this._worldPosition(object);
      if (!position) continue;
      const distance = typeof position.distanceTo === 'function'
        ? position.distanceTo(tcpPosition)
        : Math.hypot(position.x - tcpPosition.x, position.y - tcpPosition.y, position.z - tcpPosition.z);
      if (distance < nearestDistance) {
        nearest = object;
        nearestDistance = distance;
      }
    }
    return nearestDistance <= this.graspDistance
      ? { object: nearest, distance: nearestDistance }
      : null;
  }

  _setState(state) {
    this.state = state;
    this.setJawOpening(state === GRIPPER_STATES.OPEN ? 1 : 0);
    this.onStateChange(state, this);
  }

  async close() {
    this._setState(GRIPPER_STATES.CLOSED);
    if (this.attachedObject) {
      return { ok: true, state: this.state, attachedObject: this.attachedObject };
    }

    const nearest = this._nearestCandidate();
    if (!nearest) {
      return {
        ok: true,
        state: this.state,
        attachedObject: null,
        message: '夹爪已合拢，但范围内没有可夹取物体',
      };
    }

    this.tcp.attach(nearest.object);
    this.attachedObject = nearest.object;
    this.attachedObject.userData.gripped = true;
    return {
      ok: true,
      state: this.state,
      attachedObject: this.attachedObject,
      distance: nearest.distance,
    };
  }

  async open() {
    this._setState(GRIPPER_STATES.OPEN);
    const releasedObject = this.attachedObject;
    if (releasedObject) {
      this.scene.attach(releasedObject);
      releasedObject.userData.gripped = false;
      this.attachedObject = null;
    }
    return { ok: true, state: this.state, releasedObject };
  }
}

/**
 * 在 TCP 下创建简单双指夹爪。传入 THREE 命名空间，使核心夹爪逻辑不
 * 依赖渲染库，也方便独立测试。
 */
export function createGripperVisual(THREE, tcp, options = {}) {
  if (!THREE || !tcp) throw new TypeError('createGripperVisual 需要 THREE 和 tcp');
  const color = options.color ?? 0x55c7ff;
  const openWidth = options.openWidth ?? 0.7;
  const closedWidth = options.closedWidth ?? 0.28;
  const fingerLength = options.fingerLength ?? 0.8;
  const group = new THREE.Group();
  group.name = 'virtual-gripper';

  const material = new THREE.MeshStandardMaterial({ color, metalness: 0.35, roughness: 0.35 });
  const palm = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.18, 0.35), material);
  palm.position.z = 0.08;
  const fingerGeometry = new THREE.BoxGeometry(0.13, 0.18, fingerLength);
  const left = new THREE.Mesh(fingerGeometry, material);
  const right = new THREE.Mesh(fingerGeometry, material);
  left.name = 'gripper-left-finger';
  right.name = 'gripper-right-finger';
  left.position.z = right.position.z = fingerLength / 2;
  group.add(palm, left, right);
  tcp.add(group);

  const setOpening = (openness) => {
    const width = closedWidth + (openWidth - closedWidth) * Math.max(0, Math.min(1, openness));
    left.position.x = -width / 2;
    right.position.x = width / 2;
  };
  setOpening(1);

  return { group, left, right, setOpening };
}

export default GripperController;
