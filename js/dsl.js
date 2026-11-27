/**
 * 机械臂虚拟指令解析器。
 *
 * 语法（大小写不敏感）：
 *   MOVJ POINT [, SPEED=50%]
 *   MOVL POINT [, SPEED=50%]
 *   GRIP OPEN|CLOSE
 *   WAIT seconds
 *   LOOP count ... ENDLOOP
 *   SET SPEED 50%
 */

export const DEFAULT_POINT_NAMES = Object.freeze([
  'HOME',
  'P_PRE',
  'P_GRASP',
  'P_PLACE_PRE',
  'P_PLACE',
]);

const NUMBER_PATTERN = '[+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)';
const MOVE_PATTERN = new RegExp(
  `^(MOVJ|MOVL)\\s+([A-Z_][A-Z0-9_-]*)(?:\\s*,?\\s+SPEED\\s*=\\s*(${NUMBER_PATTERN})\\s*%)?\\s*$`,
  'i',
);
const SET_SPEED_PATTERN = new RegExp(`^SET\\s+SPEED\\s+(${NUMBER_PATTERN})\\s*%\\s*$`, 'i');
const WAIT_PATTERN = new RegExp(`^WAIT\\s+(${NUMBER_PATTERN})\\s*$`, 'i');
const LOOP_PATTERN = new RegExp(`^LOOP\\s+(${NUMBER_PATTERN})\\s*$`, 'i');

function lineError(line, message) {
  return `第 ${line} 行：${message}`;
}

function stripComment(line) {
  const hash = line.indexOf('#');
  const slash = line.indexOf('//');
  const cutAt = [hash, slash]
    .filter((index) => index >= 0)
    .reduce((first, index) => Math.min(first, index), line.length);
  return line.slice(0, cutAt).trim();
}

function normalizeKnownPoints(value) {
  if (value == null) return new Set(DEFAULT_POINT_NAMES);
  if (value instanceof Map) return new Set([...value.keys()].map((name) => String(name).toUpperCase()));
  if (value instanceof Set || Array.isArray(value)) {
    return new Set([...value].map((name) => String(name).toUpperCase()));
  }
  if (typeof value === 'object') {
    return new Set(Object.keys(value).map((name) => name.toUpperCase()));
  }
  throw new TypeError('knownPoints 必须是点位名数组、Set、Map 或点位对象');
}

function validSpeed(speed) {
  return Number.isFinite(speed) && speed > 0 && speed <= 100;
}

function expandedInstructionCount(instructions) {
  return instructions.reduce((total, instruction) => {
    if (instruction.type !== 'LOOP') return total + 1;
    return total + instruction.count * expandedInstructionCount(instruction.body);
  }, 0);
}

/**
 * 把 DSL 文本解析为带行号的指令树。LOOP 保留为结构化节点，
 * 由执行器在每次 run 时展开，便于急停时立即清空队列。
 *
 * @param {string} text
 * @param {{knownPoints?: object|Map|Set|string[], maxLoopCount?: number}} options
 * @returns {{ok:boolean, instructions:Array, errors:string[], instructionCount:number}}
 */
export function parseProgram(text, options = {}) {
  if (typeof text !== 'string') {
    return {
      ok: false,
      instructions: [],
      errors: ['第 1 行：程序必须是文本'],
      instructionCount: 0,
    };
  }

  let knownPoints;
  try {
    knownPoints = normalizeKnownPoints(options.knownPoints);
  } catch (error) {
    return { ok: false, instructions: [], errors: [error.message], instructionCount: 0 };
  }

  const maxLoopCount = options.maxLoopCount ?? 1000;
  const instructions = [];
  const errors = [];
  const stack = [{ body: instructions, line: 0 }];
  const lines = text.replace(/\r\n?/g, '\n').split('\n');

  for (let index = 0; index < lines.length; index += 1) {
    const line = index + 1;
    const source = stripComment(lines[index]);
    if (!source) continue;

    const currentBody = () => stack[stack.length - 1].body;
    let match;

    if ((match = source.match(MOVE_PATTERN))) {
      const type = match[1].toUpperCase();
      const point = match[2].toUpperCase();
      const speed = match[3] == null ? null : Number(match[3]);

      if (!knownPoints.has(point)) {
        errors.push(lineError(line, `未知点位 ${point}`));
      }
      if (speed != null && !validSpeed(speed)) {
        errors.push(lineError(line, `速度必须在 0% 到 100% 之间，当前为 ${match[3]}%`));
      }
      currentBody().push({ type, point, speed, line, source });
      continue;
    }

    if ((match = source.match(/^GRIP\s+(OPEN|CLOSE)\s*$/i))) {
      currentBody().push({
        type: 'GRIP',
        action: match[1].toUpperCase(),
        line,
        source,
      });
      continue;
    }

    if ((match = source.match(WAIT_PATTERN))) {
      const seconds = Number(match[1]);
      if (!Number.isFinite(seconds) || seconds < 0) {
        errors.push(lineError(line, `WAIT 时间必须是非负数，当前为 ${match[1]}`));
      }
      currentBody().push({ type: 'WAIT', seconds, line, source });
      continue;
    }

    if ((match = source.match(SET_SPEED_PATTERN))) {
      const speed = Number(match[1]);
      if (!validSpeed(speed)) {
        errors.push(lineError(line, `速度必须在 0% 到 100% 之间，当前为 ${match[1]}%`));
      }
      currentBody().push({ type: 'SET_SPEED', speed, line, source });
      continue;
    }

    if ((match = source.match(LOOP_PATTERN))) {
      const count = Number(match[1]);
      if (!Number.isInteger(count) || count < 1 || count > maxLoopCount) {
        errors.push(lineError(
          line,
          `LOOP 次数必须是 1 到 ${maxLoopCount} 之间的整数，当前为 ${match[1]}`,
        ));
      }
      const loop = { type: 'LOOP', count, body: [], line, source };
      currentBody().push(loop);
      stack.push({ body: loop.body, line });
      continue;
    }

    if (/^ENDLOOP\s*$/i.test(source)) {
      if (stack.length === 1) {
        errors.push(lineError(line, 'ENDLOOP 没有对应的 LOOP'));
      } else {
        stack.pop();
      }
      continue;
    }

    if (/^(MOVJ|MOVL)\b/i.test(source)) {
      errors.push(lineError(line, 'MOVJ/MOVL 格式应为“MOVJ 点位, SPEED=50%”'));
    } else if (/^GRIP\b/i.test(source)) {
      errors.push(lineError(line, 'GRIP 只支持 OPEN 或 CLOSE'));
    } else if (/^WAIT\b/i.test(source)) {
      errors.push(lineError(line, 'WAIT 后应跟以秒为单位的数字'));
    } else if (/^SET\b/i.test(source)) {
      errors.push(lineError(line, 'SET SPEED 格式应为“SET SPEED 50%”'));
    } else if (/^LOOP\b/i.test(source)) {
      errors.push(lineError(line, 'LOOP 后应跟正整数次数'));
    } else {
      errors.push(lineError(line, `不支持的指令“${source}”`));
    }
  }

  for (let index = stack.length - 1; index >= 1; index -= 1) {
    errors.push(lineError(stack[index].line, 'LOOP 缺少 ENDLOOP'));
  }
  if (instructions.length === 0 && errors.length === 0) {
    errors.push(lineError(1, '程序不能为空'));
  }

  return {
    ok: errors.length === 0,
    instructions,
    errors,
    instructionCount: expandedInstructionCount(instructions),
  };
}

export default parseProgram;
