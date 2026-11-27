import test from 'node:test';
import assert from 'node:assert/strict';
import { parseProgram } from '../js/dsl.js';

test('parseProgram 解析六类指令并保留 LOOP 结构', () => {
  const result = parseProgram(`
SET SPEED 50%
MOVJ HOME
MOVL P_GRASP, SPEED=30%
GRIP CLOSE
WAIT 0.5
LOOP 2
  MOVL P_PLACE, SPEED=20%
  GRIP OPEN
ENDLOOP
`);

  assert.equal(result.ok, true);
  assert.equal(result.errors.length, 0);
  assert.equal(result.instructions.at(-1).type, 'LOOP');
  assert.equal(result.instructions.at(-1).body.length, 2);
  assert.equal(result.instructionCount, 9);
});

test('parseProgram 报告未知点位、非法速度和未闭合循环的行号', () => {
  const result = parseProgram(`MOVJ NOT_FOUND
SET SPEED 0%
LOOP 2
  WAIT 1`);

  assert.equal(result.ok, false);
  assert.ok(result.errors.some((error) => error.includes('第 1 行') && error.includes('未知点位')));
  assert.ok(result.errors.some((error) => error.includes('第 2 行') && error.includes('速度')));
  assert.ok(result.errors.some((error) => error.includes('第 3 行') && error.includes('ENDLOOP')));
});

test('parseProgram 检查多余 ENDLOOP 和 WAIT 参数', () => {
  const result = parseProgram(`WAIT -1
ENDLOOP`);

  assert.equal(result.ok, false);
  assert.match(result.errors[0], /第 1 行/);
  assert.match(result.errors[1], /第 2 行/);
});
