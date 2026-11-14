import assert from 'node:assert/strict';
import test from 'node:test';
import {UNKNOWN_POSITION, normalizeReferenceState, withReferenceState} from './w600_reference.mjs';

const DEVICE = '0x00124b0000000001';
const OTHER = '0x00124b0000000002';
const ALLOWLIST = [DEVICE];
const UNKNOWN = {position: null, calibrated: 'not_ready', running_state: 'idle'};
const marker = {[0x0360]: UNKNOWN_POSITION};
const state = {position: 24, calibrated: 'ready', running_state: 'heat', battery: 90};
const normalize = (raw, converted = state, ieee = DEVICE, devices = ALLOWLIST) =>
  normalizeReferenceState(ieee, raw, converted, devices);

test('only an explicitly allowed device recognizes the exact custom marker', () => {
  assert.deepEqual(normalize(marker), {...state, ...UNKNOWN});
  assert.deepEqual(normalizeReferenceState(DEVICE, marker, undefined, ALLOWLIST), UNKNOWN);
  assert.strictEqual(normalize(marker, state, OTHER), state);
  assert.strictEqual(normalize(marker, state, DEVICE, []), state);
  assert.strictEqual(normalizeReferenceState(DEVICE, marker, state), state);
  assert.deepEqual(normalize(marker, {}, DEVICE.toUpperCase()), UNKNOWN);
});

test('ordinary positions and other invalid values are not reinterpreted', () => {
  for (const position of [0, 17.5, 100, 6553.4, -1, NaN, Infinity, '6553.5', null]) {
    assert.strictEqual(normalize({[0x0360]: position}), state);
  }
  assert.strictEqual(normalize({}), state);
  assert.strictEqual(normalize(undefined), state);
});

test('real error and progress invalidate position without inventing not_ready', () => {
  for (const [value, calibrated] of [[2, 'error'], [3, 'in_progress']]) {
    const expected = {...state, position: null, calibrated, running_state: 'idle'};
    assert.deepEqual(normalize({[0x027b]: value}), expected);
    assert.deepEqual(normalize({...marker, [0x027b]: value}), expected);
    assert.strictEqual(normalize({[0x027b]: value}, state, OTHER), state);
  }
  assert.strictEqual(normalize({[0x027b]: 1}), state);
});

function fixture() {
  return {
    model: 'W600',
    fromZigbee: [{cluster: 'manuSpecificLumi', convert: () => ({calibrated: 'ready'})}],
    extend: [{fromZigbee: [
      {cluster: 'manuSpecificLumi', convert: () => ({position: UNKNOWN_POSITION})},
      {cluster: 'manuSpecificLumi', convert: async () => ({running_state: 'heat'})},
    ]}],
  };
}

function converters(definition) {
  return [...definition.fromZigbee, ...definition.extend.flatMap((part) => part.fromZigbee)];
}

test('root and modern-extend converters cannot overwrite corrections in either order', async () => {
  const original = fixture();
  const wrapped = withReferenceState(original, ALLOWLIST);
  for (const reverse of [false, true]) {
    const ordered = converters(wrapped);
    if (reverse) ordered.reverse();
    const merged = {};
    for (const converter of ordered) {
      Object.assign(merged, await converter.convert(null,
        {cluster: 'manuSpecificLumi', device: {ieeeAddr: DEVICE}, data: marker}, () => {}, {}, {}));
    }
    assert.deepEqual(merged, UNKNOWN);
  }
  assert.equal(original.extend[0].fromZigbee[0].convert().position, UNKNOWN_POSITION);
  assert.notStrictEqual(wrapped.fromZigbee, original.fromZigbee);
  assert.notStrictEqual(wrapped.extend[0], original.extend[0]);
});

test('published state is corrected and callback arguments/context are preserved', async () => {
  const published = [];
  const context = {};
  const msg = {cluster: 'manuSpecificLumi', data: marker};
  const options = {};
  const meta = {device: {ieeeAddr: DEVICE}};
  const original = {fromZigbee: [{convert(model, received, publish, settings, metadata) {
    assert.strictEqual(this, context);
    assert.equal(model, 'model');
    assert.strictEqual(received, msg);
    assert.strictEqual(settings, options);
    assert.strictEqual(metadata, meta);
    publish({position: UNKNOWN_POSITION, running_state: 'heat'}, 'extra');
    return undefined;
  }}]};
  const wrapped = withReferenceState(original, ALLOWLIST);
  const result = await wrapped.fromZigbee[0].convert.call(context, 'model', msg,
    (...args) => published.push(args), options, meta);
  assert.deepEqual(result, UNKNOWN);
  assert.deepEqual(published, [[UNKNOWN, 'extra']]);
});

test('other clusters/devices and an empty allowlist retain native behavior', async () => {
  for (const [cluster, ieee, allowlist] of [
    ['hvacThermostat', DEVICE, ALLOWLIST],
    ['manuSpecificLumi', OTHER, ALLOWLIST],
    ['manuSpecificLumi', DEVICE, []],
  ]) {
    const original = {fromZigbee: [{convert: () => state}]};
    const wrapped = withReferenceState(original, allowlist);
    const result = await wrapped.fromZigbee[0].convert(null,
      {cluster, device: {ieeeAddr: ieee}, data: marker}, () => {}, {}, {});
    assert.strictEqual(result, state);
  }
});

test('error correction reaches every converter and leaves source data untouched', async () => {
  const data = {...marker, [0x027b]: 2};
  const copy = {...data};
  const wrapped = withReferenceState(fixture(), ALLOWLIST);
  for (const converter of converters(wrapped)) {
    const result = await converter.convert(null,
      {cluster: 'manuSpecificLumi', device: {ieeeAddr: DEVICE}, data}, () => {}, {}, {});
    assert.deepEqual(result, {position: null, calibrated: 'error', running_state: 'idle'});
  }
  assert.deepEqual(data, copy);
});

function session(retained = state) {
  let current = {...retained};
  const native = {fromZigbee: [{convert(model, msg) {
    const result = {};
    if (msg.data[0x0360] !== undefined) {
      result.position = msg.data[0x0360];
      result.running_state = result.position > 0 ? 'heat' : 'idle';
    }
    if (msg.data[0x027b] !== undefined) {
      result.calibrated = ['not_ready', 'ready', 'error', 'in_progress'][msg.data[0x027b]];
    }
    return result;
  }}]};
  const wrapped = withReferenceState(native, ALLOWLIST);
  return async (data, type = 'attributeReport', cluster = 'manuSpecificLumi') => {
    const result = await wrapped.fromZigbee[0].convert(null,
      {cluster, type, device: {ieeeAddr: DEVICE}, data}, () => {}, {}, {state: current});
    current = {...current, ...result};
    return current;
  };
}

test('marker latch rejects independent cached Ready and position frames', async () => {
  const receive = session();
  await receive(marker);
  for (const data of [{[0x027b]: 1}, {[0x0360]: 24}, {[0x027b]: 1, [0x0360]: 24}]) {
    assert.deepEqual(await receive(data), {...state, ...UNKNOWN});
  }
});

test('a fresh progress report plus Ready and position reports clears in either order', async () => {
  for (const reverse of [false, true]) {
    const receive = session();
    await receive(marker);
    await receive({[0x027b]: 3});
    const completion = [{[0x027b]: 1}, {[0x0360]: 37.5}];
    if (reverse) completion.reverse();
    assert.equal((await receive(completion[0])).position, null);
    assert.deepEqual(await receive(completion[1]), {...state, position: 37.5, calibrated: 'ready'});
    assert.equal((await receive({[0x0360]: 0})).position, 0);
  }
});

test('cache reads cannot start or finish reference confirmation', async () => {
  const receive = session();
  await receive(marker);
  await receive({[0x027b]: 3}, 'readResponse');
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 10})).position, null);
  await receive({[0x027b]: 3});
  await receive({[0x027b]: 1, [0x0360]: 10}, 'readResponse');
  assert.equal((await receive({})).position, null);
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 10})).position, 10);
});

test('retained invalid state survives converter reload and still requires new reports', async () => {
  for (const calibrated of ['not_ready', 'error', 'in_progress']) {
    const receive = session({...state, position: null, calibrated, running_state: 'idle'});
    assert.equal((await receive({[0x027b]: 1, [0x0360]: 50})).position, null);
    await receive({[0x027b]: 3});
    assert.equal((await receive({[0x027b]: 1, [0x0360]: 50})).position, 50);
  }
});

test('real error interrupts completion and fresh progress resets earlier evidence', async () => {
  const receive = session();
  await receive(marker);
  await receive({[0x027b]: 3});
  await receive({[0x0360]: 20});
  assert.equal((await receive({[0x027b]: 2})).calibrated, 'error');
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 20})).calibrated, 'error');
  await receive({[0x027b]: 3});
  await receive({[0x027b]: 1});
  await receive({[0x027b]: 3});
  assert.equal((await receive({[0x0360]: 20})).position, null);
  assert.equal((await receive({[0x027b]: 1})).position, 20);
});

test('latched state cannot be overwritten through another native cluster', async () => {
  const receive = session();
  await receive(marker);
  assert.deepEqual(await receive({[0x027b]: 1, [0x0360]: 20}, 'attributeReport', 'hvacThermostat'),
    {...state, ...UNKNOWN});
});

test('successful confirmation respects Off without inventing a heating request', async () => {
  const receive = session({...state, system_mode: 'off'});
  await receive(marker);
  await receive({[0x027b]: 3});
  const confirmed = await receive({[0x027b]: 1, [0x0360]: 50});
  assert.equal(confirmed.calibrated, 'ready');
  assert.equal(confirmed.position, 50);
  assert.equal(confirmed.running_state, 'idle');
});

test('a delayed startup marker requires a new calibration after it arrives', async () => {
  const receive = session();
  await receive({[0x027b]: 3});
  await receive(marker);
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 0})).calibrated, 'not_ready');
  await receive({[0x027b]: 3});
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 0})).calibrated, 'ready');
});
