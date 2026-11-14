import assert from 'node:assert/strict';
import test from 'node:test';
import {UNKNOWN_POSITION, withReferenceState} from './w600_reference.mjs';

const DEVICE = '0x00124b0000000001';
const OTHER = '0x00124b0000000002';
const CUSTOM = {w600_custom_firmware: true};
const UNKNOWN = {...CUSTOM, position: null, calibrated: 'not_ready', running_state: 'idle'};
const marker = {[0x0360]: UNKNOWN_POSITION};
const state = {position: 24, calibrated: 'ready', running_state: 'heat', battery: 90};
const message = (data, ieee = DEVICE, type = 'attributeReport', cluster = 'manuSpecificLumi') =>
  ({cluster, type, device: {ieeeAddr: ieee}, data});

function fixture() {
  return {
    zigbeeModel: ['lumi.airrtc.aeu005'],
    model: 'W600',
    toZigbee: [{key: ['system_mode']}],
    ota: {supported: true},
    fromZigbee: [{cluster: 'manuSpecificLumi', convert: () => ({calibrated: 'ready'})}],
    extend: [{fromZigbee: [
      {cluster: 'manuSpecificLumi', convert: () => ({position: UNKNOWN_POSITION})},
      {cluster: 'manuSpecificLumi', convert: async () => ({running_state: 'heat'})},
    ]}],
  };
}
const converters = (definition) =>
  [...definition.fromZigbee, ...definition.extend.flatMap((part) => part.fromZigbee)];

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
  const wrapped = withReferenceState(native);
  return async (data, type = 'attributeReport', cluster = 'manuSpecificLumi') => {
    const result = await wrapped.fromZigbee[0].convert(null,
      message(data, DEVICE, type, cluster), () => {}, {}, {state: current});
    current = {...current, ...result};
    return current;
  };
}

test('stock positions, calibration errors, and progress retain native output unchanged', async () => {
  const native = {fromZigbee: [{convert: () => state}]};
  const wrapped = withReferenceState(native);
  for (const data of [{}, {[0x027b]: 0}, {[0x027b]: 1}, {[0x027b]: 2}, {[0x027b]: 3},
    ...[0, 17.5, 100, 6553.4, -1, NaN, Infinity, '6553.5', null].map((v) => ({[0x0360]: v}))]) {
    const result = await wrapped.fromZigbee[0].convert(null, message(data), () => {}, {}, {});
    assert.strictEqual(result, state);
  }
});

test('the custom marker activates automatically without an address list', async () => {
  const wrapped = withReferenceState({fromZigbee: [{convert: () => state}]});
  for (const ieee of [DEVICE, OTHER, DEVICE.toUpperCase()]) {
    assert.deepEqual(await wrapped.fromZigbee[0].convert(null, message(marker, ieee), () => {}, {}, {}),
      {...state, ...UNKNOWN});
  }
});

test('the marker on another cluster or without a device identity does not activate', async () => {
  const wrapped = withReferenceState({fromZigbee: [{convert: () => state}]});
  for (const msg of [message(marker, DEVICE, 'attributeReport', 'hvacThermostat'),
    {cluster: 'manuSpecificLumi', data: marker}]) {
    assert.strictEqual(await wrapped.fromZigbee[0].convert(null, msg, () => {}, {}, {}), state);
  }
});

test('custom detection does not affect another stock head', async () => {
  const wrapped = withReferenceState({fromZigbee: [{convert: () => state}]});
  await wrapped.fromZigbee[0].convert(null, message(marker), () => {}, {}, {});
  for (const data of [{[0x027b]: 2}, {[0x027b]: 3}, {[0x0360]: 50}]) {
    assert.strictEqual(await wrapped.fromZigbee[0].convert(null, message(data, OTHER), () => {}, {}, {}), state);
  }
});

test('root and modern-extend converters cannot overwrite the correction in either order', async () => {
  const original = fixture();
  const wrapped = withReferenceState(original);
  for (const reverse of [false, true]) {
    const ordered = converters(wrapped);
    if (reverse) ordered.reverse();
    const msg = message(marker);
    const merged = {};
    for (const converter of ordered) {
      Object.assign(merged, await converter.convert(null, msg, () => {}, {}, {}));
    }
    assert.deepEqual(merged, UNKNOWN);
  }
  assert.notStrictEqual(wrapped.fromZigbee, original.fromZigbee);
  assert.notStrictEqual(wrapped.extend[0], original.extend[0]);
  assert.strictEqual(wrapped.ota, original.ota);
  assert.strictEqual(wrapped.toZigbee, original.toZigbee);
  assert.strictEqual(wrapped.zigbeeModel, original.zigbeeModel);
});

test('published state, callback arguments, and converter context are preserved', async () => {
  const published = [];
  const context = {};
  const msg = message(marker);
  const options = {};
  const meta = {device: {ieeeAddr: DEVICE}};
  const wrapped = withReferenceState({fromZigbee: [{convert(model, received, publish, settings, metadata) {
    assert.strictEqual(this, context);
    assert.equal(model, 'model');
    assert.strictEqual(received, msg);
    assert.strictEqual(settings, options);
    assert.strictEqual(metadata, meta);
    publish({position: UNKNOWN_POSITION, running_state: 'heat'}, 'extra');
    return undefined;
  }}]});
  const result = await wrapped.fromZigbee[0].convert.call(context, 'model', msg,
    (...args) => published.push(args), options, meta);
  assert.deepEqual(result, UNKNOWN);
  assert.deepEqual(published, [[UNKNOWN, 'extra']]);
});

test('real error or progress alongside the marker is preserved', async () => {
  for (const [value, calibrated] of [[2, 'error'], [3, 'in_progress']]) {
    const receive = session();
    assert.deepEqual(await receive({...marker, [0x027b]: value}),
      {...state, ...CUSTOM, position: null, calibrated, running_state: 'idle'});
  }
});

test('marker latch rejects independent cached Ready and position frames', async () => {
  const receive = session();
  await receive(marker);
  for (const data of [{[0x027b]: 1}, {[0x0360]: 24}, {[0x027b]: 1, [0x0360]: 24}]) {
    assert.deepEqual(await receive(data), {...state, ...UNKNOWN});
  }
});

test('fresh progress plus Ready and position reports clears in either order', async () => {
  for (const reverse of [false, true]) {
    const receive = session();
    await receive(marker);
    await receive({[0x027b]: 3});
    const completion = [{[0x027b]: 1}, {[0x0360]: 37.5}];
    if (reverse) completion.reverse();
    assert.equal((await receive(completion[0])).position, null);
    assert.deepEqual(await receive(completion[1]), {...state, ...CUSTOM, position: 37.5, calibrated: 'ready'});
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

test('retained custom state survives reload and still requires new calibration reports', async () => {
  for (const calibrated of ['not_ready', 'error', 'in_progress']) {
    const receive = session({...state, ...CUSTOM, position: null, calibrated, running_state: 'idle'});
    assert.equal((await receive({[0x027b]: 1, [0x0360]: 50})).position, null);
    await receive({[0x027b]: 3});
    assert.equal((await receive({[0x027b]: 1, [0x0360]: 50})).position, 50);
  }
  const receive = session({...state, position: null, calibrated: 'not_ready'});
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 50})).position, 50);
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

test('another native cluster cannot overwrite latched state', async () => {
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
  const receive = session({...state, ...CUSTOM});
  await receive({[0x027b]: 3});
  await receive(marker);
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 0})).calibrated, 'not_ready');
  await receive({[0x027b]: 3});
  assert.equal((await receive({[0x027b]: 1, [0x0360]: 0})).calibrated, 'ready');
});
