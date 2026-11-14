// Custom-controller marker, outside the normal valve-position range.
export const UNKNOWN_POSITION = 6553.5;
const POSITION_ATTRIBUTE = 0x0360;
const CALIBRATION_ATTRIBUTE = 0x027b;

export function withReferenceState(definition) {
  const references = new Map();
  const frames = new WeakMap();
  const corrections = (msg, meta) => {
    const ieee = (msg.device?.ieeeAddr ?? meta?.device?.ieeeAddr)?.toLowerCase();
    if (!ieee) return undefined;
    if (frames.has(msg)) return frames.get(msg);
    const raw = msg.cluster === 'manuSpecificLumi' ? msg.data ?? {} : {};
    if (!references.has(ieee)) {
      const retained = meta?.state;
      // shortcut: install before OTA so the startup marker establishes custom firmware.
      if (raw[POSITION_ATTRIBUTE] !== UNKNOWN_POSITION && retained?.w600_custom_firmware !== true) {
        return undefined;
      }
      const invalid = retained?.position === null &&
        ['not_ready', 'error', 'in_progress'].includes(retained.calibrated);
      references.set(ieee, invalid ? {status: retained.calibrated, progress: false} : null);
    }
    let reference = references.get(ieee);
    const calibration = raw[CALIBRATION_ATTRIBUTE];
    const report = msg.type === 'attributeReport';
    if (calibration === 2) {
      reference = {status: 'error', progress: false};
    } else if (calibration === 3) {
      if (reference?.status !== 'in_progress') reference = {status: 'in_progress', progress: false};
      if (report) {
        reference.progress = true;
        reference.ready = false;
        delete reference.position;
      }
    } else if (raw[POSITION_ATTRIBUTE] === UNKNOWN_POSITION) {
      reference = {status: 'not_ready', progress: false};
    }
    let result;
    if (reference) {
      // Cache reads cannot prove completion of a new calibration.
      if (report && reference.progress) {
        if (calibration === 1) reference.ready = true;
        const position = raw[POSITION_ATTRIBUTE];
        if (typeof position === 'number' && Number.isFinite(position) && position >= 0 && position <= 100) {
          reference.position = position;
        }
      }
      if (reference.ready && reference.position !== undefined) {
        const off = raw[0x0271] === 0 || (raw[0x0271] === undefined && meta?.state?.system_mode === 'off');
        result = {position: reference.position, calibrated: 'ready',
          running_state: !off && reference.position > 0 ? 'heat' : 'idle'};
        reference = null;
      } else {
        result = {position: null, calibrated: reference.status, running_state: 'idle'};
      }
    }
    references.set(ieee, reference);
    result = {w600_custom_firmware: true, ...result};
    frames.set(msg, result);
    return result;
  };
  const wrap = (converter) => ({
    ...converter,
    async convert(model, msg, publish, options, meta) {
      const correction = corrections(msg, meta);
      const normalize = (state) => correction ? {...state, ...correction} : state;
      const wrappedPublish = (...args) => publish(normalize(args[0]), ...args.slice(1));
      return normalize(await converter.convert.call(this, model, msg, wrappedPublish, options, meta));
    },
  });
  const wrapPart = (part) => ({
    ...part,
    ...(part.fromZigbee && {fromZigbee: part.fromZigbee.map(wrap)}),
  });
  return {
    ...wrapPart(definition),
    ...(definition.extend && {extend: definition.extend.map(wrapPart)}),
  };
}
