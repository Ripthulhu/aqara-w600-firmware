import {definitions} from 'zigbee-herdsman-converters/devices/lumi';
import {withReferenceState} from '../custom/w600_reference.mjs';

// Replace this example with the IEEE addresses of your custom-firmware heads only.
const customDevices = ['0x00124b0000000001'];
if (customDevices.includes('0x00124b0000000001') ||
    customDevices.some((ieee) => !/^0x[0-9a-f]{16}$/.test(ieee))) {
  throw new Error('Set customDevices to your custom-firmware heads before installing');
}

const native = definitions.find((definition) => definition.zigbeeModel?.includes('lumi.airrtc.aeu005'));
if (!native) throw new Error('Installed W600 definition is missing');
const {zigbeeModel, fingerprint, ...base} = native;

export default withReferenceState({
  ...base,
  fingerprint: customDevices.map((ieee) => ({
    modelID: 'lumi.airrtc.aeu005',
    ieeeAddr: new RegExp(`^${ieee}$`),
  })),
}, customDevices);
