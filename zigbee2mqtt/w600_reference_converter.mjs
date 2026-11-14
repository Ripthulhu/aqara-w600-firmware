import {definitions} from 'zigbee-herdsman-converters/devices/lumi';
import {withReferenceState} from './w600_reference.mjs';

const native = definitions.find((definition) => definition.zigbeeModel?.includes('lumi.airrtc.aeu005'));
if (!native) throw new Error('Installed W600 definition is missing');

export default withReferenceState(native);
