const mockManager = {
  startDeviceScan: jest.fn(),
  stopDeviceScan: jest.fn(),
};

jest.mock('react-native-ble-plx', () => ({
  BleManager: jest.fn(() => mockManager),
}));

import { SafeSpreadBLE } from './ble';

describe('SafeSpreadBLE connection ownership', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('cancels a delayed connection without installing stale monitors or reporting connected', async () => {
    let scanCallback!: (error: Error | null, device: unknown) => void;
    mockManager.startDeviceScan.mockImplementation((_services, _options, callback) => {
      scanCallback = callback;
    });
    let releaseConnect!: () => void;
    const delayedConnect = new Promise<void>((resolve) => { releaseConnect = resolve; });
    const connectedDevice = {
      discoverAllServicesAndCharacteristics: jest.fn(async () => {}),
      monitorCharacteristicForService: jest.fn(),
      onDisconnected: jest.fn(),
      cancelConnection: jest.fn(async () => {}),
    };
    const scannedDevice = {
      name: 'SafeSpread',
      connect: jest.fn(async () => {
        await delayedConnect;
        return connectedDevice;
      }),
    };
    const statuses: string[] = [];
    const ble = new SafeSpreadBLE();
    const connecting = ble.connect((status) => statuses.push(status));
    scanCallback(null, scannedDevice);
    await ble.disconnect();
    releaseConnect();

    await expect(connecting).rejects.toThrow(/cancel/i);
    await Promise.resolve();
    expect(connectedDevice.cancelConnection).toHaveBeenCalled();
    expect(connectedDevice.monitorCharacteristicForService).not.toHaveBeenCalled();
    expect(statuses).toEqual(['scanning']);
  });
});
