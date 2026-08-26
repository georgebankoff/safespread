import { BleManager, Device } from 'react-native-ble-plx';
import { Buffer } from 'buffer';
import { buildPosePacket } from './protocol';

const DEVICE_NAME = 'SafeSpread';
const NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';
const NUS_TX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';

export type ConnectionStatus = 'disconnected' | 'scanning' | 'connected';

export class SafeSpreadBLE {
  private manager = new BleManager();
  private device: Device | null = null;

  connect(onStatusChange: (status: ConnectionStatus) => void): Promise<void> {
    onStatusChange('scanning');
    return new Promise((resolve, reject) => {
      this.manager.startDeviceScan([NUS_SERVICE_UUID], null, async (error, scanned) => {
        if (error) {
          onStatusChange('disconnected');
          reject(error);
          return;
        }
        if (scanned?.name !== DEVICE_NAME) return;

        this.manager.stopDeviceScan();
        try {
          const device = await scanned.connect();
          await device.discoverAllServicesAndCharacteristics();
          this.device = device;
          onStatusChange('connected');
          device.onDisconnected(() => {
            this.device = null;
            onStatusChange('disconnected');
          });
          resolve();
        } catch (e) {
          onStatusChange('disconnected');
          reject(e);
        }
      });
    });
  }

  async disconnect(): Promise<void> {
    this.manager.stopDeviceScan();
    if (this.device) {
      await this.device.cancelConnection();
      this.device = null;
    }
  }

  async sendPose(x: number, y: number, heading: number): Promise<void> {
    if (!this.device) return;
    const packet = buildPosePacket(x, y, heading);
    await this.device.writeCharacteristicWithoutResponseForService(
      NUS_SERVICE_UUID,
      NUS_TX_UUID,
      Buffer.from(packet).toString('base64')
    );
  }

  async sendCommand(command: '1' | '2'): Promise<void> {
    if (!this.device) return;
    const packet = new Uint8Array([command.charCodeAt(0)]);
    await this.device.writeCharacteristicWithoutResponseForService(
      NUS_SERVICE_UUID,
      NUS_TX_UUID,
      Buffer.from(packet).toString('base64')
    );
  }
}
