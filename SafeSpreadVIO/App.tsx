import React, { useEffect, useState } from 'react';
import { Pressable, StyleSheet, Text, View } from 'react-native';
import { useVIOPose } from './src/useVIOPose';
import { ConnectionStatus, SafeSpreadBLE } from './src/ble';

const ble = new SafeSpreadBLE();

export default function App() {
  const { pose, trackingOk, zero } = useVIOPose();
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');

  useEffect(() => {
    ble.connect(setStatus).catch(() => {});
    return () => {
      ble.disconnect();
    };
  }, []);

  useEffect(() => {
    const timer = setInterval(() => {
      if (status === 'connected' && trackingOk) {
        ble.sendPose(pose.x, pose.y, pose.heading);
      }
    }, 100);
    return () => clearInterval(timer);
  }, [status, trackingOk, pose]);

  return (
    <View style={styles.container}>
      <View style={styles.crosshair} />
      <View style={styles.hud}>
        <Text style={styles.text}>X: {pose.x.toFixed(1)} ft</Text>
        <Text style={styles.text}>Y: {pose.y.toFixed(1)} ft</Text>
        <Text style={styles.text}>Hdg: {pose.heading.toFixed(0)}°</Text>
        <Text style={styles.text}>BLE: {status}</Text>
        <Text style={styles.text}>Tracking: {trackingOk ? 'OK' : 'DEGRADED'}</Text>
      </View>
      <View style={styles.buttons}>
        <Pressable style={styles.button} onPress={zero}>
          <Text style={styles.buttonText}>Start / Reset</Text>
        </Pressable>
        <Pressable style={styles.button} onPress={() => ble.disconnect()}>
          <Text style={styles.buttonText}>Stop</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: 'black' },
  crosshair: {
    position: 'absolute',
    top: '50%',
    left: '50%',
    width: 40,
    height: 40,
    marginLeft: -20,
    marginTop: -20,
    borderWidth: 2,
    borderColor: 'white',
    borderRadius: 20,
  },
  hud: { position: 'absolute', top: 60, left: 20 },
  text: { color: 'white', fontSize: 18, fontWeight: '600' },
  buttons: {
    position: 'absolute',
    bottom: 40,
    left: 20,
    right: 20,
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  button: { backgroundColor: '#2e7d32', paddingVertical: 14, paddingHorizontal: 24, borderRadius: 8 },
  buttonText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
