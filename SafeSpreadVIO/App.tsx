import React, { useEffect, useState } from 'react';
import { Keyboard, Pressable, StyleSheet, Text, TextInput, View } from 'react-native';
import { useVIOPose } from './src/useVIOPose';
import { ConnectionStatus, SafeSpreadBLE } from './src/ble';

const ble = new SafeSpreadBLE();

export default function App() {
  const { pose, trackingState, trackingOk, zero } = useVIOPose();
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');
  const [widthText, setWidthText] = useState('21.9');
  const [lengthText, setLengthText] = useState('21.9');
  const [areaNote, setAreaNote] = useState('');

  const applyArea = () => {
    Keyboard.dismiss();
    const w = parseFloat(widthText);
    const l = parseFloat(lengthText);
    if (!isFinite(w) || !isFinite(l) || w <= 0 || l <= 0) {
      setAreaNote('Enter positive numbers');
      return;
    }
    ble.sendArea(w, l);
    setAreaNote(`Sent ${w} x ${l} ft (${Math.round(w * l)} sqft)`);
  };

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
        <Text style={styles.text}>
          Tracking: {trackingState}
          {trackingOk ? '' : ' (not sending)'}
        </Text>
      </View>

      <View style={styles.areaPanel}>
        <Text style={styles.label}>Area to cover (ft)</Text>
        <View style={styles.areaRow}>
          <TextInput
            style={styles.input}
            value={widthText}
            onChangeText={setWidthText}
            keyboardType="decimal-pad"
            placeholder="width"
            placeholderTextColor="#888"
          />
          <Text style={styles.times}>×</Text>
          <TextInput
            style={styles.input}
            value={lengthText}
            onChangeText={setLengthText}
            keyboardType="decimal-pad"
            placeholder="length"
            placeholderTextColor="#888"
          />
          <Pressable style={styles.areaButton} onPress={applyArea}>
            <Text style={styles.buttonText}>Set</Text>
          </Pressable>
        </View>
        {areaNote ? <Text style={styles.note}>{areaNote}</Text> : null}
      </View>
      <View style={styles.buttons}>
        <Pressable
          style={styles.button}
          onPress={() => {
            zero();
            ble.sendCommand('1');
          }}
        >
          <Text style={styles.buttonText}>Start / Reset</Text>
        </Pressable>
        <Pressable style={styles.button} onPress={() => ble.sendCommand('2')}>
          <Text style={styles.buttonText}>Stop</Text>
        </Pressable>
        <Pressable style={styles.testButton} onPress={() => ble.sendCommand('3')}>
          <Text style={styles.buttonText}>Self Test</Text>
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
  areaPanel: {
    position: 'absolute',
    top: 200,
    left: 20,
    right: 20,
    backgroundColor: 'rgba(0,0,0,0.55)',
    padding: 12,
    borderRadius: 8,
  },
  label: { color: 'white', fontSize: 14, fontWeight: '600', marginBottom: 8 },
  areaRow: { flexDirection: 'row', alignItems: 'center' },
  input: {
    flex: 1,
    backgroundColor: '#222',
    color: 'white',
    fontSize: 16,
    paddingVertical: 8,
    paddingHorizontal: 10,
    borderRadius: 6,
  },
  times: { color: 'white', fontSize: 18, marginHorizontal: 8 },
  areaButton: {
    backgroundColor: '#1565c0',
    paddingVertical: 10,
    paddingHorizontal: 16,
    borderRadius: 6,
    marginLeft: 10,
  },
  note: { color: '#9ecbff', fontSize: 13, marginTop: 8 },
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
  testButton: { backgroundColor: '#1565c0', paddingVertical: 14, paddingHorizontal: 20, borderRadius: 8 },
  buttonText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
