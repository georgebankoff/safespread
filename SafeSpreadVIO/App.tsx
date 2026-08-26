import React, { useEffect, useRef, useState } from 'react';
import {
  Keyboard,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import { useVIOPose } from './src/useVIOPose';
import { ConnectionStatus, SafeSpreadBLE } from './src/ble';

const ble = new SafeSpreadBLE();

export default function App() {
  const { pose, trackingState, trackingOk, zero } = useVIOPose();
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');
  const [widthText, setWidthText] = useState('21.9');
  const [lengthText, setLengthText] = useState('21.9');
  const [areaNote, setAreaNote] = useState('');
  const [telemetry, setTelemetry] = useState('');
  const [log, setLog] = useState<string[]>([]);
  const [dryRun, setDryRun] = useState(false);
  const [spraying, setSpraying] = useState(false);
  const logRef = useRef<ScrollView>(null);

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
    const handleLog = (line: string) => {
      // Telemetry is a once-per-second heartbeat; pin it rather than letting
      // it scroll the interesting mission messages away.
      if (line.startsWith('[TLM]')) {
        setTelemetry(line.slice(5).trim());
      } else if (line === '[SPRAY] ON' || line === '[SPRAY] OFF') {
        setSpraying(line.endsWith('ON'));
      } else if (line === '[MODE] DRY' || line === '[MODE] WET') {
        setDryRun(line.endsWith('DRY'));
      } else {
        setLog((prev) => [...prev.slice(-40), line]);
      }
    };
    ble.connect(setStatus, handleLog).catch(() => {});
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
    <View style={[styles.container, spraying && styles.containerSpraying]}>
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
          <Pressable
            style={({ pressed }) => [styles.areaButton, pressed && styles.pressed]}
            onPress={applyArea}
          >
            <Text style={styles.buttonText}>Set</Text>
          </Pressable>
        </View>
        {areaNote ? <Text style={styles.note}>{areaNote}</Text> : null}
      </View>

      <View style={styles.roverPanel}>
        <Text style={styles.label}>Rover</Text>
        <Text style={styles.telemetry}>{telemetry || 'no telemetry yet'}</Text>
        <ScrollView
          style={styles.logScroll}
          ref={logRef}
          onContentSizeChange={() => logRef.current?.scrollToEnd({ animated: false })}
        >
          {log.map((line, idx) => (
            <Text key={idx} style={styles.logLine}>
              {line}
            </Text>
          ))}
        </ScrollView>
      </View>
      <View style={styles.modeRow}>
        <Pressable
          style={({ pressed }) => [
            styles.modeButton,
            dryRun ? styles.modeDry : styles.modeWet,
            pressed && styles.pressed,
          ]}
          onPress={() => ble.sendCommand('4')}
        >
          <Text style={styles.buttonText}>
            {dryRun ? 'DRY RUN — no spray' : 'WET — spraying enabled'}
          </Text>
        </Pressable>
      </View>
      <View style={styles.buttons}>
        <Pressable
          style={({ pressed }) => [styles.button, pressed && styles.pressed]}
          onPress={() => {
            zero();
            ble.sendCommand('1');
          }}
        >
          <Text style={styles.buttonText}>Start / Reset</Text>
        </Pressable>
        <Pressable
          style={({ pressed }) => [styles.button, pressed && styles.pressed]}
          onPress={() => ble.sendCommand('2')}
        >
          <Text style={styles.buttonText}>Stop</Text>
        </Pressable>
        <Pressable
          style={({ pressed }) => [styles.testButton, pressed && styles.pressed]}
          onPress={() => ble.sendCommand('3')}
        >
          <Text style={styles.buttonText}>Self Test</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: 'black' },
  containerSpraying: { backgroundColor: '#b00020' },
  hud: { position: 'absolute', top: 60, left: 20 },
  // Pressed feedback: buttons dim while a finger is down.
  pressed: { opacity: 0.6 },
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
  roverPanel: {
    position: 'absolute',
    top: 330,
    bottom: 164,
    left: 20,
    right: 20,
    backgroundColor: 'rgba(0,0,0,0.55)',
    padding: 12,
    borderRadius: 8,
  },
  telemetry: {
    color: '#7fffa0',
    fontSize: 13,
    fontFamily: 'Menlo',
    marginBottom: 8,
  },
  logScroll: { flex: 1 },
  logLine: { color: '#ddd', fontSize: 12, fontFamily: 'Menlo', marginBottom: 2 },
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
  modeRow: {
    position: 'absolute',
    bottom: 104,
    left: 20,
    right: 20,
  },
  modeButton: { paddingVertical: 12, borderRadius: 8, alignItems: 'center' },
  modeDry: { backgroundColor: '#6a1b9a' },
  modeWet: { backgroundColor: '#ef6c00' },
  buttonText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
