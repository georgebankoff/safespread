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
import { useKeepAwake } from 'expo-keep-awake';
import { setAudioModeAsync, useAudioPlayer } from 'expo-audio';
import { useVIOPose } from './src/useVIOPose';
import { ConnectionStatus, SafeSpreadBLE } from './src/ble';
import PathMap from './src/PathMap';
import { PathPoint, MAX_PATH_POINTS, shouldRecord } from './src/pathMath';

const ble = new SafeSpreadBLE();

export default function App() {
  // The screen locking would suspend ARKit and the BLE writes with it,
  // stranding the rover mid-pass until its VIO timeout stops it.
  useKeepAwake();

  const { pose, trackingState, trackingOk } = useVIOPose();

  // In dry mode nothing is dispensed, so the beep is the only audible cue that
  // the rover thinks it is spraying -- useful when it is across the yard.
  const beep = useAudioPlayer(require('./assets/spray-beep.wav'));
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');
  const [nText, setNText] = useState('21.9');
  const [mText, setMText] = useState('21.9');
  const [areaNote, setAreaNote] = useState('');
  const [telemetry, setTelemetry] = useState('');
  const [log, setLog] = useState<string[]>([]);
  const [dryRun, setDryRun] = useState(false);
  const [spraying, setSpraying] = useState(false);
  const [running, setRunning] = useState(false);
  const [sentCount, setSentCount] = useState(0);
  const [path, setPath] = useState<PathPoint[]>([]);
  const [mapOpen, setMapOpen] = useState(false);
  const [mapDims, setMapDims] = useState({ n: 21.9, m: 21.9 });
  const logRef = useRef<ScrollView>(null);

  const applyArea = () => {
    Keyboard.dismiss();
    const n = parseFloat(nText);
    const m = parseFloat(mText);
    if (!isFinite(n) || !isFinite(m) || n <= 0 || m <= 0) {
      setAreaNote('Enter positive numbers');
      return;
    }
    ble.sendArea(n, m);
    setAreaNote(`Sent ${n} × ${m} ft (${Math.round(n * m)} sqft)`);
  };

  // Keep beeping even with the ringer switch flipped to silent, which is where
  // a phone strapped to a rover usually lives.
  useEffect(() => {
    setAudioModeAsync({ playsInSilentMode: true }).catch(() => {});
    beep.loop = true;
  }, []);

  useEffect(() => {
    if (dryRun && spraying) {
      beep.seekTo(0);
      beep.play();
    } else {
      beep.pause();
    }
  }, [dryRun, spraying]);

  useEffect(() => {
    const handleLog = (line: string) => {
      // Telemetry is a once-per-second heartbeat; pin it rather than letting
      // it scroll the interesting mission messages away.
      if (line.startsWith('[TLM]')) {
        const body = line.slice(5).trim();
        setTelemetry(body);
        // Turn phases are just as "running" as a pass; only IDLE and DONE
        // mean Start is safe to press again.
        const phase = body.split(' ')[0];
        setRunning(phase !== 'IDLE' && phase !== 'DONE');
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

  // Read through refs, and depend on nothing: `pose` is a fresh object every
  // ARKit frame (~60Hz), so listing it here would clear and recreate the
  // interval every ~16ms and the 100ms tick would never once fire.
  const sendStateRef = useRef({ pose, status, trackingOk, spraying, running });
  sendStateRef.current = { pose, status, trackingOk, spraying, running };

  useEffect(() => {
    const timer = setInterval(() => {
      const current = sendStateRef.current;
      if (current.pose && current.status === 'connected' && current.trackingOk) {
        ble.sendPose(current.pose.x, current.pose.y, current.pose.heading);
        setSentCount((n) => n + 1);
      }

      // Trace the mission, including the headland turns, so the map shows the
      // real path and not just the sprayed parts.
      if (current.pose && current.running) {
        const next: PathPoint = {
          x: current.pose.x,
          y: current.pose.y,
          spraying: current.spraying,
        };
        setPath((prev) =>
          shouldRecord(prev[prev.length - 1], next)
            ? [...prev.slice(-(MAX_PATH_POINTS - 1)), next]
            : prev
        );
      }
    }, 100);
    return () => clearInterval(timer);
  }, []);

  return (
    <View style={[styles.container, spraying && styles.containerSpraying]}>
      <View style={styles.hud}>
        <Text style={styles.text}>X: {pose ? `${pose.x.toFixed(1)} ft` : '—'}</Text>
        <Text style={styles.text}>Y: {pose ? `${pose.y.toFixed(1)} ft` : '—'}</Text>
        <Text style={styles.text}>Hdg: {pose ? `${pose.heading.toFixed(0)}°` : '—'}</Text>
        <Text style={styles.text}>BLE: {status}</Text>
        <Text style={styles.text}>
          Tracking: {trackingState}
          {trackingOk ? '' : ' (not sending)'}
        </Text>
        <Text style={styles.text}>Sent: {sentCount}</Text>
      </View>

      <View style={styles.areaPanel}>
        <Text style={styles.label}>
          Area (ft) — N: straight ahead, M: out to the right
        </Text>
        <View style={styles.areaRow}>
          <TextInput
            style={styles.input}
            value={nText}
            onChangeText={setNText}
            keyboardType="decimal-pad"
            placeholder="N"
            placeholderTextColor="#888"
          />
          <Text style={styles.times}>×</Text>
          <TextInput
            style={styles.input}
            value={mText}
            onChangeText={setMText}
            keyboardType="decimal-pad"
            placeholder="M"
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
          disabled={running}
          style={({ pressed }) => [
            styles.button,
            running && styles.disabled,
            pressed && styles.pressed,
          ]}
          onPress={() => {
            // The old trace belongs to the previous run; rectangle-frame
            // coordinates are established by setup, not by this pose hook.
            setPath([]);
            setMapDims({ n: parseFloat(nText) || 0, m: parseFloat(mText) || 0 });
            ble.sendCommand('1');
          }}
        >
          <Text style={styles.buttonText}>{running ? 'Running…' : 'Start'}</Text>
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
        <Pressable
          style={({ pressed }) => [styles.mapButton, pressed && styles.pressed]}
          onPress={() => setMapOpen(true)}
        >
          <Text style={styles.buttonText}>Map</Text>
        </Pressable>
      </View>

      <PathMap
        visible={mapOpen}
        onClose={() => setMapOpen(false)}
        widthFt={mapDims.m}
        lengthFt={mapDims.n}
        points={path}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: 'black' },
  containerSpraying: { backgroundColor: '#b00020' },
  hud: { position: 'absolute', top: 60, left: 20 },
  // Pressed feedback: buttons dim while a finger is down.
  pressed: { opacity: 0.6 },
  disabled: { backgroundColor: '#444' },
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
    gap: 10,
  },
  // Equal widths so the row stays balanced regardless of label length
  // ("Start" vs "Running…" vs "Self Test").
  button: {
    flex: 1,
    backgroundColor: '#2e7d32',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
  },
  testButton: {
    flex: 1,
    backgroundColor: '#1565c0',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
  },
  mapButton: {
    flex: 1,
    backgroundColor: '#455a64',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
  },
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
