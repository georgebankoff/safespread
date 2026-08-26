import React from 'react';
import { Modal, Pressable, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { PathPoint, computeViewBox, projector } from './pathMath';

interface Props {
  visible: boolean;
  onClose: () => void;
  widthFt: number;
  lengthFt: number;
  points: PathPoint[];
}

const DOT = 4;

export default function PathMap({ visible, onClose, widthFt, lengthFt, points }: Props) {
  const { width: screenW, height: screenH } = useWindowDimensions();
  const canvasW = screenW - 40;
  const canvasH = screenH - 240;

  const box = computeViewBox(widthFt, lengthFt, points);
  const { toPx, scale } = projector(box, canvasW, canvasH);

  const rectTopLeft = toPx(0, lengthFt);
  const rectW = widthFt * scale;
  const rectH = lengthFt * scale;

  const sprayed = points.filter((p) => p.spraying).length;

  return (
    <Modal visible={visible} animationType="slide" onRequestClose={onClose}>
      <View style={styles.container}>
        <Text style={styles.title}>
          {lengthFt.toFixed(1)} × {widthFt.toFixed(1)} ft
        </Text>
        <Text style={styles.legend}>
          <Text style={styles.spraySwatch}>■</Text> sprayed ({sprayed}) {'   '}
          <Text style={styles.travelSwatch}>■</Text> travelling ({points.length - sprayed})
        </Text>

        <View style={[styles.canvas, { width: canvasW, height: canvasH }]}>
          {/* The rectangle asked for */}
          <View
            style={[
              styles.target,
              {
                left: rectTopLeft.left,
                top: rectTopLeft.top,
                width: rectW,
                height: rectH,
              },
            ]}
          />

          {points.map((p, i) => {
            const { left, top } = toPx(p.x, p.y);
            return (
              <View
                key={i}
                style={[
                  styles.dot,
                  p.spraying ? styles.dotSpray : styles.dotTravel,
                  { left: left - DOT / 2, top: top - DOT / 2 },
                ]}
              />
            );
          })}

          {/* Where the rover started: the corner everything is measured from */}
          {(() => {
            const o = toPx(0, 0);
            return <View style={[styles.origin, { left: o.left - 5, top: o.top - 5 }]} />;
          })()}
        </View>

        {points.length === 0 ? (
          <Text style={styles.empty}>No path recorded yet — run a mission.</Text>
        ) : null}

        <Pressable
          style={({ pressed }) => [styles.close, pressed && { opacity: 0.6 }]}
          onPress={onClose}
        >
          <Text style={styles.closeText}>Close</Text>
        </Pressable>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#111', paddingTop: 60, alignItems: 'center' },
  title: { color: 'white', fontSize: 18, fontWeight: '700', marginBottom: 4 },
  legend: { color: '#bbb', fontSize: 13, marginBottom: 12 },
  spraySwatch: { color: '#ff5252' },
  travelSwatch: { color: '#5a5a5a' },
  canvas: { backgroundColor: '#000', borderRadius: 6, overflow: 'hidden' },
  target: {
    position: 'absolute',
    borderWidth: 2,
    borderColor: '#2e7d32',
    backgroundColor: 'rgba(46,125,50,0.12)',
  },
  dot: { position: 'absolute', width: DOT, height: DOT, borderRadius: DOT / 2 },
  dotSpray: { backgroundColor: '#ff5252' },
  dotTravel: { backgroundColor: '#5a5a5a' },
  origin: {
    position: 'absolute',
    width: 10,
    height: 10,
    borderRadius: 5,
    backgroundColor: '#ffd54f',
  },
  empty: { color: '#888', marginTop: 16 },
  close: {
    marginTop: 'auto',
    marginBottom: 40,
    backgroundColor: '#1565c0',
    paddingVertical: 14,
    paddingHorizontal: 40,
    borderRadius: 8,
  },
  closeText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
