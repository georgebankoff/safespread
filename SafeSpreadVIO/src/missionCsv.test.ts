import { MISSION_CSV_COLUMNS, missionJsonlToCsv } from './missionCsv';

describe('missionJsonlToCsv', () => {
  it('uses stable replay columns and leaves missing optional fields empty', () => {
    const jsonl = [
      JSON.stringify({ type: 'metadata', missionId: 'm1' }),
      JSON.stringify({
        type: 'pose', phoneMs: 10, sequence: 2, epoch: 7,
        xFt: 1.5, yFt: -2, headingDeg: 359, speedFps: 0.8,
        yawRateDps: -1.25, trackingValid: true,
      }),
    ].join('\n');
    const lines = missionJsonlToCsv(jsonl).trimEnd().split('\n');
    expect(lines[0]).toBe(MISSION_CSV_COLUMNS.join(','));
    expect(lines[1]).toBe('10,2,7,1.5,-2,359,0.8,-1.25,true,,,,,,');
  });

  it('quotes commas, quotes, and newlines', () => {
    const csv = missionJsonlToCsv(JSON.stringify({
      type: 'fault', phoneMs: 2, fault: 'bad, "sensor"\nretry',
    }));
    expect(csv).toContain('"bad, ""sensor""\nretry"');
  });

  it('exports a firmware fault-buffer code through the replay fault column', () => {
    const csv = missionJsonlToCsv(JSON.stringify({
      type: 'fault_buffer', phoneMs: 100, sequence: 2, epoch: 42,
      speedFps: 1.1, routeIndex: 7, crossTrackFt: 0.2,
      headingErrorDeg: 1.5, steeringUs: 1700, throttleUs: 1600,
      faultCode: 9,
    }));
    expect(csv.trimEnd().split('\n')[1]).toBe(
      '100,2,42,,,,1.1,,,7,0.2,1.5,1700,1600,9',
    );
  });

  it('rejects malformed JSONL and non-object records with their line number', () => {
    expect(() => missionJsonlToCsv('{"type":"pose"}\n{bad')).toThrow('line 2');
    expect(() => missionJsonlToCsv('[]')).toThrow('line 1');
  });
});
