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

  it('rejects malformed JSONL and non-object records with their line number', () => {
    expect(() => missionJsonlToCsv('{"type":"pose"}\n{bad')).toThrow('line 2');
    expect(() => missionJsonlToCsv('[]')).toThrow('line 1');
  });
});
