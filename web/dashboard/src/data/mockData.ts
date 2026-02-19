import type { AuditSession, DefectGeoJSON, Mission } from '../types/dashboard';

export const mockMissions: Mission[] = [
  {
    id: 'mission-041',
    name: 'Bridge Span 7 - East Side',
    modelAssetUrl: '/assets/reconstruction/model.glb',
    tilesManifestUrl: '/assets/reconstruction/tileset.json',
    createdAt: '2026-01-21T08:10:00Z'
  },
  {
    id: 'mission-040',
    name: 'Bridge Span 7 - West Side',
    modelAssetUrl: '/assets/reconstruction/model-west.glb',
    tilesManifestUrl: '/assets/reconstruction/tileset-west.json',
    createdAt: '2026-01-14T08:10:00Z'
  }
];

export const mockAuditSessions: AuditSession[] = [
  {
    id: 'audit-1003',
    missionId: 'mission-041',
    startedAt: '2026-01-21T09:00:00Z',
    finishedAt: '2026-01-21T09:35:00Z',
    operator: 'A. Chen',
    notes: 'Detected localized spalling near expansion joint.'
  },
  {
    id: 'audit-1002',
    missionId: 'mission-041',
    startedAt: '2026-01-22T15:12:00Z',
    finishedAt: '2026-01-22T15:50:00Z',
    operator: 'J. Singh',
    notes: 'Longitudinal crack progression increased by approximately 8%.'
  },
  {
    id: 'audit-0990',
    missionId: 'mission-040',
    startedAt: '2026-01-14T10:05:00Z',
    finishedAt: '2026-01-14T10:40:00Z',
    operator: 'A. Chen',
    notes: 'No severe defects; recommend routine monitoring.'
  }
];

export const mockDefectGeoJSON: DefectGeoJSON = {
  type: 'FeatureCollection',
  features: [
    {
      type: 'Feature',
      geometry: { type: 'Point', coordinates: [12.2, 2.4, 1.6] },
      properties: {
        id: 'd-001',
        className: 'crack',
        confidence: 0.92,
        severity: 'high',
        imageUrl: 'https://images.unsplash.com/photo-1581094288338-2314dddb7ece?auto=format&fit=crop&w=1200&q=80',
        suggestedRepair: 'Seal crack and inject epoxy; schedule follow-up in 30 days.',
        missionId: 'mission-041',
        capturedAt: '2026-01-21T09:10:00Z'
      }
    },
    {
      type: 'Feature',
      geometry: { type: 'Point', coordinates: [7.8, 1.2, 1.1] },
      properties: {
        id: 'd-002',
        className: 'spalling',
        confidence: 0.84,
        severity: 'critical',
        imageUrl: 'https://images.unsplash.com/photo-1460574283810-2aab119d8511?auto=format&fit=crop&w=1200&q=80',
        suggestedRepair: 'Chip loose concrete, apply corrosion inhibitor, patch with polymer mortar.',
        missionId: 'mission-041',
        capturedAt: '2026-01-21T09:16:00Z'
      }
    },
    {
      type: 'Feature',
      geometry: { type: 'Point', coordinates: [4.1, 2.9, 0.8] },
      properties: {
        id: 'd-003',
        className: 'corrosion',
        confidence: 0.63,
        severity: 'medium',
        imageUrl: 'https://images.unsplash.com/photo-1429497419816-9ca5cfb4571a?auto=format&fit=crop&w=1200&q=80',
        suggestedRepair: 'Clean exposed steel and apply anti-corrosion coating.',
        missionId: 'mission-040',
        capturedAt: '2026-01-14T10:15:00Z'
      }
    }
  ]
};
