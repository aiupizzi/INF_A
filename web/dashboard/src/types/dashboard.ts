export type Severity = 'low' | 'medium' | 'high' | 'critical';

export interface DefectFeatureProperties {
  id: string;
  className: string;
  confidence: number;
  severity: Severity;
  imageUrl: string;
  suggestedRepair: string;
  missionId: string;
  capturedAt: string;
}

export interface DefectFeature {
  type: 'Feature';
  geometry: {
    type: 'Point';
    coordinates: [number, number, number?];
  };
  properties: DefectFeatureProperties;
}

export interface DefectGeoJSON {
  type: 'FeatureCollection';
  features: DefectFeature[];
}

export interface Mission {
  id: string;
  name: string;
  modelAssetUrl: string;
  tilesManifestUrl?: string;
  createdAt: string;
}

export interface AuditSession {
  id: string;
  missionId: string;
  startedAt: string;
  finishedAt: string;
  operator: string;
  notes: string;
}

export interface DashboardFilters {
  severities: Severity[];
  classes: string[];
  minConfidence: number;
}

export type LoadingState = 'idle' | 'loading' | 'success' | 'error';
