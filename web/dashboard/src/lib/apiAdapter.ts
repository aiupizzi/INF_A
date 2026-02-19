import { createClient, type SupabaseClient } from '@supabase/supabase-js';
import { mockAuditSessions, mockDefectGeoJSON, mockMissions } from '../data/mockData';
import type { AuditSession, DefectGeoJSON, Mission } from '../types/dashboard';

interface DashboardApi {
  getMissions: () => Promise<Mission[]>;
  getAuditSessions: (missionId: string) => Promise<AuditSession[]>;
  getDefects: (missionId: string) => Promise<DefectGeoJSON>;
}

const SUPABASE_URL = import.meta.env.VITE_SUPABASE_URL;
const SUPABASE_KEY = import.meta.env.VITE_SUPABASE_ANON_KEY;

class SupabaseAuditApi implements DashboardApi {
  private readonly client: SupabaseClient;

  constructor(client: SupabaseClient) {
    this.client = client;
  }

  async getMissions() {
    const { data, error } = await this.client
      .from('missions')
      .select('id,name,model_asset_url,tiles_manifest_url,created_at')
      .order('created_at', { ascending: false });

    if (error) {
      throw new Error(`Unable to load missions: ${error.message}`);
    }

    return (data ?? []).map((mission) => ({
      id: mission.id,
      name: mission.name,
      modelAssetUrl: mission.model_asset_url,
      tilesManifestUrl: mission.tiles_manifest_url,
      createdAt: mission.created_at
    }));
  }

  async getAuditSessions(missionId: string) {
    const { data, error } = await this.client
      .from('audit_logs')
      .select('id,mission_id,started_at,finished_at,operator,notes')
      .eq('mission_id', missionId)
      .order('started_at', { ascending: false });

    if (error) {
      throw new Error(`Unable to load audit sessions: ${error.message}`);
    }

    return (data ?? []).map((session) => ({
      id: session.id,
      missionId: session.mission_id,
      startedAt: session.started_at,
      finishedAt: session.finished_at,
      operator: session.operator,
      notes: session.notes
    }));
  }

  async getDefects(missionId: string) {
    const { data, error } = await this.client
      .from('defect_geojson')
      .select('payload')
      .eq('mission_id', missionId)
      .maybeSingle();

    if (error) {
      throw new Error(`Unable to load defect layer: ${error.message}`);
    }

    if (!data) {
      return { type: 'FeatureCollection', features: [] };
    }

    return data.payload as DefectGeoJSON;
  }
}

class MockDashboardApi implements DashboardApi {
  async getMissions() {
    await delay(250);
    return mockMissions;
  }

  async getAuditSessions(missionId: string) {
    await delay(200);
    return mockAuditSessions.filter((session) => session.missionId === missionId);
  }

  async getDefects(missionId: string) {
    await delay(300);
    return {
      type: 'FeatureCollection',
      features: mockDefectGeoJSON.features.filter((feature) => feature.properties.missionId === missionId)
    };
  }
}

const delay = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

export const createDashboardApi = (): DashboardApi => {
  if (SUPABASE_URL && SUPABASE_KEY) {
    const client = createClient(SUPABASE_URL, SUPABASE_KEY);
    return new SupabaseAuditApi(client);
  }

  return new MockDashboardApi();
};
