import { useEffect, useMemo, useState } from 'react';
import { createDashboardApi } from '../lib/apiAdapter';
import type { AuditSession, DefectGeoJSON, LoadingState, Mission } from '../types/dashboard';

interface DashboardDataState {
  state: LoadingState;
  errorMessage: string | null;
  missions: Mission[];
  selectedMissionId: string;
  setSelectedMissionId: (missionId: string) => void;
  auditSessions: AuditSession[];
  defects: DefectGeoJSON;
  refetch: () => Promise<void>;
}

const emptyGeoJSON: DefectGeoJSON = { type: 'FeatureCollection', features: [] };

export const useDashboardData = (): DashboardDataState => {
  const api = useMemo(() => createDashboardApi(), []);
  const [state, setState] = useState<LoadingState>('idle');
  const [errorMessage, setErrorMessage] = useState<string | null>(null);
  const [missions, setMissions] = useState<Mission[]>([]);
  const [selectedMissionId, setSelectedMissionId] = useState('');
  const [auditSessions, setAuditSessions] = useState<AuditSession[]>([]);
  const [defects, setDefects] = useState<DefectGeoJSON>(emptyGeoJSON);

  const loadMissionBundle = async (missionId: string) => {
    const [sessions, defectLayer] = await Promise.all([
      api.getAuditSessions(missionId),
      api.getDefects(missionId)
    ]);

    setAuditSessions(sessions);
    setDefects(defectLayer);
  };

  const refetch = async () => {
    setState('loading');
    setErrorMessage(null);

    try {
      const loadedMissions = await api.getMissions();
      setMissions(loadedMissions);

      const missionId = selectedMissionId || loadedMissions[0]?.id;
      if (!missionId) {
        setAuditSessions([]);
        setDefects(emptyGeoJSON);
        setState('success');
        return;
      }

      setSelectedMissionId(missionId);
      await loadMissionBundle(missionId);
      setState('success');
    } catch (error) {
      setState('error');
      setErrorMessage(error instanceof Error ? error.message : 'Unknown error while loading dashboard data.');
    }
  };

  useEffect(() => {
    void refetch();
  }, []);

  useEffect(() => {
    if (!selectedMissionId) {
      return;
    }

    setState('loading');
    setErrorMessage(null);

    void loadMissionBundle(selectedMissionId)
      .then(() => setState('success'))
      .catch((error) => {
        setState('error');
        setErrorMessage(error instanceof Error ? error.message : 'Unknown error while loading mission details.');
      });
  }, [selectedMissionId]);

  return {
    state,
    errorMessage,
    missions,
    selectedMissionId,
    setSelectedMissionId,
    auditSessions,
    defects,
    refetch
  };
};
