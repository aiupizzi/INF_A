import { useEffect, useMemo, useState } from 'react';
import { FilterControls } from './components/FilterControls';
import { InteractionPanel } from './components/InteractionPanel';
import { LoadStateBoundary } from './components/LoadStateBoundary';
import { MissionTimeline } from './components/MissionTimeline';
import { SceneViewport } from './components/SceneViewport';
import { useDashboardData } from './hooks/useDashboardData';
import type { DashboardFilters, DefectFeature } from './types/dashboard';

const defaultFilters: DashboardFilters = {
  severities: ['low', 'medium', 'high', 'critical'],
  classes: [],
  minConfidence: 0
};

function App() {
  const { state, errorMessage, missions, selectedMissionId, setSelectedMissionId, auditSessions, defects, refetch } =
    useDashboardData();

  const [filters, setFilters] = useState<DashboardFilters>(defaultFilters);
  const [selectedDefect, setSelectedDefect] = useState<DefectFeature | undefined>(undefined);

  const availableClasses = useMemo(
    () => Array.from(new Set(defects.features.map((item) => item.properties.className))),
    [defects]
  );

  const effectiveFilters = useMemo(
    () => ({
      ...filters,
      classes: filters.classes.length === 0 ? availableClasses : filters.classes
    }),
    [filters, availableClasses]
  );

  const filteredDefects = useMemo(
    () => ({
      ...defects,
      features: defects.features.filter((feature) => {
        const { severity, className, confidence } = feature.properties;
        return (
          effectiveFilters.severities.includes(severity) &&
          effectiveFilters.classes.includes(className) &&
          confidence >= effectiveFilters.minConfidence
        );
      })
    }),
    [defects, effectiveFilters]
  );

  const currentMission = missions.find((mission) => mission.id === selectedMissionId);

  useEffect(() => {
    if (selectedDefect && !filteredDefects.features.some((item) => item.properties.id === selectedDefect.properties.id)) {
      setSelectedDefect(undefined);
    }
  }, [filteredDefects, selectedDefect]);

  return (
    <main className="dashboard">
      <h1>Inspection Reconstruction Dashboard</h1>
      <LoadStateBoundary state={state} errorMessage={errorMessage} onRetry={refetch}>
        <MissionTimeline
          missions={missions}
          selectedMissionId={selectedMissionId}
          onMissionChange={setSelectedMissionId}
          sessions={auditSessions}
        />

        <FilterControls filters={effectiveFilters} defects={defects.features} onChange={setFilters} />

        {currentMission ? (
          <SceneViewport
            modelAssetUrl={currentMission.modelAssetUrl}
            tilesManifestUrl={currentMission.tilesManifestUrl}
            defects={filteredDefects}
            selectedDefectId={selectedDefect?.properties.id}
            onSelectDefect={setSelectedDefect}
          />
        ) : (
          <section className="card state-card">
            <p>No mission selected.</p>
          </section>
        )}

        <InteractionPanel defect={selectedDefect} />
      </LoadStateBoundary>
    </main>
  );
}

export default App;
