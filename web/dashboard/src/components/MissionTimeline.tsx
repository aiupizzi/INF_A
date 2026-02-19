import type { AuditSession, Mission } from '../types/dashboard';

interface MissionTimelineProps {
  missions: Mission[];
  selectedMissionId: string;
  onMissionChange: (missionId: string) => void;
  sessions: AuditSession[];
}

export const MissionTimeline = ({
  missions,
  selectedMissionId,
  onMissionChange,
  sessions
}: MissionTimelineProps) => {
  return (
    <section className="card">
      <div className="mission-header">
        <h2>Mission & Audit Timeline</h2>
        <select value={selectedMissionId} onChange={(event) => onMissionChange(event.target.value)}>
          {missions.map((mission) => (
            <option key={mission.id} value={mission.id}>
              {mission.name}
            </option>
          ))}
        </select>
      </div>

      <ol className="timeline">
        {sessions.map((session) => (
          <li key={session.id}>
            <time>{new Date(session.startedAt).toLocaleString()}</time>
            <p>
              <strong>{session.operator}</strong> — {session.notes}
            </p>
          </li>
        ))}
      </ol>
    </section>
  );
};
