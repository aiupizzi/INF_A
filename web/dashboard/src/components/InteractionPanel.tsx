import type { DefectFeature } from '../types/dashboard';

interface InteractionPanelProps {
  defect?: DefectFeature;
}

export const InteractionPanel = ({ defect }: InteractionPanelProps) => {
  if (!defect) {
    return (
      <section className="card detail-card">
        <h2>Defect Details</h2>
        <p>Select a defect pin to inspect high-resolution evidence and repair guidance.</p>
      </section>
    );
  }

  const { properties } = defect;

  return (
    <section className="card detail-card">
      <h2>Defect Details</h2>
      <img src={properties.imageUrl} alt={`Defect ${properties.id}`} className="detail-image" />
      <dl>
        <div>
          <dt>ID</dt>
          <dd>{properties.id}</dd>
        </div>
        <div>
          <dt>Class</dt>
          <dd>{properties.className}</dd>
        </div>
        <div>
          <dt>Confidence</dt>
          <dd>{Math.round(properties.confidence * 100)}%</dd>
        </div>
        <div>
          <dt>Severity</dt>
          <dd>{properties.severity}</dd>
        </div>
      </dl>
      <h3>Suggested repair action</h3>
      <p>{properties.suggestedRepair}</p>
    </section>
  );
};
