import type { DashboardFilters, DefectFeature, Severity } from '../types/dashboard';

interface FilterControlsProps {
  filters: DashboardFilters;
  defects: DefectFeature[];
  onChange: (next: DashboardFilters) => void;
}

const severities: Severity[] = ['low', 'medium', 'high', 'critical'];

export const FilterControls = ({ filters, defects, onChange }: FilterControlsProps) => {
  const classes = Array.from(new Set(defects.map((item) => item.properties.className)));

  return (
    <section className="card">
      <h2>Filters</h2>
      <div className="filter-grid">
        <fieldset>
          <legend>Severity</legend>
          {severities.map((severity) => (
            <label key={severity}>
              <input
                type="checkbox"
                checked={filters.severities.includes(severity)}
                onChange={() => {
                  const nextSeverities = filters.severities.includes(severity)
                    ? filters.severities.filter((value) => value !== severity)
                    : [...filters.severities, severity];
                  onChange({ ...filters, severities: nextSeverities });
                }}
              />
              {severity}
            </label>
          ))}
        </fieldset>

        <fieldset>
          <legend>Defect class</legend>
          {classes.map((className) => (
            <label key={className}>
              <input
                type="checkbox"
                checked={filters.classes.includes(className)}
                onChange={() => {
                  const nextClasses = filters.classes.includes(className)
                    ? filters.classes.filter((value) => value !== className)
                    : [...filters.classes, className];
                  onChange({ ...filters, classes: nextClasses });
                }}
              />
              {className}
            </label>
          ))}
        </fieldset>

        <fieldset>
          <legend>Minimum confidence</legend>
          <input
            type="range"
            min={0}
            max={1}
            step={0.01}
            value={filters.minConfidence}
            onChange={(event) => onChange({ ...filters, minConfidence: Number(event.target.value) })}
          />
          <output>{Math.round(filters.minConfidence * 100)}%</output>
        </fieldset>
      </div>
    </section>
  );
};
